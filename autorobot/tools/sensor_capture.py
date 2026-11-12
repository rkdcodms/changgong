"""Record and analyse the Mega's eight line-sensor ADC rows (A0..A7, 100 ms).

Purpose: dark stop tape, 5 mm speed bump and deck/crew load tests. It only
READS the existing tab-separated telemetry; it never sends a byte to the Mega.

SAFETY: opening a USB serial port can RESET the Mega. With MS1 or vision_stop
firmware the robot then waits braked for a Jetson heartbeat, so it cannot move
while this tool runs. With the original line_tracer (auto start) firmware the
robot drives after reset: lift the wheels or switch motor power off first.
Do not run this while the mission/vision program owns the same port.

  python3 tools/sensor_capture.py record --serial /dev/serial/by-id/... --seconds 20 --label dark_tape_A --out runtime/tape_A.csv
  python3 tools/sensor_capture.py analyze runtime/tape_A.csv
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys
import time

THRESHOLD = 500           # firmware: analogRead <= 500 is white
CHANNELS = [f'A{i}' for i in range(8)]   # A0 = robot right ... A7 = robot left


def parse_row(line):
    parts = line.strip().split('\t')
    if len(parts) != 8 or not all(p.isdigit() and len(p) <= 4 for p in parts):
        return None
    values = [int(p) for p in parts]
    return values if all(0 <= v <= 1023 for v in values) else None


def analyze(rows, period=.1):
    """rows: list of (t, [8 values]). Returns a dict of per-channel and gap facts."""
    if not rows:
        return {'samples': 0}
    per = []
    for i, name in enumerate(CHANNELS):
        col = [v[i] for _, v in rows]
        white = [x for x in col if x <= THRESHOLD]
        dark = [x for x in col if x > THRESHOLD]
        per.append({'channel': name, 'min': min(col), 'max': max(col),
                    'white_max': max(white) if white else None, 'dark_min': min(dark) if dark else None,
                    'margin_to_threshold': min([abs(x - THRESHOLD) for x in col])})
    counts = [sum(x <= THRESHOLD for x in v) for _, v in rows]
    gaps, run_start = [], None
    for (t, _), c in zip(rows, counts):
        if c == 0 and run_start is None:
            run_start = t
        elif c != 0 and run_start is not None:
            gaps.append(round(t - run_start, 3)); run_start = None
    if run_start is not None:
        gaps.append(round(rows[-1][0] - run_start + period, 3))
    histogram = {k: counts.count(k) for k in range(9) if counts.count(k)}
    return {'samples': len(rows), 'per_channel': per, 'white_count_histogram': histogram,
            'all_dark_gaps_s': gaps, 'all_white_samples': counts.count(8),
            'note': ('All-dark gaps make FOLLOW enter SEARCH (unknown direction -> FAULT after ~0.17 s). '
                     'Compare the longest gap with GAP_COAST_MS before enabling it. '
                     'Telemetry is every 100 ms, so gaps shorter than that may be missed.')}


def read_csv(path):
    rows = []
    with Path(path).open(encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            rows.append((float(row['t']), [int(row[c]) for c in CHANNELS]))
    return rows


def record(port, seconds, out, label):
    import serial  # pyserial, only needed for recording
    link = serial.Serial(port=None, baudrate=115200, timeout=.1)
    link.dtr = False; link.rts = False; link.port = port
    link.open()
    start = time.monotonic(); rows = 0
    try:
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(['t', 'label', *CHANNELS])
            buffer = b''
            while time.monotonic() - start < seconds:
                buffer += link.read(256)
                *lines, buffer = buffer.split(b'\n')
                for raw in lines:
                    values = parse_row(raw.decode('ascii', 'replace'))
                    if values:
                        writer.writerow([f'{time.monotonic() - start:.3f}', label, *values]); rows += 1
    finally:
        link.close()
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='command', required=True)
    rec = sub.add_parser('record')
    rec.add_argument('--serial', required=True)
    rec.add_argument('--seconds', type=float, default=20)
    rec.add_argument('--label', required=True, help='e.g. dark_tape_A, bump_left, deck_3crew')
    rec.add_argument('--out', type=Path, required=True)
    an = sub.add_parser('analyze')
    an.add_argument('csv', type=Path)
    args = ap.parse_args(argv)
    if args.command == 'record':
        n = record(args.serial, args.seconds, args.out, args.label)
        print(f'{n} sensor rows -> {args.out}')
        return 0 if n else 1
    import json
    print(json.dumps(analyze(read_csv(args.csv)), indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
