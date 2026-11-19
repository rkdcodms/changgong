"""Field test record: stop error, recognition, comm errors, task time, resets.

Append one measured result per command to a local JSONL file (Git-ignored),
then summarise. Only what was measured is stored; nothing is inferred.

  python3 tools/field_log.py add stop --statue Scylla --error-mm 12 --on-mark yes --load deck+3crew
  python3 tools/field_log.py add recognition --statue Siren --result missed --lux dim
  python3 tools/field_log.py add comm --error heartbeat_fault --note "USB cable bumped"
  python3 tools/field_log.py add task --task crew_transfer --seconds 14.2 --crew 2
  python3 tools/field_log.py add reset --cause line_lost_fault
  python3 tools/field_log.py add bump --load deck+3crew --result pass --pwm-scale 0.6
  python3 tools/field_log.py add rehearsal --seconds 288 --odysseus yes --score 61
  python3 tools/field_log.py import-mission runtime/mission_events.jsonl
  python3 tools/field_log.py summary
"""
from __future__ import annotations

import argparse
import datetime
import json
from pathlib import Path
import statistics
import sys

ROOT = Path(__file__).resolve().parents[1]
DEFAULT = ROOT / 'runtime' / 'field_tests.jsonl'
YES = {'yes': True, 'no': False}

KINDS = {
    'stop': {'statue': str, 'error_mm': float, 'on_mark': 'bool'},
    'recognition': {'statue': str, 'result': ('hit', 'missed', 'wrong_class', 'false_stop')},
    'comm': {'error': ('heartbeat_fault', 'ack_timeout', 'session_mismatch', 'serial_disconnect',
                      'camera_fail', 'mega_reset', 'bt_dropout', 'ppm_dropout', 'other')},
    'task': {'task': str, 'seconds': float},
    'reset': {'cause': str},
    'bump': {'load': str, 'result': ('pass', 'line_lost', 'stalled', 'object_fell', 'fault')},
    'rehearsal': {'seconds': float, 'odysseus': 'bool', 'score': float},
}
OPTIONAL = {'note': str, 'load': str, 'lux': str, 'speed': str, 'crew': int, 'trial': int,
            'pwm_scale': float, 'firmware': str, 'commit': str, 'confidence': float, 'resets': int}


def _convert(name, kind, raw):
    if kind == 'bool':
        if raw not in YES:
            raise ValueError(f'--{name} must be yes or no')
        return YES[raw]
    if isinstance(kind, tuple):
        if raw not in kind:
            raise ValueError(f'--{name} must be one of {", ".join(kind)}')
        return raw
    try:
        return kind(raw)
    except ValueError:
        raise ValueError(f'--{name} must be {kind.__name__}') from None


def make_record(kind, values, now=None):
    if kind not in KINDS:
        raise ValueError(f'Unknown record kind {kind}')
    row = {'type': kind, 'recorded_at': (now or datetime.datetime.now().astimezone()).isoformat(timespec='seconds')}
    for name, spec in KINDS[kind].items():
        if values.get(name) is None:
            raise ValueError(f'{kind} needs --{name.replace("_", "-")}')
        row[name] = _convert(name, spec, values[name])
    for name, spec in OPTIONAL.items():
        if values.get(name) is not None and name not in row:
            row[name] = _convert(name, spec, values[name])
    if kind == 'stop' and row['error_mm'] < 0:
        raise ValueError('error_mm is a distance (>= 0); record the side in --note')
    return row


def import_mission(path):
    """Turn mission_events.jsonl timing rows into task records (source=mission_log)."""
    rows = []
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        start = line.find('{"mission"')
        if start < 0:
            continue
        try:
            m = json.loads(line[start:])['mission']
        except (ValueError, KeyError):
            continue
        if m.get('event') == 'stop_confirmed':
            rows.append({'type': 'task', 'task': 'seek_to_statue', 'seconds': m['seek_s'], 'trial': m['trial'],
                         'source': 'mission_log'})
        elif m.get('event') == 'crew_done':
            rows.append({'type': 'task', 'task': 'stop_to_crew_done', 'seconds': m['wait_s'], 'trial': m['trial'],
                         'source': 'mission_log'})
        elif m.get('event') == 'motion' and m.get('motion') == 'DEPARTING' and 'dwell_s' in m:
            rows.append({'type': 'task', 'task': 'dwell_at_statue', 'seconds': m['dwell_s'], 'trial': m['trial'],
                         'source': 'mission_log'})
        elif m.get('event') == 'motion' and m.get('motion') == 'FAULT':
            rows.append({'type': 'comm', 'error': 'other', 'note': m.get('reason', ''), 'source': 'mission_log'})
    return rows


def _stats(values):
    if not values:
        return 'n=0'
    text = f'n={len(values)} mean={statistics.mean(values):.2f} max={max(values):.2f} min={min(values):.2f}'
    if len(values) > 1:
        text += f' sd={statistics.stdev(values):.2f}'
    return text


def summarize(rows):
    out = []
    stops = [r for r in rows if r['type'] == 'stop']
    if stops:
        on = sum(r['on_mark'] for r in stops)
        out.append(f'STOP  on-mark {on}/{len(stops)}  error_mm {_stats([r["error_mm"] for r in stops])}')
        for statue in sorted({r['statue'] for r in stops}):
            s = [r for r in stops if r['statue'] == statue]
            out.append(f'  {statue:<15} on-mark {sum(r["on_mark"] for r in s)}/{len(s)}  {_stats([r["error_mm"] for r in s])}')
    rec = [r for r in rows if r['type'] == 'recognition']
    if rec:
        out.append('RECOGNITION (counts, not accuracy of the model in general)')
        for statue in sorted({r['statue'] for r in rec}):
            s = [r['result'] for r in rec if r['statue'] == statue]
            out.append(f'  {statue:<15} ' + ' '.join(f'{k}={s.count(k)}' for k in KINDS['recognition']['result']))
    for kind, key in (('comm', 'error'), ('reset', 'cause'), ('bump', 'result')):
        group = [r for r in rows if r['type'] == kind]
        if group:
            counts = {}
            for r in group:
                counts[r[key]] = counts.get(r[key], 0) + 1
            out.append(f'{kind.upper():<6}' + ' '.join(f'{k}={v}' for k, v in sorted(counts.items())))
    bumps = [r for r in rows if r['type'] == 'bump']
    if bumps:
        streak = best = 0
        for r in bumps:
            streak = streak + 1 if r['result'] == 'pass' else 0
            best = max(best, streak)
        out.append(f'  bump consecutive passes: current={streak} best={best}')
    tasks = [r for r in rows if r['type'] == 'task']
    for task in sorted({r['task'] for r in tasks}):
        out.append(f'TASK  {task:<20} {_stats([r["seconds"] for r in tasks if r["task"] == task])}')
    reh = [r for r in rows if r['type'] == 'rehearsal']
    if reh:
        ok = sum(r['odysseus'] for r in reh)
        out.append(f'REHEARSAL odysseus {ok}/{len(reh)}  seconds {_stats([r["seconds"] for r in reh])}  '
                   f'score {_stats([r["score"] for r in reh])}')
    return '\n'.join(out) if out else 'No records.'


def read_rows(path):
    if not Path(path).exists():
        return []
    return [json.loads(line) for line in Path(path).read_text(encoding='utf-8').splitlines() if line.strip()]


def append(path, rows):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with Path(path).open('a', encoding='utf-8') as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False) + '\n')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--file', type=Path, default=DEFAULT)
    sub = ap.add_subparsers(dest='command', required=True)
    add = sub.add_parser('add')
    add.add_argument('kind', choices=sorted(KINDS))
    names = {n for spec in KINDS.values() for n in spec} | set(OPTIONAL)
    for name in sorted(names):
        add.add_argument('--' + name.replace('_', '-'), dest=name)
    imp = sub.add_parser('import-mission')
    imp.add_argument('log', type=Path)
    sub.add_parser('summary')
    args = ap.parse_args(argv)
    try:
        if args.command == 'add':
            row = make_record(args.kind, vars(args))
            append(args.file, [row])
            print(json.dumps(row, ensure_ascii=False))
        elif args.command == 'import-mission':
            rows = import_mission(args.log)
            append(args.file, rows)
            print(f'Imported {len(rows)} rows from {args.log}')
        else:
            print(summarize(read_rows(args.file)))
    except (OSError, ValueError) as exc:
        ap.exit(2, f'Not recorded: {exc}\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
