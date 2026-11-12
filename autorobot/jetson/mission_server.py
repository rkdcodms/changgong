"""AutoRobot contest program: YOLO target stop + trial state machine + MS1 Mega.

Start only with arduino/mission_stop (MS1) on the Mega; the link refuses to
send a heartbeat to any other firmware. Camera, serial and web threads are the
ones from vision_stop_server.py (PR #2) unchanged; this file adds the target
filter, the trial plan and a local event socket.

The robot does NOT move when this program starts. It waits braked until a
``start`` event (scripts/mission_event.py start) is accepted.
"""
from __future__ import annotations

import argparse
import dataclasses
import datetime
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import secrets
import signal
import socket
import threading
import time

from mission import MissionConfig, MissionRunner
from mission_link import MissionLinkController
from mission_plan import load_plan
from vision_stop_server import DEFAULT_WEIGHTS, Detector, SerialWorker, VisionHub
from web_preview import PreviewServer, TAILSCALE_NETWORK
import ipaddress

VERSION = 'mission-v1'
MAX_DATAGRAM = 1024


class MissionHub(VisionHub):
    def __init__(self, controller, runner, **kwargs):
        super().__init__(controller, **kwargs)
        self.runner = runner
        self.runner_lock = threading.Lock()

    def status(self):
        state = super().status()
        with self.runner_lock:
            state['mission'] = self.runner.status(time.monotonic())
        state['version'] = VERSION
        return state


def load_config(path):
    if path is None:
        return MissionConfig()
    data = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    known = {f.name for f in dataclasses.fields(MissionConfig)}
    unknown = set(data) - known
    if unknown:
        raise ValueError(f'Unknown mission config fields: {sorted(unknown)}')
    if 'allowed_sources' in data:
        data['allowed_sources'] = tuple(data['allowed_sources'])
    return MissionConfig(**data).validate()


class MissionWorker(threading.Thread):
    """Owns the runner: local events in, periodic tick, JSONL log out."""
    def __init__(self, hub, sock, logger, period=.05):
        super().__init__(name='mission', daemon=True)
        self.hub, self.sock, self.logger, self.period = hub, sock, logger, period

    def run(self):
        runner = self.hub.runner
        while not self.hub.stopped.is_set():
            started = time.monotonic()
            while True:
                try:
                    data, peer = self.sock.recvfrom(MAX_DATAGRAM)
                except BlockingIOError:
                    break
                reply = self._handle_event(data)
                if peer:
                    try:
                        self.sock.sendto(json.dumps(reply).encode(), peer)
                    except OSError:
                        pass
            with self.hub.runner_lock:
                runner.tick()
                rows = runner.drain()
            for row in rows:
                self.logger.info(json.dumps({'mission': row}, ensure_ascii=False, allow_nan=False))
            time.sleep(max(0., self.period - (time.monotonic() - started)))

    def _handle_event(self, data):
        try:
            event = json.loads(data.decode('utf-8'))
            if not isinstance(event, dict):
                raise ValueError
        except (UnicodeDecodeError, ValueError):
            return {'accepted': False, 'reason': 'bad_json'}
        with self.hub.runner_lock:
            accepted, reason = self.hub.runner.submit(event)
            return {'accepted': accepted, 'reason': reason, 'motion': self.hub.runner.motion}


def open_event_socket(path):
    path = Path(path)
    if path.exists():
        path.unlink()             # stale socket from a previous run; the file is ours
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    old = os.umask(0o177)
    try:
        sock.bind(str(path))      # 0600: only this Jetson account can send events
    finally:
        os.umask(old)
    sock.setblocking(False)
    return sock


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--plan', type=Path, required=True)
    ap.add_argument('--mission-config', type=Path)
    ap.add_argument('--bind', required=True)
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--allow-client', action='append', default=[])
    ap.add_argument('--allow-tailscale', action='store_true')
    ap.add_argument('--camera', default='/dev/video0')
    ap.add_argument('--rotate-ccw', type=int, choices=(0, 90, 180, 270), default=0)
    ap.add_argument('--stop-zone-left', type=float, default=.4)
    ap.add_argument('--stop-zone-right', type=float, default=.6)
    ap.add_argument('--serial', required=True)
    ap.add_argument('--weights', type=Path, default=DEFAULT_WEIGHTS)
    ap.add_argument('--confidence', type=float, default=.55)
    ap.add_argument('--event-socket', type=Path, default=Path('mission.sock'))
    ap.add_argument('--status-file', type=Path)
    ap.add_argument('--log-file', type=Path, default=Path('mission_events.jsonl'))
    args = ap.parse_args(argv)
    try:
        bind = ipaddress.IPv4Address(args.bind)
        args.allow_client = [str(ipaddress.IPv4Address(v)) for v in args.allow_client]
    except ValueError:
        ap.error('Bind and explicit viewers must be valid IPv4 addresses')
    if not (bind in TAILSCALE_NETWORK or bind.is_loopback):
        ap.error('Bind only a specific Tailscale or loopback IPv4 address')
    if not args.allow_tailscale and not args.allow_client:
        ap.error('Specify --allow-tailscale or at least one --allow-client')
    if args.allow_tailscale and bind not in TAILSCALE_NETWORK:
        ap.error('--allow-tailscale requires binding the specific Tailscale IPv4 address')
    if not 0 < args.confidence <= 1 or not 0 <= args.stop_zone_left < args.stop_zone_right <= 1:
        ap.error('Invalid confidence or stop zone')
    args.bind = str(bind)
    return args


def main(argv=None):
    args = parse_args(argv)
    plan = load_plan(args.plan)
    config = load_config(args.mission_config)
    args.log_file.parent.mkdir(parents=True, exist_ok=True)
    settings = args.log_file.parent / 'yolo_settings'
    (settings / 'Ultralytics').mkdir(parents=True, exist_ok=True)
    os.environ.update(YOLO_CONFIG_DIR=str(settings), YOLO_AUTOINSTALL='false', YOLO_OFFLINE='true', MPLBACKEND='Agg')
    logger = logging.getLogger(VERSION)
    logger.setLevel(logging.INFO)
    handler = RotatingFileHandler(args.log_file, maxBytes=5_000_000, backupCount=5, encoding='utf-8')
    handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
    logger.addHandler(handler)

    core = MissionLinkController(session=secrets.randbelow(0xffffffff) + 1, confidence=args.confidence,
                                 min_span=.1, min_frames=3, frame_freshness=.5)
    runner = MissionRunner(plan, core, config)
    stop_zone = (args.stop_zone_left, args.stop_zone_right)
    hub = MissionHub(core, runner, max_seconds=0, rotation_ccw=args.rotate_ccw, stop_zone=stop_zone)
    server = PreviewServer((args.bind, args.port), hub, args.allow_client,
                           allowed_networks=(TAILSCALE_NETWORK,) if args.allow_tailscale else ())
    server.timeout = .1
    sock = open_event_socket(args.event_socket)
    serial = SerialWorker(core, hub, args.serial, logger)
    detector = Detector(core, hub, args.weights, args.camera, logger, args.rotate_ccw, stop_zone)
    mission = MissionWorker(hub, sock, logger)
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: hub.stop())
    logger.info(json.dumps({'event': 'started', 'version': VERSION, 'session': core.session,
                            'plan': plan.to_json(), 'config': dataclasses.asdict(config)}))
    print('PLAN:', plan.summary(), flush=True)
    print('Robot stays braked until: scripts/mission_event.py start', flush=True)
    serial.start(); detector.start(); mission.start()
    last_status = 0.
    try:
        while not hub.stopped.is_set():
            server.handle_request()
            now = time.monotonic()
            if args.status_file and now - last_status >= .25:
                state = hub.status()
                state['updated_at_utc'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
                args.status_file.parent.mkdir(parents=True, exist_ok=True)
                tmp = args.status_file.with_name(args.status_file.name + '.tmp')
                tmp.write_text(json.dumps(state, ensure_ascii=False, allow_nan=False, default=str), encoding='utf-8')
                os.replace(tmp, args.status_file)
                last_status = now
    finally:
        hub.stop()
        core.fail('PROGRAM_STOPPED', time.monotonic())
        server.server_close()
        serial.join(timeout=2); detector.join(timeout=3); mission.join(timeout=2)
        sock.close()
        try:
            args.event_socket.unlink()
        except OSError:
            pass
        logger.info(json.dumps({'event': 'stopped'}))
        handler.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
