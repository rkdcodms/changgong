"""Mission runtime glue: launcher command, config, local event socket. No devices."""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'jetson'))
sys.path.insert(0, str(ROOT / 'tests'))
import mission_server  # noqa: E402
from mission import MissionConfig  # noqa: E402


def load_script(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'scripts' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


run_mission = load_script('team_run_mission_test', 'run_mission.py')


class LauncherTests(unittest.TestCase):
    def config(self):
        return {'bind': '100.64.0.10', 'port': 8765, 'allow_tailscale': True, 'allowed_clients': [],
                'camera': '/dev/video-test', 'rotate_ccw': 90, 'serial': '/dev/serial/by-id/example-mega',
                'weights': '/x/best.pt', 'confidence': .55, 'stop_zone_left': .4, 'stop_zone_right': .6,
                'max_seconds': 0}

    def test_command_targets_mission_server_and_keeps_device_settings(self):
        cmd = run_mission.command(self.config(), Path('/r/plan.json'), Path('/r/m.json'), executable='py')
        self.assertEqual(cmd[:5], ['py', '-u', str(ROOT / 'jetson/mission_server.py'), '--plan', '/r/plan.json'])
        self.assertNotIn('vision_stop_server.py', ' '.join(cmd))
        self.assertNotIn('--max-seconds', cmd)
        for flag, value in (('--serial', '/dev/serial/by-id/example-mega'), ('--rotate-ccw', '90'),
                            ('--confidence', '0.55'), ('--mission-config', '/r/m.json')):
            self.assertEqual(cmd[cmd.index(flag) + 1], value)
        self.assertIn('--allow-tailscale', cmd)
        args = mission_server.parse_args(cmd[3:])
        self.assertEqual(args.serial, '/dev/serial/by-id/example-mega')

    def test_server_rejects_public_bind(self):
        with self.assertRaises(SystemExit):
            mission_server.parse_args(['--plan', 'p', '--serial', '/dev/x', '--bind', '0.0.0.0', '--allow-tailscale'])

    def test_mission_config_example_and_unknown_fields(self):
        cfg = mission_server.load_config(ROOT / 'config/mission.example.json')
        self.assertEqual(cfg, MissionConfig().validate())
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'm.json'
            p.write_text(json.dumps({'departure_policy': 'events', 'auto_restart': 5}))
            with self.assertRaises(ValueError):
                mission_server.load_config(p)


class FakeRunner:
    def __init__(self):
        self.motion, self.got, self.ticks = 'READY', [], 0

    def submit(self, event):
        self.got.append(event)
        return event.get('kind') == 'start', 'ok' if event.get('kind') == 'start' else 'nope'

    def tick(self):
        self.ticks += 1

    def drain(self):
        return []


class Hub:
    def __init__(self, runner):
        self.runner, self.runner_lock, self.stopped = runner, threading.Lock(), threading.Event()


class SocketTests(unittest.TestCase):
    def test_event_cli_round_trip_and_bad_json(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'mission.sock'
            sock = mission_server.open_event_socket(path)
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
            hub = Hub(FakeRunner())
            worker = mission_server.MissionWorker(hub, sock, logger=type('L', (), {'info': lambda *a: None})())
            worker.start()
            try:
                out = subprocess.run([sys.executable, str(ROOT / 'scripts/mission_event.py'), 'start',
                                      '--socket', str(path)], capture_output=True, text=True, timeout=10)
                self.assertEqual(out.returncode, 0, out.stderr)
                self.assertTrue(json.loads(out.stdout)['accepted'])
                out = subprocess.run([sys.executable, str(ROOT / 'scripts/mission_event.py'), 'crew_done',
                                      '--socket', str(path)], capture_output=True, text=True, timeout=10)
                self.assertNotEqual(out.returncode, 0, 'crew_done without a trial number is refused')
                event = hub.runner.got[0]
                self.assertEqual((event['kind'], event['source']), ('start', 'operator'))
                self.assertLess(abs(event['created_unix'] - time.time()), 5)
                client = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
                reply = Path(d) / 'r.sock'; client.bind(str(reply)); client.settimeout(2)
                client.sendto(b'\xff not json', str(path))
                self.assertEqual(json.loads(client.recv(1024))['reason'], 'bad_json')
                client.close()
                time.sleep(.2)
                self.assertGreater(hub.runner.ticks, 1)
            finally:
                hub.stopped.set(); worker.join(2); sock.close()

    def test_no_reply_is_reported_as_failure(self):
        with tempfile.TemporaryDirectory() as d:
            out = subprocess.run([sys.executable, str(ROOT / 'scripts/mission_event.py'), 'start',
                                  '--socket', str(Path(d) / 'missing.sock')], capture_output=True, text=True, timeout=10)
            self.assertEqual(out.returncode, 1)
            self.assertIn('Nothing was confirmed', out.stderr)


if __name__ == '__main__':
    unittest.main()
