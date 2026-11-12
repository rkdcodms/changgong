"""Validate local robot configuration, or explicitly start the Jetson service.

--check reads configuration/model files only. It never opens a camera or serial
port, resets the Mega, sends heartbeats, or starts the robot process.
"""
from __future__ import annotations
import argparse
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path
import shlex
import stat
import sys

ROOT = Path(__file__).resolve().parents[1]
TAILNET = ipaddress.IPv4Network('100.64.0.0/10')
WEIGHTS_HASH = 'e67df5f04399513f8578a6948843902ea31b346c558375af88e919834283bd84'
FIELDS = {'bind', 'port', 'allow_tailscale', 'allowed_clients', 'camera', 'rotate_ccw',
          'serial', 'weights', 'confidence', 'stop_zone_left', 'stop_zone_right', 'max_seconds'}

def numeric(value, name, low, high, integer=False):
    valid = type(value) is int if integer else type(value) in (int, float)
    if not valid or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name} must be between {low} and {high}' + (' and an integer' if integer else ''))
    return value

def validate(config, root=ROOT):
    if not isinstance(config, dict):
        raise ValueError('Configuration must be a JSON object')
    missing, unknown = FIELDS - config.keys(), config.keys() - FIELDS
    if missing or unknown:
        raise ValueError(f'Missing fields: {sorted(missing)}; unknown fields: {sorted(unknown)}')
    c = dict(config)
    try:
        address = ipaddress.IPv4Address(c['bind'])
    except (ValueError, TypeError):
        raise ValueError('Set bind to the Jetson Tailscale IPv4 address from tailscale ip -4') from None
    if address not in TAILNET and not address.is_loopback:
        raise ValueError('bind must be one specific Tailscale or loopback IPv4; 0.0.0.0 is not allowed')
    if type(c['allow_tailscale']) is not bool:
        raise ValueError('allow_tailscale must be true or false')
    if c['allow_tailscale'] and address not in TAILNET:
        raise ValueError('allow_tailscale requires a Tailscale bind address')
    if not isinstance(c['allowed_clients'], list):
        raise ValueError('allowed_clients must be a list of IPv4 addresses')
    try:
        c['allowed_clients'] = [str(ipaddress.IPv4Address(value)) for value in c['allowed_clients'] if isinstance(value,str)]
    except ValueError:
        raise ValueError('Every allowed_clients entry must be an IPv4 address') from None
    if len(c['allowed_clients']) != len(config['allowed_clients']):
        raise ValueError('Every allowed_clients entry must be an IPv4 string')
    if not c['allow_tailscale'] and not c['allowed_clients']:
        raise ValueError('Enable allow_tailscale or specify at least one allowed client')
    numeric(c['port'], 'port', 1, 65535, True)
    numeric(c['rotate_ccw'], 'rotate_ccw', 0, 270, True)
    if c['rotate_ccw'] not in (0, 90, 180, 270):
        raise ValueError('rotate_ccw must be 0, 90, 180 or 270')
    numeric(c['confidence'], 'confidence', .000001, 1)
    numeric(c['stop_zone_left'], 'stop_zone_left', 0, 1)
    numeric(c['stop_zone_right'], 'stop_zone_right', 0, 1)
    if c['stop_zone_left'] >= c['stop_zone_right']:
        raise ValueError('stop_zone_left must be less than stop_zone_right')
    numeric(c['max_seconds'], 'max_seconds', 0, 2**31-1, True)
    for key in ('camera', 'serial', 'weights'):
        if not isinstance(c[key], str) or not c[key] or '\x00' in c[key]:
            raise ValueError(f'{key} must be a non-empty path string')
    if not c['serial'].startswith('/dev/') or 'REPLACE' in c['serial']:
        raise ValueError('Set serial to the actual /dev/serial/by-id/... Mega path')
    if not c['camera'].startswith('/dev/'):
        raise ValueError('camera must name the local Jetson camera, such as /dev/video0')
    weights = Path(c['weights']).expanduser()
    c['weights'] = str((root/weights).resolve() if not weights.is_absolute() else weights.resolve())
    c['bind'] = str(address)
    return c

def load_config(path, root=ROOT):
    with Path(path).open(encoding='utf-8-sig') as stream:
        return validate(json.load(stream), root)

def command(config, executable=sys.executable, root=ROOT):
    runtime = root/'runtime'
    result = [str(executable), '-u', str(root/'jetson/vision_stop_server.py'),
              '--bind', config['bind'], '--port', str(config['port']),
              '--camera', config['camera'], '--rotate-ccw', str(config['rotate_ccw']),
              '--serial', config['serial'], '--weights', config['weights'],
              '--confidence', str(config['confidence']),
              '--stop-zone-left', str(config['stop_zone_left']), '--stop-zone-right', str(config['stop_zone_right']),
              '--max-seconds', str(config['max_seconds']),
              '--status-file', str(runtime/'runtime_status.json'), '--log-file', str(runtime/'vision_events.jsonl')]
    if config['allow_tailscale']:
        result.append('--allow-tailscale')
    for address in config['allowed_clients']:
        result.extend(['--allow-client', address])
    return result

def check_model(path):
    if not Path(path).is_file():
        raise ValueError('weights must point to an existing regular model file, not a device or directory')
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    if h.hexdigest() != WEIGHTS_HASH:
        raise ValueError('Model SHA-256 differs from the tested model; do not substitute an unreviewed best.pt')

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/robot.local.json')
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--check', action='store_true', help='Read configuration/model files only; no device access')
    mode.add_argument('--start', action='store_true', help='Start camera/serial control; keep motor power off during setup')
    args = parser.parse_args(argv)
    try:
        c = load_config(args.config)
        check_model(c['weights'])
        cmd = command(c)
        print('Configuration and model SHA-256 verified.')
        print('Chrome URL: http://' + c['bind'] + ':' + str(c['port']) + '/')
        print('Command: ' + shlex.join(cmd))
        if args.check:
            print('CHECK ONLY: no camera, serial port, network listener, or motor operation was started.')
            return 0
        if sys.platform != 'linux':
            raise ValueError('--start must run on the Jetson (Linux), not on a teammate laptop')
        for name in ('camera', 'serial'):
            device = Path(c[name]).resolve(strict=True)
            if not stat.S_ISCHR(device.stat().st_mode):
                raise ValueError(f'{name} is not a character device')
        runtime = ROOT/'runtime'
        runtime.mkdir(exist_ok=True)
        os.chdir(runtime)
        print('Starting robot service. Keep motor power OFF until camera/model/Mega status is confirmed.', flush=True)
        os.execv(sys.executable, cmd)
    except (OSError, ValueError) as exc:
        parser.exit(1, f'Cannot start: {exc}\n')

if __name__ == '__main__':
    raise SystemExit(main())
