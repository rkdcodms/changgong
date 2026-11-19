"""Check or start the mission program (MS1 firmware required on the Mega).

--check reads configuration, plan and model files only; it opens no device.
Uses the same config/robot.local.json as scripts/run_robot.py.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import shlex
import stat
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'jetson'))
import run_robot  # noqa: E402
from mission_plan import load_plan  # noqa: E402


def command(config, plan, mission_config=None, executable=sys.executable, root=ROOT):
    base = run_robot.command(config, executable, root)
    # Replace the PR #2 5-second server with the mission server and drop its lifetime flag.
    cmd = [str(executable), '-u', str(root / 'jetson/mission_server.py'), '--plan', str(plan)]
    skip = {'--max-seconds', '--status-file', '--log-file'}
    rest, i = base[3:], 0
    while i < len(rest):
        if rest[i] in skip:
            i += 2; continue
        cmd.append(rest[i]); i += 1
    runtime = root / 'runtime'
    cmd += ['--status-file', str(runtime / 'mission_status.json'), '--log-file', str(runtime / 'mission_events.jsonl'),
            '--event-socket', str(runtime / 'mission.sock')]
    if mission_config:
        cmd += ['--mission-config', str(mission_config)]
    return cmd


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config', type=Path, default=ROOT / 'config/robot.local.json')
    ap.add_argument('--plan', type=Path, default=ROOT / 'runtime/mission_plan.json')
    ap.add_argument('--mission-config', type=Path)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument('--check', action='store_true')
    mode.add_argument('--start', action='store_true')
    args = ap.parse_args(argv)
    try:
        c = run_robot.load_config(args.config)
        run_robot.check_model(c['weights'])
        plan = load_plan(args.plan)
        if args.mission_config:
            sys.path.insert(0, str(ROOT / 'jetson'))
            from mission_server import load_config as load_mission
            load_mission(args.mission_config)
        cmd = command(c, args.plan.resolve(), args.mission_config.resolve() if args.mission_config else None)
        print('Configuration, plan and model SHA-256 verified.')
        print('PLAN: ' + plan.summary())
        print('Command: ' + shlex.join(cmd))
        if args.check:
            print('CHECK ONLY: no camera, serial port, network listener, or motor operation was started.')
            return 0
        if sys.platform != 'linux':
            raise ValueError('--start must run on the Jetson (Linux)')
        for name in ('camera', 'serial'):
            if not stat.S_ISCHR(Path(c[name]).resolve(strict=True).stat().st_mode):
                raise ValueError(f'{name} is not a character device')
        (ROOT / 'runtime').mkdir(exist_ok=True)
        os.chdir(ROOT / 'runtime')
        print('Starting mission program. The robot stays braked until a start event.', flush=True)
        os.execv(sys.executable, cmd)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        ap.exit(1, f'Cannot start: {exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
