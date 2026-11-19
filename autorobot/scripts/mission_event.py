"""Send one event to the running mission program over its local socket.

  python3 scripts/mission_event.py start
  python3 scripts/mission_event.py crew_done 1
  python3 scripts/mission_event.py departure_clear 1
  python3 scripts/mission_event.py altar_done 1
  python3 scripts/mission_event.py hold | resume

DEVELOPMENT INPUT. Events are tagged source=operator and the run is reported
as not autonomous. Do not use crew/departure events in a contest run until the
TA confirms the method (rulebook p.7: the AutoRobot must be fully autonomous).
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import socket
import tempfile
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
KINDS = ('start', 'crew_done', 'departure_clear', 'altar_done', 'hold', 'resume')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('kind', choices=KINDS)
    ap.add_argument('trial', nargs='?', type=int, help='Trial number n (1-based) for crew/departure/altar')
    ap.add_argument('--socket', type=Path, default=ROOT / 'runtime' / 'mission.sock')
    ap.add_argument('--source', default='operator')
    args = ap.parse_args(argv)
    if args.kind in ('crew_done', 'departure_clear', 'altar_done') and args.trial is None:
        ap.error(f'{args.kind} needs the trial number')
    event = {'kind': args.kind, 'trial': args.trial, 'id': uuid.uuid4().hex[:16],
             'created_unix': time.time(), 'source': args.source}
    reply_path = Path(tempfile.gettempdir()) / f'mission-reply-{os.getpid()}.sock'
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        sock.bind(str(reply_path))
        sock.settimeout(2.0)
        sock.sendto(json.dumps(event).encode(), str(args.socket))
        reply = json.loads(sock.recv(1024).decode())
    except (OSError, ValueError) as exc:
        ap.exit(1, f'No reply from mission program ({exc}). Nothing was confirmed.\n')
    finally:
        sock.close()
        try:
            reply_path.unlink()
        except OSError:
            pass
    print(json.dumps(reply, ensure_ascii=False))
    return 0 if reply.get('accepted') else 3


if __name__ == '__main__':
    raise SystemExit(main())
