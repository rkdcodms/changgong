"""Robocon 순서 주행. autorobot 폴더에서 실행 예시:

  python3 "scripts/robot_run_for robocon.py" --start --objects Calypso Cattle Siren

--objects를 생략하면 아래 TARGET_OBJECTS를 사용합니다.
Mega에는 이 버전의 arduino/vision_stop/vision_stop.ino를 업로드해야 합니다.
"""
import sys

from run_robot import ROOT, launch, launch_parser

sys.path.insert(0, str(ROOT / 'jetson'))
from vision_control import NAMES


# 경기 순서대로 동상 이름을 1~8개 입력하세요. 같은 이름을 반복해도 됩니다.
TARGET_OBJECTS = ['Calypso', 'Cattle', 'Cyclopes']


def main(argv=None):
    parser = launch_parser(__doc__)
    parser.add_argument('--objects', nargs='+', metavar='NAME', default=TARGET_OBJECTS,
                        help='주행 순서대로 이름을 입력하세요: ' + ', '.join(NAMES.values()))
    args = parser.parse_args(argv)
    if not isinstance(args.objects, (list, tuple)) or not 1 <= len(args.objects) <= 8:
        parser.error('동상 이름을 1~8개 입력하세요.')
    by_name = {name.casefold(): cid for cid, name in NAMES.items()}
    targets = []
    for name in args.objects:
        cid = by_name.get(name.strip().casefold()) if isinstance(name, str) else None
        if cid is None:
            parser.error(f'알 수 없는 동상 {name!r}. 가능한 이름: {", ".join(NAMES.values())}')
        targets.append(cid)
    print('Robocon 순서: ' + ' → '.join(NAMES[cid] for cid in targets), flush=True)
    return launch(parser, args, ['--target-ids', *map(str, targets)])


if __name__ == '__main__':
    raise SystemExit(main())
