"""Robocon 순서 주행. autorobot 폴더에서 실행 예시:

  python3 "scripts/robot_run_for robocon.py" --start

--objects를 생략하면 아래 TARGET_OBJECTS(최종 룰북의 고정 순서 8개)를 사용합니다.
다른 순서로 시험할 때만 --objects Calypso Cattle Siren 처럼 넣습니다.
Mega에는 arduino/trace_stop/trace_stop.ino를 업로드해야 합니다.
"""
import sys

from run_robot import ROOT, launch, launch_parser

sys.path.insert(0, str(ROOT / 'jetson'))
from vision_control import NAMES


# 최종 룰북(Final revised) 9쪽: 시련 순서는 항상 고정, 위치만 매 경기 무작위.
# 트로이 목마 → 키클롭스 → 라이스트리고네스 → 칼립소 → 스킬라 → 세이렌 → 헬리오스의 소 → 포세이돈
# 이름은 모델 클래스 이름 기준 (룰북 'Cyclops' = 모델 'Cyclopes').
TARGET_OBJECTS = ['TrojanHorse', 'Cyclopes', 'Laestrygonians', 'Calypso',
                  'Scylla', 'Siren', 'Cattle', 'Poseidon']


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
