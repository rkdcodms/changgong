"""Write runtime/mission_plan.json from the order shown on the screen.

Example (statue names are case-insensitive; rulebook names are accepted):
  python3 scripts/mission_plan.py --order "Scylla, Cyclops, Sirens" --crew "2,0,3"
Reads and writes one local file only; no device or network access.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'jetson'))
from mission_plan import parse_cli  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--order', required=True, help='Statues in trial order, comma separated')
    ap.add_argument('--crew', required=True, help='Crew (sacrifice) count per trial, 0..3, comma separated')
    ap.add_argument('--out', type=Path, default=ROOT / 'runtime' / 'mission_plan.json')
    args = ap.parse_args(argv)
    try:
        plan = parse_cli(args.order, args.crew)
    except ValueError as exc:
        ap.exit(2, f'PLAN REJECTED: {exc}\n')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    tmp = args.out.with_name(args.out.name + '.tmp')
    tmp.write_text(json.dumps(plan.to_json(), indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    tmp.replace(args.out)
    print('PLAN OK -> ' + str(args.out))
    for t in plan.trials:
        print(f'  n={t.n}  {t.name:<15} class_id={t.class_id}  crew={t.crew}  stop={5 * t.n}pt  crew={2 * t.crew}pt  altar={2 * t.n}pt')
    print('Compare every row with the screen before starting.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
