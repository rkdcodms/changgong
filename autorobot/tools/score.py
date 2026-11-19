"""ROBOCON score per the 2026-11-10 rulebook (pp.28, 33-34). Planning aid only.

Score rules are configurable because the TA said they may change after Eval2.
Expected values need measured success rates (tools/field_log.py); without them
this only computes the score of one assumed outcome.

  python3 tools/score.py --crew 2,0,3 --stops ok,ok,wrong --crew-done 1,1,0 --altar 1,1,0 \
      --odysseus 1 --bow 1 --pin 0 --first 0 --laps 1 --cubes 0 --resets 1
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass


@dataclass(frozen=True)
class Rules:
    lap: int = 5
    stop: int = 5            # x n; a stop at the wrong position earns `stop` once
    crew: int = 2            # x crew count of that trial
    altar: int = 2           # x n
    odysseus: int = 10
    bow: int = 5
    pin: int = 5
    first_multiplier: int = 2
    cube: int = 5
    collision: int = -20
    reset: int = -5


def score(crews, stops, crew_done, altar, odysseus=False, bow=False, pin=False, first=False,
          laps=0, cubes=0, collisions=0, resets=0, rules=Rules()):
    """stops: 'ok' (on the mark, correct order), 'wrong' (stopped elsewhere/wrong order), 'none'."""
    n_trials = len(crews)
    if not (len(stops) == len(crew_done) == len(altar) == n_trials):
        raise ValueError('Per-trial lists must have the same length')
    parts = {'stop': 0, 'crew': 0, 'altar': 0}
    for i, (crew, stop, done, alt) in enumerate(zip(crews, stops, crew_done, altar)):
        n = i + 1
        if stop not in ('ok', 'wrong', 'none'):
            raise ValueError(f'stop {stop!r}')
        parts['stop'] += rules.stop * n if stop == 'ok' else (rules.stop if stop == 'wrong' else 0)
        if done:
            parts['crew'] += rules.crew * crew
        if alt:
            if not done and crew > 0:
                raise ValueError(f'Trial {n}: statue to altar requires the crew first (rulebook p.30)')
            parts['altar'] += rules.altar * n
    parts['laps'] = min(laps * rules.lap, rules.lap * (5 + n_trials))
    ithaca = 0
    if odysseus:
        ithaca = rules.odysseus + (rules.bow if bow else 0) + (rules.pin if pin else 0)
        if first:
            ithaca *= rules.first_multiplier
    elif bow or pin:
        raise ValueError('Bow/pin points count only with Odysseus in Ithaca')
    parts['ithaca'] = ithaca
    parts['hidden'] = rules.cube * cubes
    parts['penalty'] = rules.collision * collisions + rules.reset * resets
    parts['total'] = sum(parts.values())
    return parts


def _ints(text):
    return [int(x) for x in text.split(',')] if text else []


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--crew', required=True)
    ap.add_argument('--stops', required=True)
    ap.add_argument('--crew-done', required=True)
    ap.add_argument('--altar', required=True)
    for name in ('odysseus', 'bow', 'pin', 'first', 'laps', 'cubes', 'collisions', 'resets'):
        ap.add_argument('--' + name, type=int, default=0)
    a = ap.parse_args(argv)
    result = score(_ints(a.crew), a.stops.split(','), [bool(x) for x in _ints(a.crew_done)],
                   [bool(x) for x in _ints(a.altar)], bool(a.odysseus), bool(a.bow), bool(a.pin),
                   bool(a.first), a.laps, a.cubes, a.collisions, a.resets)
    print(' '.join(f'{k}={v}' for k, v in result.items()))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
