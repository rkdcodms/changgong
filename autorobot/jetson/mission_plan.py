"""Trial order and crew counts announced on the screen before each game.

Pure data validation; no hardware access. The class ids come from the model's
``names`` (vision_control.NAMES). Rulebook display names and common spellings
are accepted as aliases, but the stored plan always uses the model names.
"""
from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path

from vision_control import NAMES

MODEL_NAME_TO_ID = {name: cid for cid, name in NAMES.items()}

# Rulebook p.8 display names and spellings seen in team documents -> model name.
# The model (best.pt, 9f9d66d) spells the Cyclops class "Cyclopes".
ALIASES = {
    'calypso': 'Calypso',
    'cattle': 'Cattle', 'cattleofhelios': 'Cattle', 'helios': 'Cattle',
    'cyclopes': 'Cyclopes', 'cyclops': 'Cyclopes', 'cyclopse': 'Cyclopes',
    'laestrygonians': 'Laestrygonians', 'laestrygonian': 'Laestrygonians',
    'poseidon': 'Poseidon', 'poseidonswrath': 'Poseidon', 'poseidonwrath': 'Poseidon',
    'scylla': 'Scylla',
    'siren': 'Siren', 'sirens': 'Siren',
    'trojanhorse': 'TrojanHorse', 'trojan': 'TrojanHorse', 'horse': 'TrojanHorse',
}
MIN_TRIALS, MAX_TRIALS = 3, 8        # rulebook p.10
MIN_CREW, MAX_CREW = 0, 3            # rulebook p.10


def canonical(name):
    """Return the model class name for a typed statue name, or raise ValueError."""
    if isinstance(name, int) and not isinstance(name, bool):
        if name in NAMES:
            return NAMES[name]
        raise ValueError(f'Unknown class id {name}')
    if not isinstance(name, str):
        raise ValueError(f'Statue name must be text, got {name!r}')
    key = re.sub(r"[^a-z]", '', name.lower())
    if key in ALIASES:
        return ALIASES[key]
    raise ValueError(f'Unknown statue {name!r}. Use one of: ' + ', '.join(NAMES.values()))


@dataclass(frozen=True)
class Trial:
    index: int          # 0-based position; the rulebook's n is index + 1
    class_id: int
    name: str
    crew: int

    @property
    def n(self):
        return self.index + 1


@dataclass(frozen=True)
class MissionPlan:
    trials: tuple

    def to_json(self):
        return {'schema': 'mission-plan-v1',
                'trials': [{'n': t.n, 'statue': t.name, 'class_id': t.class_id, 'crew': t.crew}
                           for t in self.trials]}

    def summary(self):
        return ' -> '.join(f'{t.n}:{t.name}(crew {t.crew})' for t in self.trials)


def build_plan(statues, crews, allow_repeat=False):
    """Validate the announced order. Raises ValueError with a readable message."""
    if not isinstance(statues, (list, tuple)) or not isinstance(crews, (list, tuple)):
        raise ValueError('Order and crew counts must be lists')
    if len(statues) != len(crews):
        raise ValueError(f'{len(statues)} statues but {len(crews)} crew counts')
    if not MIN_TRIALS <= len(statues) <= MAX_TRIALS:
        raise ValueError(f'Trial count must be {MIN_TRIALS}..{MAX_TRIALS}, got {len(statues)}')
    trials, seen = [], set()
    for index, (statue, crew) in enumerate(zip(statues, crews)):
        name = canonical(statue)
        if type(crew) is not int or not MIN_CREW <= crew <= MAX_CREW:
            raise ValueError(f'Trial {index + 1} ({name}): crew must be an integer {MIN_CREW}..{MAX_CREW}, got {crew!r}')
        if name in seen and not allow_repeat:
            # A repeated statue makes "which stop belongs to which trial" ambiguous
            # for a class-only detector. Confirm with the TA before allowing it.
            raise ValueError(f'Statue {name} appears twice; pass allow_repeat only after the TA confirms repeats')
        seen.add(name)
        trials.append(Trial(index, MODEL_NAME_TO_ID[name], name, crew))
    return MissionPlan(tuple(trials))


def parse_cli(order, crews):
    """Parse "Scylla, Cyclops, Siren" and "2,0,3" style strings."""
    statues = [part.strip() for part in re.split(r'[,>\n]+', order) if part.strip()]
    try:
        counts = [int(part) for part in re.split(r'[,\s]+', crews.strip()) if part]
    except ValueError:
        raise ValueError(f'Crew counts must be integers, got {crews!r}') from None
    return build_plan(statues, counts)


def load_plan(path):
    data = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    if not isinstance(data, dict) or data.get('schema') != 'mission-plan-v1' or not isinstance(data.get('trials'), list):
        raise ValueError('Plan file must be a mission-plan-v1 object')
    rows = data['trials']
    for i, row in enumerate(rows):
        if not isinstance(row, dict) or row.get('n') != i + 1:
            raise ValueError(f'Plan row {i + 1} is malformed or out of order')
    plan = build_plan([r.get('statue') for r in rows], [r.get('crew') for r in rows])
    for row, trial in zip(rows, plan.trials):
        if 'class_id' in row and row['class_id'] != trial.class_id:
            raise ValueError(f'Plan row {trial.n}: class_id {row["class_id"]} does not match {trial.name}')
    return plan
