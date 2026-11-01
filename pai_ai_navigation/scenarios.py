"""Named goal variants; obstacle world and learned weights are shared unchanged."""
from __future__ import annotations
from dataclasses import dataclass


@dataclass(frozen=True)
class Scenario:
    key: str
    label: str
    goal: tuple[float, float]


SCENARIOS = (
    Scenario('standard', '標準ゴール (3.8, -1.0)', (3.8, -1.0)),
    Scenario('behind_obstacle', '障害物の真裏 (3.8, 0.0)', (3.8, 0.0)),
)


def get_scenario(key: str) -> Scenario:
    for scenario in SCENARIOS:
        if scenario.key == key:
            return scenario
    raise ValueError(f'Unknown scenario: {key}')