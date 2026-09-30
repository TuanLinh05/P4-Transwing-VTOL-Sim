"""
Wind scenario generation.
"""
from dataclasses import dataclass
from typing import Generator
import json

@dataclass
class WindScenario:
    base_wind_enu_mps: tuple[float, float, float]
    gust_model: dict | None
    turbulence_model: dict | None
    vertical_profile: dict | None

def steady_wind(scenario: WindScenario) -> Generator[tuple[float, float, float], None, None]:
    while True:
        yield scenario.base_wind_enu_mps

def discrete_pulse_gust(scenario: WindScenario, seed: int) -> Generator[tuple[float, float, float], None, None]:
    # Mock implementation
    while True:
        yield scenario.base_wind_enu_mps

def filtered_noise_turbulence(scenario: WindScenario, seed: int) -> Generator[tuple[float, float, float], None, None]:
    # Mock implementation
    while True:
        yield scenario.base_wind_enu_mps

def load_scenario_from_json(path: str) -> WindScenario:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return WindScenario(
        base_wind_enu_mps=tuple(data.get("base_wind_enu_mps", (0.0, 0.0, 0.0))),
        gust_model=data.get("gust"),
        turbulence_model=data.get("turbulence"),
        vertical_profile=data.get("vertical_profile")
    )
