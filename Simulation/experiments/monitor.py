"""
Safety monitor and envelope checking.
"""
from dataclasses import dataclass
from typing import Any
from Simulation.experiments.schemas import SafetyLimits

@dataclass
class Sample:
    relative_alt_m: float
    distance_home_m: float
    roll_deg: float
    heartbeat_age_s: float

@dataclass
class Violation:
    reason: str

def check_envelope(sample: Sample, limits: SafetyLimits) -> list[Violation]:
    violations = []
    if sample.relative_alt_m > limits.max_altitude_m:
        violations.append(Violation("MAX_ALTITUDE"))
    if sample.distance_home_m > limits.max_distance_home_m:
        violations.append(Violation("GEOFENCE"))
    if abs(sample.roll_deg) > limits.max_roll_deg:
        violations.append(Violation("ROLL_LIMIT"))
    if sample.heartbeat_age_s > 2.0:
        violations.append(Violation("STALE_HEARTBEAT"))
    return violations
