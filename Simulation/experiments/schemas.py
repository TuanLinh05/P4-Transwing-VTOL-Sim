"""
Data schemas for experiments framework.
"""
from dataclasses import dataclass

from Simulation.experiments.compat import StrEnum

class RunPhase(StrEnum):
    PREPARE = "prepare"
    STARTING = "starting"
    READY = "ready"
    UPLOADING = "uploading"
    ARMING = "arming"
    FLYING = "flying"
    LANDING = "landing"
    COLLECTING = "collecting"
    ANALYZING = "analyzing"
    COMPLETED = "completed"
    FAILED = "failed"
    ABORTING = "aborting"
    CLEANING = "cleaning"

@dataclass(frozen=True)
class SafetyLimits:
    max_altitude_m: float
    max_distance_home_m: float
    max_roll_deg: float
    max_pitch_deg: float

@dataclass(frozen=True)
class ExperimentTimeouts:
    stack_ready_s: float
    arm_s: float
    takeoff_s: float
    mission_s: float
    landing_s: float

@dataclass(frozen=True)
class ExperimentRequest:
    schema_version: int
    case: str
    mission: str
    profile: str
    repetitions: int
    seed_base: int
    timeouts: ExperimentTimeouts
    safety: SafetyLimits

@dataclass(frozen=True)
class RunManifest:
    schema_version: int
    run_id: str
    case_name: str
    repetition: int
    seed: int
    hashes: dict[str, str]
    versions: dict[str, str]
    started_at_utc: str
