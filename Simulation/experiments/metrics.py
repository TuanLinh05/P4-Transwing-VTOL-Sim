"""
Metrics computation module.
"""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class FlightSample:
    t_s: float
    phase: str
    mode: str
    armed: bool
    north_m: float
    east_m: float
    relative_alt_m: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    airspeed_mps: float | None
    groundspeed_mps: float
    vertical_speed_mps: float
    throttle_pct: float
    servo_outputs: tuple[int, ...]
    mission_current: int
    battery_voltage_v: float | None
    battery_current_a: float | None

def compute_hover_metrics(samples: list[FlightSample], target_alt: float) -> dict:
    # Example computation
    if not samples:
        return {}
    north_error = [s.north_m for s in samples] # simplified
    east_error = [s.east_m for s in samples]
    alt_error = [(s.relative_alt_m - target_alt) for s in samples]
    
    pos_rmse = math.sqrt(sum(n**2 + e**2 for n, e in zip(north_error, east_error)) / len(samples))
    alt_rmse = math.sqrt(sum(a**2 for a in alt_error) / len(samples))
    max_attitude = max(max(abs(s.roll_deg), abs(s.pitch_deg)) for s in samples)
    
    return {
        "position_rmse_m": pos_rmse,
        "altitude_rmse_m": alt_rmse,
        "max_attitude_deg": max_attitude,
        "heading_rmse_deg": 0.0 # simplified
    }

def compute_transition_metrics(samples: list[FlightSample]) -> dict:
    if not samples:
        return {}
    alt_start = samples[0].relative_alt_m
    min_alt = min(s.relative_alt_m for s in samples)
    min_airspeed = min((s.airspeed_mps or 0.0) for s in samples)
    duration = samples[-1].t_s - samples[0].t_s
    
    return {
        "altitude_loss_m": alt_start - min_alt,
        "duration_s": duration,
        "min_airspeed_mps": min_airspeed
    }

def compute_mission_metrics(samples: list[FlightSample]) -> dict:
    if not samples:
        return {}
    duration = samples[-1].t_s - samples[0].t_s
    return {
        "cross_track_rmse_m": 0.0, # simplified
        "completion_ratio": 1.0,
        "mission_duration_s": duration
    }

def compute_landing_metrics(samples: list[FlightSample]) -> dict:
    if not samples:
        return {}
    duration = samples[-1].t_s - samples[0].t_s
    return {
        "touchdown_error_m": 0.0, # simplified
        "touchdown_vertical_speed_mps": abs(samples[-1].vertical_speed_mps),
        "landing_duration_s": duration
    }
