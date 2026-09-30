from dataclasses import dataclass
from typing import Optional

@dataclass
class EnergyReport:
    status: str
    payload_kg: float
    wind_mps: float
    mission_energy_wh: float
    reserve_energy_wh: float
    total_energy_wh: float
    range_km: float

def estimate_range(battery_capacity_wh: float, power_cruise_w: float, speed_mps: float, reserve_fraction: float = 0.2) -> float:
    usable_energy = battery_capacity_wh * (1.0 - reserve_fraction)
    flight_time_h = usable_energy / power_cruise_w
    range_km = flight_time_h * speed_mps * 3.6
    return range_km
