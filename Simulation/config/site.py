"""Single source of truth for P4 Transwing simulation site configuration.

Other Python modules should import from here rather than hardcoding coordinates.
Non-Python files (SDF worlds, shell scripts, waypoints) must be updated manually;
see Simulation/SETUP.md section 'Đổi vị trí Home'.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

_SITE_FILE = Path(__file__).resolve().parent / "site.json"


@dataclass(frozen=True)
class HomePosition:
    lat: float
    lon: float
    alt: float


@dataclass(frozen=True)
class MissionPolicy:
    operating_radius_m: float
    max_relative_altitude_m: float
    max_items: int


@dataclass(frozen=True)
class SiteConfig:
    home: HomePosition
    mission_policy: MissionPolicy


def load_site() -> dict:
    with _SITE_FILE.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def load_site_config() -> SiteConfig:
    """Load and validate site.json into typed dataclasses."""
    raw = load_site()
    home_raw = raw["home"]
    home = HomePosition(
        lat=float(home_raw["latitude"]),
        lon=float(home_raw["longitude"]),
        alt=float(home_raw["altitude_m"]),
    )
    policy_raw = raw.get("mission_policy", {})
    policy = MissionPolicy(
        operating_radius_m=float(policy_raw.get("operating_radius_m", 500.0)),
        max_relative_altitude_m=float(policy_raw.get("max_relative_altitude_m", 150.0)),
        max_items=int(policy_raw.get("max_items", 100)),
    )
    return SiteConfig(home=home, mission_policy=policy)


def home_position() -> dict[str, float]:
    home = load_site()["home"]
    return {"lat": home["latitude"], "lon": home["longitude"], "alt": home["altitude_m"]}


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance between two points in metres."""
    R = 6_371_000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
