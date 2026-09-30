"""
Landing phase monitor.
"""
from typing import Any

from Simulation.experiments.compat import StrEnum

class LandingPhase(StrEnum):
    REQUESTED = "requested"
    TRANSITIONING_TO_VTOL = "transitioning_to_vtol"
    POSITION_CAPTURE = "position_capture"
    DESCENDING = "descending"
    FINAL = "final"
    TOUCHDOWN = "touchdown"
    DISARMED = "disarmed"
    FAILED = "failed"

def touchdown_candidate(state: Any) -> bool:
    return (
        state.relative_alt_m < 0.3
        and abs(state.vertical_speed_mps) < 0.25
        and state.groundspeed_mps < 0.5
    )

class LandingMonitor:
    def __init__(self):
        self.phase = LandingPhase.REQUESTED

    def update(self, state: Any) -> LandingPhase:
        if touchdown_candidate(state):
            self.phase = LandingPhase.TOUCHDOWN
        if getattr(state, "armed", True) is False and self.phase == LandingPhase.TOUCHDOWN:
            self.phase = LandingPhase.DISARMED
        return self.phase
