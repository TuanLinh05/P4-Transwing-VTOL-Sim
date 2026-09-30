"""
Fault injection module.
"""
from dataclasses import dataclass
from typing import Protocol

@dataclass
class FaultEvent:
    at_s: float
    type: str
    duration_s: float
    target: str | None = None
    value: float | None = None

@dataclass
class ActiveFault:
    event_id: str
    type: str

class FaultInjector(Protocol):
    def activate(self, event: FaultEvent, sim_time_s: float) -> None: ...
    def update(self, dt_s: float) -> None: ...
    def deactivate(self, event_id: str) -> None: ...
    def snapshot(self) -> list[ActiveFault]: ...

class FaultScheduler:
    def __init__(self, injector: FaultInjector, events: list[FaultEvent]):
        self.injector = injector
        self.events = sorted(events, key=lambda e: e.at_s)
        self.active_events = {}
        self.current_time = 0.0

    def update(self, dt_s: float) -> None:
        self.current_time += dt_s
        self.injector.update(dt_s)
