"""
Control station client for experiment runner.
"""
from typing import Protocol, Any
import urllib.request
import json
from dataclasses import dataclass
from pathlib import Path

@dataclass
class Health:
    healthy: bool

@dataclass
class Operation:
    id: str

@dataclass
class VehicleSnapshot:
    armed: bool
    mode: str
    vehicle: Any = None

@dataclass
class MissionReceipt:
    mission_id: str
    confirmed: bool

@dataclass
class CommandReceipt:
    command_id: str
    confirmed: bool

class ControlStationClient(Protocol):
    def health(self) -> Health: ...
    def start_stack(self, case: str, run_id: str) -> Operation: ...
    def state(self) -> VehicleSnapshot: ...
    def upload_mission(self, mission: Any) -> MissionReceipt: ...
    def arm(self) -> CommandReceipt: ...
    def start_auto(self, mission_id: str) -> CommandReceipt: ...
    def abort(self) -> CommandReceipt: ...
    def qland(self) -> CommandReceipt: ...
    def stop_stack(self) -> Operation: ...
    def download_artifacts(self, run_id: str, destination: Path) -> None: ...

class HttpControlClient:
    def __init__(self, base_url: str = "http://localhost:8000"):
        self.base_url = base_url

    def _post(self, endpoint: str, data: dict = None) -> dict:
        req = urllib.request.Request(
            f"{self.base_url}{endpoint}",
            data=json.dumps(data).encode("utf-8") if data else b"{}",
            headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(req, timeout=10.0) as response:
            return json.loads(response.read().decode())

    def _get(self, endpoint: str) -> dict:
        req = urllib.request.Request(f"{self.base_url}{endpoint}")
        with urllib.request.urlopen(req, timeout=10.0) as response:
            return json.loads(response.read().decode())

    def health(self) -> Health:
        res = self._get("/api/health")
        return Health(healthy=res.get("ok", False))

    def start_stack(self, case: str, run_id: str) -> Operation:
        res = self._post("/api/stack/start", {"case": case, "run_id": run_id})
        return Operation(id=res.get("commandId", ""))

    def state(self) -> VehicleSnapshot:
        res = self._get("/api/state")
        
        class VehicleMock:
            def __init__(self, armed: bool):
                self.armed = armed
        
        return VehicleSnapshot(
            armed=res.get("armed", False),
            mode=res.get("mode", ""),
            vehicle=VehicleMock(res.get("armed", False))
        )

    def upload_mission(self, mission: Any) -> MissionReceipt:
        res = self._post("/api/mission/upload", {"mission": mission})
        return MissionReceipt(
            mission_id=res.get("missionId", ""),
            confirmed=res.get("confirmed", False)
        )

    def arm(self) -> CommandReceipt:
        res = self._post("/api/command/arm")
        return CommandReceipt(
            command_id=res.get("commandId", ""),
            confirmed=res.get("confirmed", False)
        )

    def start_auto(self, mission_id: str) -> CommandReceipt:
        res = self._post("/api/command/auto", {"missionId": mission_id})
        return CommandReceipt(
            command_id=res.get("commandId", ""),
            confirmed=res.get("confirmed", False)
        )

    def abort(self) -> CommandReceipt:
        res = self._post("/api/command/abort")
        return CommandReceipt(
            command_id=res.get("commandId", ""),
            confirmed=res.get("confirmed", False)
        )

    def qland(self) -> CommandReceipt:
        res = self._post("/api/command/qland")
        return CommandReceipt(
            command_id=res.get("commandId", ""),
            confirmed=res.get("confirmed", False)
        )

    def stop_stack(self) -> Operation:
        res = self._post("/api/stack/stop")
        return Operation(id=res.get("commandId", ""))

    def download_artifacts(self, run_id: str, destination: Path) -> None:
        pass
