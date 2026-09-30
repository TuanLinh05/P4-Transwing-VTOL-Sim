"""
Experiment Runner state machine and orchestration.
"""
import time
from typing import Any
from pathlib import Path
from Simulation.experiments.schemas import ExperimentRequest, RunPhase, RunManifest
from Simulation.experiments.run_store import RunStore, RunContext
from Simulation.experiments.control_client import ControlStationClient

def compute_metrics(run: RunContext) -> dict:
    return {"status": "computed"}

def evaluate(metrics: dict, threshold_profile: Any) -> dict:
    return {"status": "evaluated"}

class ExperimentRunner:
    def __init__(self, store: RunStore, client: ControlStationClient, monitor: Any = None):
        self.store = store
        self.client = client
        self.monitor = monitor

    def _phase(self, run: RunContext, phase: RunPhase) -> None:
        run.phase = phase
        print(f"[{run.run_id}] Phase: {phase.value}")

    def wait_until(self, predicate, timeout_s: float, description: str = "") -> None:
        start = time.monotonic()
        while time.monotonic() - start < timeout_s:
            if predicate():
                return
            time.sleep(0.5)
        raise TimeoutError(f"Timeout waiting for {description}")

    def prepare_inputs(self, request: ExperimentRequest, run: RunContext) -> Any:
        class Inputs:
            case_name = request.case
            mission = request.mission
        return Inputs()

    def ready_for_flight(self) -> bool:
        try:
            return self.client.health().healthy
        except Exception:
            return False

    def ensure_landed_and_disarmed(self, request: ExperimentRequest) -> None:
        self.wait_until(
            lambda: not self.client.state().armed,
            timeout_s=request.timeouts.landing_s,
            description="disarm after landing"
        )

    def collect_artifacts(self, run: RunContext) -> None:
        pass

    def emergency_cleanup(self, run: RunContext, error: Exception) -> None:
        print(f"Emergency cleanup due to {error}")
        try:
            self.client.abort()
        except Exception:
            pass

    def run_once(self, request: ExperimentRequest, repetition: int) -> Path:
        run = self.store.begin(request, repetition)
        try:
            self._phase(run, RunPhase.PREPARE)
            inputs = self.prepare_inputs(request, run)

            self._phase(run, RunPhase.STARTING)
            self.client.start_stack(inputs.case_name, run.run_id)
            self.wait_until(self.ready_for_flight, request.timeouts.stack_ready_s, "stack ready")

            self._phase(run, RunPhase.UPLOADING)
            receipt = self.client.upload_mission(inputs.mission)
            if not receipt.confirmed:
                raise RuntimeError("Mission upload was not confirmed")

            self._phase(run, RunPhase.ARMING)
            self.client.arm()
            self.wait_until(lambda: self.client.state().vehicle.armed, request.timeouts.arm_s, "armed")

            self._phase(run, RunPhase.FLYING)
            self.client.start_auto(receipt.mission_id)
            if self.monitor:
                self.monitor.until_mission_terminal(request)

            self._phase(run, RunPhase.LANDING)
            self.ensure_landed_and_disarmed(request)

            self._phase(run, RunPhase.COLLECTING)
            self.collect_artifacts(run)

            self._phase(run, RunPhase.ANALYZING)
            metrics = compute_metrics(run)
            verdict = evaluate(metrics, getattr(request, "threshold_profile", None))
            return self.store.complete(run, metrics, verdict)
        except Exception as error:
            self.emergency_cleanup(run, error)
            return self.store.fail(run, error)
        finally:
            self.client.stop_stack()
