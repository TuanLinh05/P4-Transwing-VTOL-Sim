"""
RunStore for managing experiment artifacts and lifecycle.
"""
import json
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from datetime import datetime, timezone
from Simulation.experiments.schemas import RunPhase, ExperimentRequest, RunManifest

@dataclass
class RunContext:
    run_id: str
    path: Path
    phase: RunPhase

def make_run_id(case_name: str, repetition: int) -> str:
    now_utc = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"{now_utc}_{case_name}_r{repetition:03d}"

def atomic_write_json(path: Path, data: dict) -> None:
    temp_path = path.with_suffix(".tmp")
    with open(temp_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    temp_path.replace(path)

class RunStore:
    def __init__(self, root_dir: Path):
        self.root = root_dir
        self.root.mkdir(parents=True, exist_ok=True)

    def begin(self, request: ExperimentRequest, repetition: int) -> RunContext:
        run_id = make_run_id(request.case, repetition)
        partial = self.root / f"{run_id}.partial"
        partial.mkdir(parents=True, exist_ok=False)
        return RunContext(run_id=run_id, path=partial, phase=RunPhase.PREPARE)

    def finalize(self, run: RunContext, manifest: RunManifest) -> Path:
        atomic_write_json(run.path / "manifest.json", asdict(manifest))
        final = run.path.with_suffix("")
        run.path.replace(final)
        run.path = final
        return final

    def fail(self, run: RunContext, error: Exception) -> Path:
        failure_info = {
            "error": str(error),
            "phase": run.phase.value,
            "failed_at_utc": datetime.now(timezone.utc).isoformat()
        }
        atomic_write_json(run.path / "failure.json", failure_info)
        return run.path
