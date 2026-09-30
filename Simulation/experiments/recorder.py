"""
Telemetry recording and alignment module.
"""
from dataclasses import dataclass, asdict
import queue
import gzip
import json
from pathlib import Path
from typing import Any, Protocol, Iterator

@dataclass(frozen=True)
class TelemetryRecord:
    sequence: int
    received_monotonic_s: float
    received_utc: str
    sim_time_s: float | None
    boot_time_ms: int | None
    message_type: str
    data: dict[str, Any]

class TelemetryRecorder:
    def __init__(self, output: Path, max_queue: int = 20_000):
        self.output = output
        self.queue: queue.Queue[TelemetryRecord] = queue.Queue(maxsize=max_queue)
        self.dropped_samples = 0
        self.stopped = False

    def publish(self, sample: TelemetryRecord) -> None:
        try:
            self.queue.put_nowait(sample)
        except queue.Full:
            self.dropped_samples += 1

    def writer_loop(self) -> None:
        with gzip.open(self.output, "wt", encoding="utf-8") as stream:
            while not self.stopped or not self.queue.empty():
                try:
                    sample = self.queue.get(timeout=0.2)
                    stream.write(json.dumps(asdict(sample), separators=(",", ":")))
                    stream.write("\n")
                except queue.Empty:
                    continue

@dataclass(frozen=True)
class TimeAnchor:
    run_id: str
    wall_time_utc: str
    sim_time_s: float
    autopilot_boot_ms: int
    mavlink_time_usec: int | None

class TimelineBuilder:
    def add_gazebo_sample(self, sim_time_s: float, payload: dict) -> None: pass
    def add_mavlink_sample(self, boot_ms: int, payload: dict) -> None: pass
    def add_dataflash_record(self, time_us: int, payload: dict) -> None: pass
    def align(self, anchors: list[TimeAnchor]) -> Any: pass

class FlightLogReader(Protocol):
    def records(self) -> Iterator[Any]: ...

class DataFlashReader(FlightLogReader):
    def records(self) -> Iterator[Any]: yield from []

class MavlinkTlogReader(FlightLogReader):
    def records(self) -> Iterator[Any]: yield from []

class GazeboTraceReader(FlightLogReader):
    def records(self) -> Iterator[Any]: yield from []
