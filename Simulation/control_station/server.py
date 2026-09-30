#!/usr/bin/env python3
"""P4 Transwing local simulation control station.

The server intentionally binds only to localhost. It controls Gazebo / SITL
processes started by this application and communicates with ArduPilot over a
dedicated MAVLink UDP output.
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import mimetypes
import os
import queue
import re
import signal
import subprocess
import threading
import time
import traceback
import urllib.parse
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable, Literal
import secrets
import uuid

from Simulation.config.site import load_site_config, haversine_m

from pymavlink import mavutil


VERSION = "0.2.0"
HOST = "127.0.0.1"
PORT = 8765
MAVLINK_ENDPOINT = "udpin:127.0.0.1:14551"
try:
    _site = load_site_config()
    HOME_POSITION = {
        "lat": _site.home.lat,
        "lon": _site.home.lon,
        "alt": _site.home.altitude_m,
    }
except Exception:
    # Fallback nếu file thiếu
    HOME_POSITION = {"lat": 10.772100, "lon": 106.657900, "alt": 10.0}

SAFE_CASE_NAME = re.compile(r"^[a-z0-9][a-z0-9_-]*$")
FLIGHT_LOG_NAME = re.compile(r"^[0-9]{1,8}\.BIN$", re.IGNORECASE)
TAKEOFF_ALTITUDE_TOLERANCE_M = 0.4
# FBWB maps the throttle stick to requested airspeed when ARSPD_USE=1.  Keep
# the short transition boost separate from the steady fixed-wing cruise trim:
# 1750 requests about 38 m/s with the current 27–42 m/s airspeed envelope.
# FBWB maps 1000..2000 linearly across AIRSPEED_MIN..AIRSPEED_MAX, therefore
# 1470 requests about 34 m/s; 1500 would request the 34.5 m/s midpoint.
TRANSITION_RC3 = 1750
FIXED_WING_CRUISE_RC3 = 1470
# The MKR normally supplies a new frame every 10 ms.  A slightly longer
# timeout absorbs an occasional Windows USB/HTTP scheduling hiccup without
# making a genuine unplugged-radio condition linger for an unsafe duration.
EXTERNAL_RC_TIMEOUT_S = 0.75
EXTERNAL_RC_SWITCH_DEBOUNCE_S = 0.20
EXTERNAL_RC_ARM_DEBOUNCE_S = 0.10
EXTERNAL_RC_RECOVERY_DEBOUNCE_S = 0.12
EXTERNAL_RC_SEND_INTERVAL_S = 0.01
EXTERNAL_RC_LANDED_ALTITUDE_M = 2.0
EXTERNAL_RC_DISARM_RETRY_S = 0.35
EXTERNAL_RC_SWITCH_LOW = 1300
EXTERNAL_RC_SWITCH_HIGH = 1700
EXTERNAL_RC_CHANNEL_MIN = 900
EXTERNAL_RC_CHANNEL_MAX = 2100
# The folded-wing surrogate is intentionally conservative in QLOITER.  Full
# FlySky stick travel can inject more translational energy than its estimated
# inertia/aero model can dissipate cleanly, then the position loop oscillates.
# Limit and slew only roll/pitch while hovering; FBWB keeps the full stick.
EXTERNAL_HOVER_AXIS_LIMIT_PWM = 110
EXTERNAL_HOVER_AXIS_SLEW_PWM_PER_S = 260.0
EXTERNAL_HOVER_RECOVERY_ENTER_TILT_DEG = 11.0
EXTERNAL_HOVER_RECOVERY_ENTER_RATE_DPS = 32.0
EXTERNAL_HOVER_RECOVERY_EXIT_TILT_DEG = 5.0
EXTERNAL_HOVER_RECOVERY_EXIT_RATE_DPS = 12.0
EXTERNAL_HOVER_RECOVERY_HOLD_S = 1.5
# P4RC v1 carries all ten FlySky i6 iBUS channels. Only CH1..CH4 are
# forwarded as MAVLink flight axes. CH7..CH9 are handled here as guarded
# command switches; CH5, CH6 and CH10 remain telemetry/spare inputs.
EXTERNAL_RC_CHANNEL_COUNT = 10
EXTERNAL_RC_ARM_CHANNEL = 6       # CH7 / SWA
EXTERNAL_RC_QRTL_CHANNEL = 7      # CH8 / SWB
EXTERNAL_RC_MODE_CHANNEL = 8      # CH9 / SWC (3-position)
ALLOWED_MODES = {
    "MANUAL",
    "FBWA",
    "FBWB",
    "AUTO",
    "RTL",
    "LOITER",
    "QSTABILIZE",
    "QHOVER",
    "QLOITER",
    "QLAND",
    "QRTL",
}
MISSION_COMMANDS = {
    "WAYPOINT": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    "VTOL_TAKEOFF": mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF,
    "VTOL_LAND": mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND,
    "LOITER_TIME": mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
    "TRANSITION_FW": mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
    "TRANSITION_VTOL": mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
}


def now_ms() -> int:
    return int(time.time() * 1000)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def fixed_wing_transition_precondition(
    state: dict[str, Any],
    minimum_altitude_m: float = 25.0,
    stable_duration_s: float | None = None,
) -> str | None:
    """Return a user-facing reason when a bounded FW transition is unsafe."""
    if not state.get("connected", False):
        return "MAVLink is not connected"
    if not state.get("armed", False):
        return "Vehicle must be armed"
    if str(state.get("mode", "")).upper() != "QLOITER":
        return "Start the fixed-wing transition from QLOITER"
    altitude = float(state.get("position", {}).get("relativeAlt", 0.0))
    if altitude < minimum_altitude_m:
        return (
            f"Altitude must be at least {minimum_altitude_m:.1f} m "
            f"(current {altitude:.1f} m)"
        )
    attitude = state.get("attitude", {})
    roll = abs(float(attitude.get("roll", 0.0)))
    pitch = abs(float(attitude.get("pitch", 0.0)))
    if roll > 10.0 or pitch > 10.0:
        return (
            "Vehicle must be level within 10 degrees "
            f"(roll {roll:.1f}, pitch {pitch:.1f})"
        )
    if stable_duration_s is not None and stable_duration_s < EXTERNAL_HOVER_RECOVERY_HOLD_S:
        return (
            "Vehicle must remain level and low-rate for "
            f"{EXTERNAL_HOVER_RECOVERY_HOLD_S:.1f} seconds "
            f"(current {stable_duration_s:.1f} seconds)"
        )
    return None


def hover_attitude_is_stable(
    state: dict[str, Any],
    *,
    max_tilt_deg: float = EXTERNAL_HOVER_RECOVERY_EXIT_TILT_DEG,
    max_rate_dps: float = EXTERNAL_HOVER_RECOVERY_EXIT_RATE_DPS,
) -> bool:
    """Whether QLOITER is calm enough to hand back roll/pitch or transition."""
    attitude = state.get("attitude") or {}
    return (
        abs(float(attitude.get("roll", 0.0))) <= max_tilt_deg
        and abs(float(attitude.get("pitch", 0.0))) <= max_tilt_deg
        and abs(float(attitude.get("rollSpeed", 0.0))) <= max_rate_dps
        and abs(float(attitude.get("pitchSpeed", 0.0))) <= max_rate_dps
    )


def takeoff_throttle_for_remaining(remaining_m: float) -> int:
    """Return a tapered QLOITER RC3 command for the remaining climb."""
    if remaining_m <= 0.8:
        return 1600
    if remaining_m <= 2.0:
        return 1700
    return 1900


def takeoff_altitude_tolerance_m(target_altitude_m: float) -> float:
    """Return a practical SITL completion band for a relative takeoff target."""
    return float(clamp(target_altitude_m * 0.04, TAKEOFF_ALTITUDE_TOLERANCE_M, 0.8))


def json_copy(value: Any) -> Any:
    return json.loads(json.dumps(value))



class OperationCancelled(Exception):
    pass

class CommandContext:
    def __init__(self, cancelled: threading.Event):
        self._cancelled = cancelled
        
    def checkpoint(self) -> None:
        if self._cancelled.is_set():
            raise OperationCancelled("Command aborted")

@dataclass
class OperationContext:
    operation_id: int
    name: str
    cancel_event: threading.Event

    def raise_if_cancelled(self) -> None:
        if self.cancel_event.is_set():
            raise OperationCancelled(self.name)

class OperationController:
    def __init__(self, state_dict: dict[str, Any], publish_cb: Callable):
        self._lock = threading.RLock()
        self._active: OperationContext | None = None
        self._sequence = 0
        self._state = state_dict
        self._publish_cb = publish_cb

    def start(self, name: str, target_altitude: float | None) -> OperationContext:
        with self._lock:
            if self._active and not self._active.cancel_event.is_set():
                raise RuntimeError(f"Flight operation is already running: {self._active.name}")
            self._sequence += 1
            context = OperationContext(self._sequence, name, threading.Event())
            self._active = context
            self._state.update(
                {
                    "name": name,
                    "status": "running",
                    "message": f"Bắt đầu {name}",
                    "targetAltitude": target_altitude,
                    "startedAt": now_ms(),
                }
            )
            self._publish_cb("running", f"Bắt đầu {name}", target_altitude)
            return context

    def cancel_active(self, reason: str) -> OperationContext | None:
        with self._lock:
            ctx = self._active
            if ctx:
                ctx.cancel_event.set()
                self._publish_cb("cancelling", reason, None)
            return ctx

    def finish_if_current(self, context: OperationContext, status: str, message: str = "") -> None:
        with self._lock:
            if self._active is context:
                self._publish_cb(status, message, None)
                if status in ("completed", "failed", "cancelled"):
                    self._active = None

    def is_current(self, context: OperationContext) -> bool:
        with self._lock:
            return self._active is context

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return self._state.copy()

@dataclass(frozen=True)
class RcLease:
    owner: str
    generation: int


@dataclass(frozen=True)
class ExternalRcFrame:
    """A validated P4RC v1 frame forwarded by the Windows USB bridge."""

    sequence: int
    mcu_millis: int
    channels: tuple[int, ...]
    flags: int

    @property
    def input_valid(self) -> bool:
        return bool(self.flags & 0x01) and not bool(self.flags & 0x02)


def crc16_ccitt_false(data: bytes) -> int:
    """Return CRC-16/CCITT-FALSE used by the MKR P4RC v1 protocol."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def parse_external_rc_line(line: str) -> ExternalRcFrame:
    """Validate one USB line from the MKR before it can affect RC outputs."""
    if not isinstance(line, str):
        raise ValueError("External RC line must be text")
    text = line.strip()
    if not text or len(text) > 256:
        raise ValueError("External RC line has an invalid length")
    try:
        payload, crc_text = text.rsplit(",", 1)
        supplied_crc = int(crc_text, 16)
    except (TypeError, ValueError) as error:
        raise ValueError("External RC frame has no valid CRC field") from error
    if not 0 <= supplied_crc <= 0xFFFF:
        raise ValueError("External RC CRC is outside 16-bit range")
    try:
        payload_bytes = payload.encode("ascii")
    except UnicodeEncodeError as error:
        raise ValueError("External RC frame must be ASCII") from error
    expected_crc = crc16_ccitt_false(payload_bytes)
    if supplied_crc != expected_crc:
        raise ValueError("External RC CRC mismatch")

    fields = payload.split(",")
    # P4RC,1,sequence,millis,ch1..ch10,flags
    if len(fields) != 15 or fields[:2] != ["P4RC", "1"]:
        raise ValueError("Unsupported external RC protocol")
    try:
        sequence = int(fields[2])
        mcu_millis = int(fields[3])
        channels = tuple(int(value) for value in fields[4:14])
        flags = int(fields[14])
    except ValueError as error:
        raise ValueError("External RC frame contains a non-integer field") from error
    if not 0 <= sequence <= 0xFFFFFFFF or not 0 <= mcu_millis <= 0xFFFFFFFF:
        raise ValueError("External RC sequence or timestamp is outside uint32 range")
    if not 0 <= flags <= 0xFF:
        raise ValueError("External RC flags are outside uint8 range")
    if len(channels) != EXTERNAL_RC_CHANNEL_COUNT or any(
        value < EXTERNAL_RC_CHANNEL_MIN or value > EXTERNAL_RC_CHANNEL_MAX
        for value in channels
    ):
        raise ValueError("External RC channel is outside the permitted range")
    return ExternalRcFrame(sequence, mcu_millis, channels, flags)


class ExternalRcInput:
    """Thread-safe state for radio frames; only the MAVLink loop emits overrides."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._frame: ExternalRcFrame | None = None
        self._received_at = 0.0
        self._enabled = False
        self._failsafe_latched = False
        self._last_error = "Waiting for P4RC v1 frame"
        self._candidate_switch5 = "MID"  # legacy telemetry only
        self._candidate_arm = "LOW"
        self._candidate_qrtl = "LOW"
        self._candidate_mode = "LOW"
        self._candidate_since = {
            "arm": 0.0,
            "qrtl": 0.0,
            "mode": 0.0,
        }
        self._stable_switch5 = "MID"  # legacy telemetry only
        self._stable_arm = "LOW"
        self._stable_qrtl = "LOW"
        self._stable_mode = "LOW"
        self._events: collections.deque[str] = collections.deque(maxlen=4)
        self._last_action = ""

    @staticmethod
    def _switch5_level(value: int) -> str:
        if value <= EXTERNAL_RC_SWITCH_LOW:
            return "LOW"
        if value >= EXTERNAL_RC_SWITCH_HIGH:
            return "HIGH"
        return "MID"

    @staticmethod
    def _binary_level(value: int) -> str:
        return "HIGH" if value >= EXTERNAL_RC_SWITCH_HIGH else "LOW"

    @staticmethod
    def _mode_level(value: int) -> str:
        if value <= EXTERNAL_RC_SWITCH_LOW:
            return "LOW"
        if value >= EXTERNAL_RC_SWITCH_HIGH:
            return "HIGH"
        return "MID"

    def ingest(self, frame: ExternalRcFrame) -> None:
        now = time.monotonic()
        with self._lock:
            self._frame = frame
            self._received_at = now
            if not frame.input_valid:
                self._last_error = "MKR reports iBUS timeout"
                return
            # A failsafe must release the aircraft from external RC ownership,
            # but it must not leave the UI permanently showing FAILSAFE once
            # valid radio frames have returned.  _enabled remains False after
            # trip_failsafe(), so recovery still requires an explicit operator
            # enable action before any RC override can resume.
            recovered_from_failsafe = self._failsafe_latched
            self._failsafe_latched = False
            self._last_error = (
                "MKR/iBUS stream recovered; press TRAO QUYỀN RADIO to resume control"
                if recovered_from_failsafe
                else ""
            )
            # Keep CH5 visible for backwards-compatible telemetry, but do not
            # let it command flight modes anymore. The physical controls are
            # CH7=arm, CH8=QRTL, CH9=three-position mode selection.
            self._stable_switch5 = self._switch5_level(frame.channels[4])

            controls = (
                ("arm", self._binary_level(frame.channels[EXTERNAL_RC_ARM_CHANNEL]), "_candidate_arm", "_stable_arm", EXTERNAL_RC_ARM_DEBOUNCE_S),
                ("qrtl", self._binary_level(frame.channels[EXTERNAL_RC_QRTL_CHANNEL]), "_candidate_qrtl", "_stable_qrtl", EXTERNAL_RC_RECOVERY_DEBOUNCE_S),
                ("mode", self._mode_level(frame.channels[EXTERNAL_RC_MODE_CHANNEL]), "_candidate_mode", "_stable_mode", EXTERNAL_RC_SWITCH_DEBOUNCE_S),
            )
            for name, level, candidate_attr, stable_attr, debounce_s in controls:
                candidate = getattr(self, candidate_attr)
                if level != candidate:
                    setattr(self, candidate_attr, level)
                    self._candidate_since[name] = now
                    continue
                stable = getattr(self, stable_attr)
                if (
                    level == stable
                    or now - self._candidate_since[name] < debounce_s
                ):
                    continue
                setattr(self, stable_attr, level)
                if not self._enabled:
                    continue
                if name == "arm":
                    self._events.append("arm_high" if level == "HIGH" else "arm_low")
                elif name == "qrtl" and level == "HIGH":
                    self._events.append("qrtl")
                elif name == "mode":
                    self._events.append({
                        "LOW": "qloiter",
                        "MID": "transition_fw",
                        "HIGH": "fixed_wing",
                    }[level])

    def set_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._enabled = bool(enabled)
            self._failsafe_latched = False
            self._events.clear()
            if self._frame is not None:
                now = time.monotonic()
                self._stable_switch5 = self._switch5_level(self._frame.channels[4])
                self._candidate_switch5 = self._stable_switch5
                self._stable_arm = self._binary_level(self._frame.channels[EXTERNAL_RC_ARM_CHANNEL])
                self._candidate_arm = self._stable_arm
                self._stable_qrtl = self._binary_level(self._frame.channels[EXTERNAL_RC_QRTL_CHANNEL])
                self._candidate_qrtl = self._stable_qrtl
                self._stable_mode = self._mode_level(self._frame.channels[EXTERNAL_RC_MODE_CHANNEL])
                self._candidate_mode = self._stable_mode
                for key in self._candidate_since:
                    self._candidate_since[key] = now

    def pause_for_operation(self, reason: str) -> None:
        with self._lock:
            self._enabled = False
            self._events.clear()
            self._last_error = reason

    def set_action(self, message: str) -> None:
        with self._lock:
            self._last_action = str(message)

    def clear_action(self) -> None:
        with self._lock:
            self._last_action = ""

    def trip_failsafe(self, reason: str) -> None:
        with self._lock:
            self._enabled = False
            self._failsafe_latched = True
            self._events.clear()
            self._last_error = reason

    def pop_event(self) -> str | None:
        with self._lock:
            if not self._events:
                return None
            # Recovery switches must win over arm/mode events if the pilot
            # changes more than one switch during the same debounce window.
            for priority in (
                "arm_low",
                "qrtl",
                "qloiter",
                "transition_fw",
                "fixed_wing",
                "arm_high",
            ):
                try:
                    self._events.remove(priority)
                    return priority
                except ValueError:
                    continue
            return self._events.popleft()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            age = time.monotonic() - self._received_at if self._received_at else None
            frame = self._frame
            fresh = bool(
                frame is not None
                and frame.input_valid
                and age is not None
                and age <= EXTERNAL_RC_TIMEOUT_S
            )
            return {
                "protocol": "P4RC/1",
                "received": frame is not None,
                "fresh": fresh,
                "enabled": self._enabled,
                "failsafeLatched": self._failsafe_latched,
                "ageSeconds": age,
                "sequence": frame.sequence if frame else None,
                "mcuMillis": frame.mcu_millis if frame else None,
                "channels": list(frame.channels) if frame else [
                    1500, 1500, 1000, 1500, 1500,
                    1500, 1500, 1500, 1500, 1500,
                ],
                "flags": frame.flags if frame else 0,
                "switch5": self._stable_switch5,
                "armSwitch": self._stable_arm,
                "qrtlSwitch": self._stable_qrtl,
                "modeSwitch": self._stable_mode,
                "modeRequest": {
                    "LOW": "QLOITER",
                    "MID": "TRANSITION",
                    "HIGH": "FBWB",
                }[self._stable_mode],
                "lastError": self._last_error,
                "lastAction": self._last_action,
            }

class RequestSecurity:
    def __init__(self, bind_host: str, lan_secret: str | None) -> None:
        self.bind_host = bind_host
        self.csrf_token = secrets.token_urlsafe(32)
        self.lan_secret = lan_secret

    def validate_mutation(self, handler: 'BaseHTTPRequestHandler') -> None:
        content_type = (handler.headers.get("Content-Type") or "").split(";")[0].strip().lower()
        if content_type != "application/json":
            raise PermissionError("application/json is required")

        origin = handler.headers.get("Origin")
        if not origin:
            raise PermissionError("Origin is required for browser mutations")
        request_host = (handler.headers.get("Host") or "").lower()
        parsed_origin = urllib.parse.urlparse(origin)
        if (
            parsed_origin.scheme not in {"http", "https"}
            or not request_host
            or parsed_origin.netloc.lower() != request_host
        ):
            raise PermissionError("Origin does not match request host")

        supplied_csrf = handler.headers.get("X-P4-Transwing-CSRF")
        if not secrets.compare_digest(supplied_csrf or "", self.csrf_token):
            raise PermissionError("Invalid CSRF token")

        if self.bind_host != "127.0.0.1":
            supplied_secret = handler.headers.get("X-P4-Transwing-LAN-Secret")
            if not self.lan_secret or not secrets.compare_digest(
                supplied_secret or "", self.lan_secret
            ):
                raise PermissionError("LAN authentication failed")

class CommandQueueFull(RuntimeError): pass
class CommandTimeout(RuntimeError): pass
class CommandExpired(RuntimeError): pass

@dataclass
class StackRun:
    generation: int
    cancel_event: threading.Event
    processes: dict[str, Any] = field(default_factory=dict)

class LogBroker:
    def __init__(self, limit: int = 1600) -> None:
        self._entries: collections.deque[dict[str, Any]] = collections.deque(
            maxlen=limit
        )
        self._lock = threading.Lock()
        self._sequence = 0

    def add(self, source: str, level: str, message: str) -> None:
        clean = message.rstrip()
        if not clean:
            return
        with self._lock:
            self._sequence += 1
            self._entries.append(
                {
                    "id": self._sequence,
                    "timestamp": now_ms(),
                    "source": source,
                    "level": level,
                    "message": clean[-3000:],
                }
            )

    def read(self, after: int = 0, limit: int = 300) -> list[dict[str, Any]]:
        with self._lock:
            entries = [entry for entry in self._entries if entry["id"] > after]
            return entries[-max(1, min(limit, 500)) :]


@dataclass
class ManagedProcess:
    name: str
    command: list[str]
    process: subprocess.Popen[str]
    started_at: int
    lines: collections.deque[str] = field(
        default_factory=lambda: collections.deque(maxlen=400)
    )

    def status(self) -> dict[str, Any]:
        code = self.process.poll()
        return {
            "name": self.name,
            "pid": self.process.pid,
            "running": code is None,
            "exitCode": code,
            "startedAt": self.started_at,
        }


class ProcessManager:
    def __init__(self, repo: Path, logs: LogBroker) -> None:
        self.repo = repo
        self.logs = logs
        self._lock = threading.RLock()
        self._processes: dict[str, ManagedProcess] = {}
        self._status = "stopped"
        self._error = ""
        self._active_config: dict[str, Any] = {}
        self._mission_planner_pid: int | None = None
        self._worker: threading.Thread | None = None
        self._run_generation = 0
        self._active_run: 'StackRun' | None = None

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            process_state = {
                name: managed.status() for name, managed in self._processes.items()
            }
            if (
                self._status == "running"
                and any(not item["running"] for item in process_state.values())
            ):
                self._status = "degraded"
            return {
                "status": self._status,
                "error": self._error,
                "activeConfig": json_copy(self._active_config),
                "processes": process_state,
                "missionPlannerPid": self._mission_planner_pid,
            }

    def _reader(self, managed: ManagedProcess) -> None:
        stream = managed.process.stdout
        if stream is None:
            return
        try:
            for line in stream:
                clean = line.rstrip()
                managed.lines.append(clean)
                level = (
                    "error"
                    if re.search(
                        r"\b(error|failed|fatal|segmentation fault)\b",
                        clean,
                        re.IGNORECASE,
                    )
                    else "info"
                )
                self.logs.add(managed.name, level, clean)
        except Exception as error:  # pragma: no cover - defensive stream handling
            self.logs.add(managed.name, "error", f"Log reader stopped: {error}")

    def _spawn(
        self,
        name: str,
        command: list[str],
        extra_env: dict[str, str] | None = None,
        keep_stdin_open: bool = False,
    ) -> ManagedProcess:
        environment = os.environ.copy()
        environment["P4_TRANSWING_REPO"] = str(self.repo)
        if extra_env:
            environment.update(extra_env)
        self.logs.add(name, "info", "$ " + " ".join(command))
        process = subprocess.Popen(
            command,
            cwd=self.repo,
            env=environment,
            # MAVProxy exits cleanly as soon as it receives EOF on stdin. The
            # dashboard has no terminal, so keep an unused pipe open for the
            # SITL process instead of attaching /dev/null.
            stdin=subprocess.PIPE if keep_stdin_open else subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            errors="replace",
            bufsize=1,
            start_new_session=True,
        )
        managed = ManagedProcess(name, command, process, now_ms())
        with self._lock:
            self._processes[name] = managed
        threading.Thread(
            target=self._reader,
            args=(managed,),
            daemon=True,
            name=f"log-{name}",
        ).start()
        return managed

    def _wait_for_log(
        self, managed: ManagedProcess, token: str, timeout: float
    ) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if managed.process.poll() is not None:
                raise RuntimeError(
                    f"{managed.name} stopped with code {managed.process.returncode}"
                )
            if any(token in line for line in managed.lines):
                return True
            time.sleep(0.15)
        return False

    @staticmethod
    def _windows_path_from_wsl(path: str) -> str:
        match = re.fullmatch(r"/mnt/([a-zA-Z])/(.*)", path)
        if not match:
            return path
        drive, remainder = match.groups()
        return f"{drive.upper()}:\\{remainder.replace('/', chr(92))}"

    @staticmethod
    def _mission_planner_path() -> Path | None:
        configured = os.environ.get("P4_TRANSWING_MISSION_PLANNER", "").strip()
        return Path(configured) if configured else None

    def _launch_mission_planner(self) -> bool:
        mission_planner = self._mission_planner_path()
        if mission_planner is None:
            self.logs.add(
                "mission-planner",
                "warning",
                "Mission Planner is optional and was not configured; set "
                "P4_TRANSWING_MISSION_PLANNER to its WSL path to auto-open it",
            )
            return False
        wsl_path = str(mission_planner)
        if not mission_planner.is_file():
            self.logs.add(
                "mission-planner",
                "error",
                f"Mission Planner was not found at {wsl_path}",
            )
            return False
        windows_path = self._windows_path_from_wsl(wsl_path)
        quoted = windows_path.replace("'", "''")
        script = (
            f"$process = Start-Process -FilePath '{quoted}' -PassThru; "
            "$process.Id"
        )
        try:
            result = subprocess.run(
                ["powershell.exe", "-NoProfile", "-Command", script],
                capture_output=True,
                text=True,
                timeout=15,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            self.logs.add(
                "mission-planner",
                "error",
                f"Unable to open Mission Planner: {error}",
            )
            return False
        if result.returncode != 0:
            self.logs.add(
                "mission-planner",
                "error",
                result.stderr.strip() or "Unable to open Mission Planner",
            )
            return False
        try:
            self._mission_planner_pid = int(result.stdout.strip().splitlines()[-1])
            self.logs.add(
                "mission-planner",
                "info",
                f"Mission Planner started (Windows PID {self._mission_planner_pid})",
            )
            return True
        except (ValueError, IndexError):
            self.logs.add(
                "mission-planner",
                "warning",
                "Mission Planner opened but its PID could not be read",
            )
            return True

    def start(self, config: dict[str, Any]) -> None:
        with self._lock:
            if self._status in {"starting", "running", "degraded"}:
                raise RuntimeError("Simulation is already active")
            self._status = "starting"
            self._error = ""
            self._active_config = json_copy(config)
            self._run_generation += 1
            import threading
            self._active_run = StackRun(self._run_generation, threading.Event())
        self._worker = threading.Thread(
            target=self._start_worker,
            args=(json_copy(config), self._active_run),
            daemon=True,
            name="simulation-start",
        )
        self._worker.start()

    def _assert_current_run(self, run: 'StackRun') -> None:
        if run.cancel_event.is_set():
            raise RuntimeError("Stack run cancelled")
        with self._lock:
            if self._active_run is not run:
                raise RuntimeError("Stale stack run")

    def _start_worker(self, config: dict[str, Any], run: 'StackRun') -> None:
        try:
            self._assert_current_run(run)
            mode = str(config.get("environmentMode", "fixed"))
            if mode == "live":
                raise ValueError(
                    "Live weather is locked until P4 aerodynamic and wind "
                    "coefficients have been identified"
                )

            case_name = str(config.get("case", "baseline"))
            if case_name != "baseline":
                raise ValueError(
                    "Only the baseline P4 case is enabled during controller integration"
                )

            preflight_command = [
                "bash",
                str(self.repo / "Simulation/tools/preflight_wsl.sh"),
                "--quiet",
            ]
            preflight_environment = os.environ.copy()
            preflight_environment["P4_TRANSWING_REPO"] = str(self.repo)
            self.logs.add(
                "preflight", "info", "$ " + " ".join(preflight_command)
            )
            try:
                preflight = subprocess.run(
                    preflight_command,
                    cwd=self.repo,
                    env=preflight_environment,
                    capture_output=True,
                    text=True,
                    errors="replace",
                    timeout=45,
                    check=False,
                )
            except subprocess.TimeoutExpired as error:
                raise RuntimeError("P4 preflight timed out after 45 seconds") from error
            preflight_output = "\n".join(
                part.strip()
                for part in (preflight.stdout, preflight.stderr)
                if part.strip()
            )
            for line in preflight_output.splitlines():
                self.logs.add(
                    "preflight",
                    "warning" if line.startswith("[WARN]") else "info",
                    line,
                )
            if preflight.returncode != 0:
                detail = preflight_output.splitlines()[-1] if preflight_output else ""
                raise RuntimeError(
                    "P4 preflight failed; fix the reported dependency/port issue"
                    + (f": {detail}" if detail else "")
                )
            self._assert_current_run(run)

            scenery_mode = os.environ.get(
                "P4_TRANSWING_GAZEBO_WORLD", ""
            ).strip().lower()
            if scenery_mode == "scenery_xl":
                gazebo_runner = "run_p4_ardupilot_scenery_xl_server_wsl.sh"
                scenery_label = "scenery-xl"
            elif scenery_mode == "scenery":
                gazebo_runner = "run_p4_ardupilot_scenery_server_wsl.sh"
                scenery_label = "scenery"
            else:
                gazebo_runner = "run_p4_ardupilot_server_wsl.sh"
                scenery_label = "light"
            self.logs.add(
                "control-station",
                "info",
                "Gazebo world: " + scenery_label,
            )
            server_command = [
                "bash",
                str(self.repo / "Simulation/gazebo" / gazebo_runner),
            ]

            gazebo = self._spawn("gazebo-server", server_command)
            self._assert_current_run(run)
            if not self._wait_for_log(gazebo, "World [", 90):
                raise RuntimeError("Gazebo server did not become ready in 90 seconds")

            self._assert_current_run(run)
            gazebo_gui = self._spawn(
                "gazebo-gui",
                [
                    "bash",
                    str(self.repo / "Simulation/gazebo/run_p4_gui_wsl.sh"),
                ],
            )
            # A common remote-WSL failure leaves only a taskbar icon because
            # `gz sim -g` has already exited after a WSLg/Qt error. Surface
            # that deterministically instead of claiming a fully running
            # stack. The Gazebo server and SITL may still be useful headless.
            time.sleep(1.0)
            gazebo_gui_started = gazebo_gui.process.poll() is None
            if not gazebo_gui_started:
                self.logs.add(
                    "gazebo-gui",
                    "warning",
                    "Gazebo GUI stopped during startup; inspect its log for WSLg/Qt/OpenGL details. Continuing headless.",
                )
            self._assert_current_run(run)
            sitl = self._spawn(
                "ardupilot-sitl",
                [
                    "bash",
                    str(self.repo / "Simulation/ardupilot/run_p4_sitl_wsl.sh"),
                ],
                {
                    "P4_TRANSWING_EXTRA_MAVLINK_OUT": "127.0.0.1:14551",
                    "P4_TRANSWING_MAVPROXY_CONSOLE": "0",
                    "P4_TRANSWING_HEADLESS": "1",
                    "SITL_RITW_MINIMIZE": "1",
                },
                keep_stdin_open=True,
            )
            self._assert_current_run(run)
            if not self._wait_for_log(sitl, "Detected vehicle", 25):
                self.logs.add(
                    "control-station",
                    "warning",
                    "SITL started, but heartbeat readiness was not seen in its log",
                )

            mission_planner_requested = bool(
                config.get("openMissionPlanner", True)
            )
            mission_planner_started = (
                self._launch_mission_planner()
                if mission_planner_requested
                else True
            )
            
            self._assert_current_run(run)

            with self._lock:
                self._status = (
                    "running"
                    if mission_planner_started and gazebo_gui_started
                    else "degraded"
                )
            degradation = []
            if not gazebo_gui_started:
                degradation.append("Gazebo GUI")
            if not mission_planner_started:
                degradation.append("Mission Planner")
            self.logs.add(
                "control-station",
                "warning" if degradation else "info",
                (
                    "Simulation stack is running without "
                    + " and ".join(degradation)
                    + "; vehicle remains DISARMED"
                    if degradation
                    else "Simulation stack is running; vehicle remains DISARMED"
                ),
            )
        except Exception as error:
            with self._lock:
                self._status = "error"
                self._error = str(error)
            self.logs.add("control-station", "error", str(error))
            self.logs.add("control-station", "error", traceback.format_exc())
            self.stop(close_mission_planner=False, preserve_error=True)

    def _terminate_managed(self, managed: ManagedProcess) -> None:
        process_group = managed.process.pid

        def group_exists() -> bool:
            try:
                os.killpg(process_group, 0)
                return True
            except ProcessLookupError:
                return False
            except PermissionError:
                return True

        try:
            os.killpg(process_group, signal.SIGTERM)
        except ProcessLookupError:
            return

        # sim_vehicle.py can exit before MAVProxy. MAVProxy intentionally keeps
        # stdin open and may ignore TERM, so waiting only for the group leader
        # leaves an orphan that corrupts the next start. Wait for the complete
        # process group, then use KILL as a bounded fallback.
        deadline = time.monotonic() + 4
        while group_exists() and time.monotonic() < deadline:
            time.sleep(0.1)
        if group_exists():
            try:
                os.killpg(process_group, signal.SIGKILL)
            except ProcessLookupError:
                pass
        try:
            managed.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            pass

    def stop(
        self, close_mission_planner: bool = True, preserve_error: bool = False
    ) -> None:
        with self._lock:
            if self._active_run:
                self._active_run.cancel_event.set()
            if not preserve_error:
                self._status = "stopping"
                self._error = ""
            processes = list(self._processes.values())
        for name in ("ardupilot-sitl", "gazebo-gui", "gazebo-server"):
            managed = next((item for item in processes if item.name == name), None)
            if managed:
                self._terminate_managed(managed)
        if close_mission_planner and self._mission_planner_pid:
            mission_planner_pid = self._mission_planner_pid
            # FIX-15: Xác minh PID còn thuộc Mission Planner trước khi force-kill
            # để tránh race-condition khi OS tái dùng PID (PID reuse).
            verify = subprocess.run(
                ["tasklist.exe", "/FI", f"PID eq {mission_planner_pid}", "/FO", "CSV", "/NH"],
                capture_output=True, text=True, timeout=10, check=False,
            )
            if "MissionPlanner.exe" not in verify.stdout:
                self.logs.add(
                    "mission-planner", "warning",
                    f"PID {mission_planner_pid} không còn là Mission Planner; bỏ qua taskkill.",
                )
                self._mission_planner_pid = None
            else:
                result = subprocess.run(
                    [
                        "taskkill.exe",
                        "/PID",
                        str(mission_planner_pid),
                        "/T",
                        "/F",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=10,
                    check=False,
                )
                if result.returncode == 0:
                    self.logs.add(
                        "mission-planner",
                        "info",
                        f"Closed Windows PID {mission_planner_pid}",
                    )
                else:
                    self.logs.add(
                        "mission-planner",
                        "error",
                        result.stderr.strip()
                        or f"Could not close Windows PID {mission_planner_pid}",
                    )
                self._mission_planner_pid = None

        with self._lock:
            self._processes.clear()
            if not preserve_error:
                self._status = "stopped"
                self._active_config = {}
        self.logs.add("control-station", "info", "Managed simulation processes stopped")


@dataclass
class MavTask:
    command_id: str
    name: str
    deadline: float
    action: Callable[[], Any]
    cancelled: threading.Event = field(default_factory=threading.Event)
    completed: threading.Event = field(default_factory=threading.Event)
    result: Any = None
    error: Exception | None = None


class MavlinkManager:
    def __init__(self, logs: LogBroker, mission_output: Path) -> None:
        self.logs = logs
        self.mission_output = mission_output
        self._lock = threading.RLock()
        self._tasks: queue.Queue[MavTask] = queue.Queue(maxsize=32)
        
        self._rc_owner: str | None = None
        self._rc_generation: int = 0
        # Raw USB data is only stored here.  The MAVLink thread below is the
        # single writer of RC_CHANNELS_OVERRIDE, preventing transport races.
        self._external_rc = ExternalRcInput()
        self._uploaded_mission_id = 0
        self._connection = mavutil.mavlink_connection(
            MAVLINK_ENDPOINT,
            # ArduPilot accepts RC_CHANNELS_OVERRIDE only from MAV_GCS_SYSID.
            # The SITL default and MAVProxy both use 255.
            source_system=255,
            source_component=190,
            autoreconnect=True,
        )
        self._target_system = 1
        self._target_component = 1
        self._last_heartbeat = 0.0
        self._rc = [1500, 1500, 1000, 1500, 65535, 65535, 65535, 65535]
        self._rc_override_active = False
        self._last_rc_send = 0.0
        # Keep a landed SWA-LOW disarm request level-triggered so a missed
        # edge or transient MAVLink response does not leave the vehicle armed.
        self._external_disarm_inflight = False
        self._external_disarm_next_retry = 0.0
        self._hover_stable_since: float | None = None
        self._external_hover_recovery_active = False
        self._external_hover_recovery_stable_since: float | None = None
        self._external_hover_axes = [1500, 1500]
        self._external_hover_axes_updated_at = time.monotonic()
        self._status_messages: collections.deque[dict[str, Any]] = (
            collections.deque(maxlen=80)
        )
        self._state: dict[str, Any] = {
            "connected": False,
            "armed": False,
            "mode": "UNKNOWN",
            "systemStatus": 0,
            "homeSet": False,
            "heartbeatAge": None,
            "gps": {
                "lat": HOME_POSITION["lat"],
                "lon": HOME_POSITION["lon"],
                "fixType": 0,
                "satellites": 0,
                "hdop": None,
            },
            "position": {
                "altMsl": HOME_POSITION["alt"],
                "relativeAlt": 0.0,
                "vx": 0.0,
                "vy": 0.0,
                "vz": 0.0,
                "heading": 270.0,
            },
            "attitude": {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 270.0,
                "rollSpeed": 0.0,
                "pitchSpeed": 0.0,
                "yawSpeed": 0.0,
            },
            "hud": {
                "airspeed": 0.0,
                "groundspeed": 0.0,
                "throttle": 0,
                "climb": 0.0,
            },
            "battery": {
                "voltage": None,
                "current": None,
                "remaining": None,
            },
            "rc": {
                "roll": 1500,
                "pitch": 1500,
                "throttle": 1000,
                "yaw": 1500,
            },
            "mission": {
                "current": 0,
                "total": 0,
                "state": 0,
                "lastReached": None,
                "lastUpload": None,
            },
            "lastCommandAck": None,
            "prearm": {
                "ready": False,
                "message": "Waiting for telemetry"
            },
            "nav": {
                "valid": False,
                "updatedAt": None,
                "navBearing": 0.0,
                "targetBearing": 0.0,
                "wpDist": 0.0
            }
        }
        
        op_state = {
            "name": None,
            "status": "idle",
            "message": "Không có tác vụ bay đang chạy",
            "targetAltitude": None,
            "startedAt": None,
        }
        self._operation_state = op_state
        self._operation = OperationController(op_state, self._set_operation)
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="mavlink"
        )
        self._thread.start()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            state = json_copy(self._state)
            age = (
                time.monotonic() - self._last_heartbeat
                if self._last_heartbeat
                else None
            )
            connected = age is not None and age < 4.0
            state["heartbeatAge"] = age
            state["connected"] = connected
            state["rcOverrideActive"] = self._rc_override_active and connected
            if not connected:
                # Never expose stale flight authority after SITL has stopped.
                state["armed"] = False
                state["mode"] = "UNKNOWN"
            state["statusMessages"] = list(self._status_messages)[-30:]
            state["rcOwner"] = self._rc_owner if self._rc_owner is not None else "NONE"
            external = getattr(self, "_external_rc", None)
            state["externalController"] = (
                external.snapshot()
                if external is not None
                else {
                    "protocol": "P4RC/1", "received": False, "fresh": False,
                    "enabled": False, "failsafeLatched": False, "ageSeconds": None,
                    "sequence": None, "mcuMillis": None,
                    "channels": [
                        1500, 1500, 1000, 1500, 1500,
                        1500, 1500, 1500, 1500, 1500,
                    ],
                    "flags": 0,
                    "switch5": "MID",
                    "armSwitch": "LOW",
                    "qrtlSwitch": "LOW",
                    "modeSwitch": "LOW",
                    "modeRequest": "QLOITER",
                    "lastError": "Not initialised",
                    "lastAction": "",
                }
            )
            state["externalController"]["hoverRecoveryActive"] = bool(
                getattr(self, "_external_hover_recovery_active", False)
            )
            state["externalController"]["hoverStableSeconds"] = (
                max(0.0, time.monotonic() - self._hover_stable_since)
                if getattr(self, "_hover_stable_since", None) is not None
                else 0.0
            )
            mission_state = state.setdefault("mission", {})
            uploaded_mission_id = getattr(self, "_uploaded_mission_id", 0)
            mission_state["uploadedMissionId"] = (
                str(uploaded_mission_id) if uploaded_mission_id > 0 else None
            )
            # Expose total as fcCount so the frontend capability check
            # (state.vehicle.mission?.fcCount >= 2) works correctly.
            mission_state["fcCount"] = int(mission_state.get("total", 0))
            state["operation"] = self._operation.snapshot()

            # SIM-P1-01 prearm checks — time-bounded to last 10 seconds
            prearm_cutoff = now_ms() - 10_000
            msgs = [
                m['text'] for m in self._status_messages
                if 'prearm:' in m['text'].lower() and m['timestamp'] > prearm_cutoff
            ]
            msg = msgs[-1] if msgs else None

            gps_ok = (
                state['gps']['fixType'] >= 3
                and state['gps']['satellites'] >= 6
                and state['gps']['hdop'] is not None
                and state['gps']['hdop'] < 3.0
            )
            home_ok = bool(state.get("homeSet", False))
            battery_available = state['battery']['voltage'] is not None
            hb_ok = (
                state['heartbeatAge'] is not None
                and state['heartbeatAge'] < 2.0
            )

            # Battery telemetry is optional in Gazebo SITL. The simulated
            # vehicle reports voltage_battery=0 when no battery model is
            # configured, but ArduPilot can still pass its own arming checks.
            # Keep the missing voltage visible without blocking ARM / AUTO.
            ready = gps_ok and home_ok and hb_ok and (not msg)
            if not hb_ok: msg = "Mất kết nối MAVLink"
            elif not gps_ok: msg = "GPS chưa đủ tốt"
            elif not home_ok: msg = "Đang chờ EKF/AHRS thiết lập Home"

            checks = [
                {
                    "id": "heartbeat",
                    "label": "Kết nối MAVLink",
                    "status": "pass" if hb_ok else "fail",
                },
                {
                    "id": "gps",
                    "label": "GPS 3D và HDOP",
                    "status": "pass" if gps_ok else "fail",
                },
                {
                    "id": "home",
                    "label": "EKF/AHRS Home",
                    "status": "pass" if home_ok else "fail",
                },
                {
                    "id": "battery",
                    "label": (
                        "Telemetry pin"
                        if battery_available
                        else "Telemetry pin chưa được mô hình hóa trong SITL"
                    ),
                    "status": "pass" if battery_available else "warn",
                },
            ]
            state["prearm"] = {
                "ready": ready,
                "message": msg or "Sẵn sàng",
                "checks": checks
            }
            return state

    def _set_operation(
        self,
        status: str,
        message: str,
        target_altitude: float | None = None,
    ) -> None:
        """Update the state owned by ``OperationController``.

        The controller invokes this callback while holding its own lock, so this
        method must not acquire ``MavlinkManager._lock``.  Manager snapshots take
        the locks in the opposite order and doing so would introduce a deadlock.
        """
        current = self._operation_state
        current.update(
            {
                "status": status,
                "message": message,
                "targetAltitude": (
                    target_altitude
                    if target_altitude is not None
                    else current.get("targetAltitude")
                ),
            }
        )

    def _execute(self, action: Callable[[], Any], timeout: float = 4.0, name: str = "command") -> Any:
        task = MavTask(
            command_id=uuid.uuid4().hex,
            name=name,
            deadline=time.monotonic() + timeout,
            action=action
        )
        try:
            self._tasks.put_nowait(task)
        except queue.Full as exc:
            raise CommandQueueFull("Command queue full") from exc

        if not task.completed.wait(timeout):
            task.cancelled.set()
            raise CommandTimeout(task.command_id)
        if task.error:
            raise task.error
        return {"commandId": task.command_id, "result": task.result}

    def _run_task(self, task: MavTask) -> None:
        if task.cancelled.is_set() or time.monotonic() >= task.deadline:
            task.error = CommandExpired(task.command_id)
            task.completed.set()
            return
        try:
            import inspect
            sig = inspect.signature(task.action)
            if len(sig.parameters) > 0:
                task.result = task.action(CommandContext(task.cancelled))
            else:
                task.result = task.action()
        except Exception as e:
            task.error = e
        task.completed.set()

    def _require_connection(self) -> None:
        if (
            not self._last_heartbeat
            or time.monotonic() - self._last_heartbeat >= 4.0
        ):
            raise RuntimeError("No current MAVLink heartbeat")

    @staticmethod
    def _mav_result_name(result: int) -> str:
        entry = mavutil.mavlink.enums.get("MAV_RESULT", {}).get(int(result))
        return str(getattr(entry, "name", f"MAV_RESULT_{result}"))

    def _receive_message(self, timeout: float = 0.25) -> Any | None:
        message = self._connection.recv_match(blocking=True, timeout=timeout)
        if message is not None:
            self._handle_message(message)
        return message


    def _wait_until(self, predicate: Callable[[], bool], timeout: float, description: str, context: 'CommandContext | None' = None) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if context: context.checkpoint()
            message = self._receive_message(timeout=0.25)
            if predicate():
                return
        raise TimeoutError(description)

    def _set_mode_and_wait(self, requested: str, timeout: float, context: 'CommandContext | None' = None) -> None:
        if context: context.checkpoint()
        mapping = self._connection.mode_mapping() or {}
        if requested not in mapping:
            raise RuntimeError(f"Mode is unavailable: {requested}")
        self._connection.mav.set_mode_send(
            self._target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mapping[requested],
        )
        self._wait_until(
            lambda: str(self._state["mode"]).upper() == requested,
            timeout,
            f"FC did not enter {requested} within {timeout}s",
            context
        )
        
    def _wait_for_mode(self, mode: str, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            time.sleep(0.2)
            with self._lock:
                if str(self._state['mode']).upper() == mode.upper():
                    return
        raise TimeoutError(f'Mode {mode} not reached within {timeout}s')

    def _send_mode_command(self, mode: str) -> None:
        mapping = self._connection.mode_mapping() or {}
        if mode not in mapping:
            raise RuntimeError(f'Mode unavailable: {mode}')
        self._connection.mav.set_mode_send(
            self._target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mapping[mode],
        )

    def _center_rc_internal(self) -> None:
        self._send_rc_internal([1500, 1500, 1500, 1500])
        
    def neutralize_and_release_rc(
        self,
        lease: 'RcLease | None',
        reason: str = "",
        *,
        hover_throttle: bool = False,
    ) -> None:
        """Set RC to neutral and release the lease.

        When ``hover_throttle`` is True the throttle channel is left at 1500
        (mid-stick / hover) instead of 1000 (minimum).  Use this whenever
        the vehicle is airborne so that releasing control does not drop the
        throttle to minimum and cause an uncommanded descent.
        """
        try:
            if lease is not None:
                # Use 1500 (hover) throttle when airborne; 1000 only on ground.
                thr = 1500 if hover_throttle else 1000
                self._rc[:4] = [1500, 1500, thr, 1500]
                self.send_rc_with_lease(lease, self._rc[:4])
        except Exception:
            pass
        self.release_rc_lease(reason)

    def handover_rc_to_user(
        self, lease: 'RcLease', reason: str = "", *, throttle: int = 1500
    ) -> None:
        """Transfer RC ownership from an automation owner (e.g. TAKEOFF) to USER.

        Unlike ``neutralize_and_release_rc``, this does NOT send the all-zeros
        rc_channels_override that clears the GCS override.  In ArduPilot SITL
        without a physical radio, clearing the override drops RC3 to the SITL
        default (1000 = minimum throttle), which causes QLOITER to command
        maximum-rate descent immediately after takeoff.

        Instead we atomically promote the owner to "USER" and refresh the
        channels at centered roll/pitch/yaw and the requested throttle. The
        background RC keepalive loop will continue sending refreshes so
        ArduPilot never loses the override. The user can then click "Release
        RC" in the UI when ready.
        """
        throttle = int(clamp(throttle, 1000, 2000))
        neutral = [1500, 1500, throttle, 1500]
        try:
            self.send_rc_with_lease(lease, neutral)
        except Exception:
            pass
        with self._lock:
            # Promote to USER atomically — keep _rc_override_active True so the
            # keepalive loop continues flushing neutral values to ArduPilot.
            self._rc_owner = "USER"
            self._rc_generation += 1
            self._rc[:4] = neutral
            self._rc_override_active = True
            self._state["rc"].update(
                {"roll": 1500, "pitch": 1500, "throttle": throttle, "yaw": 1500}
            )
        self.logs.add(
            "flight",
            "info",
            f"{reason}: RC handed over to USER with centered axes and RC3 {throttle}",
        )

    def handover_rc_to_external(
        self, lease: 'RcLease', reason: str = "", *, throttle: int | None = None
    ) -> None:
        """Return RC ownership to the live MKR input after an automation step.

        The old transition path intentionally handed control to the dashboard
        user. Hardware-switch workflows must instead resume the external owner
        atomically, otherwise the radio appears connected but its stick values
        stop reaching MAVLink as soon as the transition completes.
        """
        external = self._external_rc.snapshot()
        if not external["enabled"] or not external["fresh"]:
            raise RuntimeError("MKR/iBUS input is no longer fresh")
        channels = [int(value) for value in external["channels"][:4]]
        if throttle is not None and abs(channels[2] - 1000) < 75:
            # Keep a safe fixed-wing cruise trim when the transmitter throttle
            # is still at the VTOL-safe low position during handover.
            channels[2] = int(clamp(throttle, 1000, 2000))
        try:
            self.send_rc_with_lease(lease, channels)
        except Exception:
            pass
        with self._lock:
            self._rc_owner = "EXTERNAL"
            self._rc_generation += 1
            self._rc[:4] = channels
            self._rc_override_active = True
        self._send_rc()
        self._external_rc.set_action(f"{reason}; đã trả quyền điều khiển cho tay cầm")
        self.logs.add(
            "external-rc",
            "info",
            f"{reason}: RC returned to EXTERNAL with RC3 {channels[2]}",
        )


    def _release_rc_internal(self) -> None:
        with self._lock:
            self._rc_owner = None
            self._rc_generation += 1
            self._rc_override_active = False
        self._connection.mav.rc_channels_override_send(
            self._target_system, self._target_component,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def _recent_arm_failure(self, after_timestamp: int) -> str | None:
        with self._lock:
            messages = [
                item["text"]
                for item in self._status_messages
                if item["timestamp"] >= after_timestamp
                and (
                    item["text"].lower().startswith("arm:")
                    or item["text"].lower().startswith("prearm:")
                )
            ]
        return messages[-1] if messages else None

    def arm(self, armed: bool) -> Any:
        if not armed:
            self.preempt_flight_operation("DISARM")
        def action() -> dict[str, Any]:
            self._require_connection()
            if armed:
                with self._lock:
                    current_mode = str(self._state.get("mode", "UNKNOWN")).upper()
                    current_altitude = float(
                        self._state.get("position", {}).get("relativeAlt", 0.0)
                    )
                if current_mode == "QRTL":
                    if current_altitude > EXTERNAL_RC_LANDED_ALTITUDE_M:
                        raise RuntimeError(
                            f"Cannot ARM while QRTL is airborne ({current_altitude:.1f} m)"
                        )
                    # QRTL is a return/landing mode and ArduPilot deliberately
                    # rejects ARM in it. After a completed QRTL landing, return
                    # to an armable hover mode before sending ARM.
                    self._set_mode_and_wait("QLOITER", 4.0)
                    self.logs.add(
                        "flight",
                        "info",
                        "QRTL was landed; switched to QLOITER before ARM",
                    )
            requested_at = now_ms()
            requested_label = "ARM" if armed else "DISARM"
            self.logs.add("flight", "info", f"{requested_label} requested")
            self._connection.mav.command_long_send(
                self._target_system,
                self._target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1.0 if armed else 0.0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            deadline = time.monotonic() + 6.0
            rejection: str | None = None
            rejection_deadline = 0.0
            acknowledgement: dict[str, Any] | None = None
            while time.monotonic() < deadline:
                message = self._receive_message(
                    min(0.25, max(0.01, deadline - time.monotonic()))
                )
                if message is not None and message.get_type() == "COMMAND_ACK":
                    if int(message.command) == int(
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
                    ):
                        result = int(message.result)
                        acknowledgement = {
                            "command": int(message.command),
                            "result": result,
                            "resultName": self._mav_result_name(result),
                        }
                        if result not in {
                            mavutil.mavlink.MAV_RESULT_ACCEPTED,
                            mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
                        }:
                            rejection = acknowledgement["resultName"]
                            rejection_deadline = time.monotonic() + 0.8
                with self._lock:
                    actual = bool(self._state["armed"])
                if actual == armed:
                    self.logs.add(
                        "flight",
                        "info",
                        f"{requested_label} confirmed by vehicle heartbeat",
                    )
                    return {
                        "requested": requested_label.lower(),
                        "confirmed": True,
                        "armed": actual,
                        "ack": acknowledgement,
                    }
                if rejection and time.monotonic() >= rejection_deadline:
                    reason = self._recent_arm_failure(requested_at)
                    detail = f": {reason}" if reason else ""
                    raise RuntimeError(
                        f"ArduPilot rejected {requested_label} "
                        f"({rejection}){detail}"
                    )
            if rejection:
                reason = self._recent_arm_failure(requested_at)
                detail = f": {reason}" if reason else ""
                raise RuntimeError(
                    f"ArduPilot rejected {requested_label} ({rejection}){detail}"
                )
            raise TimeoutError(
                f"ArduPilot did not confirm {requested_label} within 6 seconds"
            )

        return self._execute(action, timeout=8.0)

    def set_mode(self, mode: str) -> Any:
        requested = mode.upper()
        if requested not in ALLOWED_MODES:
            raise ValueError(f"Unsupported mode: {requested}")
        self.preempt_flight_operation(f"Mode change to {requested}")

        def action() -> dict[str, Any]:
            self._require_connection()
            mapping = self._connection.mode_mapping() or {}
            if requested not in mapping:
                raise RuntimeError(f"Mode is unavailable: {requested}")
            self._connection.mav.set_mode_send(
                self._target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mapping[requested],
            )
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline:
                self._receive_message(
                    min(0.25, max(0.01, deadline - time.monotonic()))
                )
                with self._lock:
                    actual = str(self._state["mode"]).upper()
                if actual == requested:
                    self.logs.add(
                        "flight", "info", f"Flight mode confirmed: {requested}"
                    )
                    return {"requested": requested, "confirmed": True}
            raise TimeoutError(
                f"ArduPilot did not enter {requested} within 4 seconds"
            )

        return self._execute(action, timeout=6.0)

    def set_rc(self, channel: int, value: int) -> Any:
        if type(value) is not int:
            raise ValueError("RC value must be an integer")
        if channel not in {1, 2, 3, 4}:
            raise ValueError("Only RC channels 1-4 are supported")
        if not 1000 <= value <= 2000:
            raise ValueError(f"RC{channel} must be between 1000 and 2000")

        def action() -> dict[str, Any]:
            self._require_connection()
            lease = self.acquire_rc('USER')
            self._rc[channel - 1] = int(value)
            self._rc_override_active = True
            self.send_rc_with_lease(lease, self._rc[:4])
            return {"channel": channel, "value": value}

        return self._execute(action)

    def center_controls(self, throttle: int = 1500) -> Any:
        throttle = int(clamp(throttle, 500, 2000))

        def action() -> dict[str, Any]:
            self._require_connection()
            lease = self.acquire_rc('USER')
            self._rc[:4] = [1500, 1500, throttle, 1500]
            self._rc_override_active = True
            self.send_rc_with_lease(lease, self._rc[:4])
            return {"rc": self._rc[:4]}

        return self._execute(action)

    def release_rc(self) -> Any:
        def action() -> dict[str, Any]:
            self._require_connection()
            self.release_rc_lease("User released RC")
            return {"released": True}

        return self._execute(action)


    def acquire_rc(
        self,
        owner: str,
        *,
        allow_user_handover: bool = False,
        preserve_external: bool = False,
    ) -> RcLease:
        with self._lock:
            current_owner = self._rc_owner
            automated_owner = owner in {"TAKEOFF", "ALTITUDE", "TRANSITION", "LANDING"}
            may_preempt_external = current_owner == "EXTERNAL" and automated_owner
            may_take_external_control = owner == "EXTERNAL" and current_owner == "USER"
            may_handover = (
                allow_user_handover and current_owner in {"USER", "EXTERNAL"}
            )
            if (
                current_owner not in {None, owner}
                and not may_preempt_external
                and not may_take_external_control
                and not may_handover
            ):
                raise RuntimeError(f"RC is owned by {self._rc_owner}")
            if (
                current_owner == "EXTERNAL"
                and (may_preempt_external or may_handover)
                and not preserve_external
            ):
                external = getattr(self, "_external_rc", None)
                if external is not None:
                    external.pause_for_operation(
                        f"External radio paused while {owner.lower()} owns RC"
                    )
            self._rc_generation += 1
            self._rc_owner = owner
            return RcLease(owner, self._rc_generation)

    def release_rc_lease(self, reason: str = "") -> None:
        with self._lock:
            self._rc_owner = None
            self._rc_generation += 1
            self._rc_override_active = False
        self._connection.mav.rc_channels_override_send(
            self._target_system, self._target_component,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def send_rc_with_lease(self, lease: RcLease, channels: list[int]) -> None:
        with self._lock:
            if self._rc_owner != lease.owner or self._rc_generation != lease.generation:
                raise OperationCancelled("RC lease expired")
        self._send_rc_internal(channels)

    def _send_rc_internal(self, channels: list[int]) -> None:
        c = [int(v) for v in channels] + [65535] * (8 - len(channels))
        self._connection.mav.rc_channels_override_send(
            self._target_system, self._target_component, *c
        )
        self._last_rc_send = time.monotonic()
        with self._lock:
            self._rc[:4] = c[:4]
            self._state["rc"].update({"roll": c[0], "pitch": c[1], "throttle": c[2], "yaw": c[3]})

    def _send_rc(self) -> None:
        self._connection.mav.rc_channels_override_send(
            self._target_system,
            self._target_component,
            *self._rc,
        )
        self._last_rc_send = time.monotonic()
        with self._lock:
            self._state["rc"] = {
                "roll": self._rc[0],
                "pitch": self._rc[1],
                "throttle": self._rc[2],
                "yaw": self._rc[3],
            }

    def external_rc_snapshot(self) -> dict[str, Any]:
        return self._external_rc.snapshot()

    def ingest_external_rc_line(self, line: str) -> dict[str, Any]:
        """Store one USB frame. It is not sent to MAVLink from the HTTP thread."""
        frame = parse_external_rc_line(line)
        self._external_rc.ingest(frame)
        return self._external_rc.snapshot()

    def set_external_rc_enabled(self, enabled: bool) -> Any:
        """Explicitly hand RC1-4 and guarded switch handling to the MKR."""
        def action() -> dict[str, Any]:
            self._require_connection()
            external = self._external_rc.snapshot()
            if enabled:
                if not external["fresh"]:
                    raise RuntimeError("MKR/iBUS input is not fresh")
                all_channels = [int(value) for value in external["channels"]]
                if all_channels[EXTERNAL_RC_ARM_CHANNEL] > EXTERNAL_RC_SWITCH_LOW:
                    raise RuntimeError("Đưa SWA/CH7 về LOW trước khi trao quyền radio")
                if all_channels[EXTERNAL_RC_QRTL_CHANNEL] > EXTERNAL_RC_SWITCH_LOW:
                    raise RuntimeError("Đưa SWB/CH8 về LOW trước khi trao quyền radio")
                if all_channels[EXTERNAL_RC_MODE_CHANNEL] > EXTERNAL_RC_SWITCH_LOW:
                    raise RuntimeError("Đưa SWC/CH9 về LOW (QLOITER) trước khi trao quyền radio")
                channels = all_channels[:4]
                state = self.snapshot()
                if state["armed"]:
                    expected_throttle = (
                        FIXED_WING_CRUISE_RC3
                        if str(state["mode"]).upper() == "FBWB"
                        else 1500
                    )
                    if abs(channels[2] - expected_throttle) > 75:
                        raise RuntimeError(
                            "Set throttle near the current hold value before taking control "
                            f"(expected about RC3 {expected_throttle})"
                        )
                    if any(abs(channels[index] - 1500) > 100 for index in (0, 1, 3)):
                        raise RuntimeError("Center roll, pitch and yaw before taking control")
                elif channels[2] > 1100:
                    raise RuntimeError("Throttle must be low before enabling external RC while disarmed")

                lease = self.acquire_rc("EXTERNAL")
                self._external_rc.set_enabled(True)
                self._rc[:4] = channels
                self._rc_override_active = True
                self.send_rc_with_lease(lease, channels)
                self.logs.add("external-rc", "info", "External radio control enabled")
            else:
                self._external_rc.pause_for_operation("External radio control disabled by operator")
                with self._lock:
                    was_external = self._rc_owner == "EXTERNAL"
                    if was_external:
                        self._rc_generation += 1
                        self._rc_owner = "USER"
                        throttle = 1500 if self._state.get("armed") else 1000
                        self._rc[:4] = [1500, 1500, throttle, 1500]
                        self._rc_override_active = True
                if was_external:
                    self._send_rc()
                self.logs.add("external-rc", "info", "External radio control disabled; RC held neutral")
            return self._external_rc.snapshot()

        return self._execute(action)

    def _trip_external_rc_failsafe(self, reason: str) -> None:
        """Bound a USB/iBUS loss without clearing airborne throttle override."""
        with self._lock:
            if self._rc_owner != "EXTERNAL":
                return
            self._rc_generation += 1
            self._rc_owner = "USER"
            self._rc[:4] = [1500, 1500, 1500, 1500]
            self._rc_override_active = True
            connected = bool(self._last_heartbeat and time.monotonic() - self._last_heartbeat < 4.0)
            armed = bool(self._state.get("armed"))
            mode = str(self._state.get("mode", "UNKNOWN")).upper()
        self._external_rc.trip_failsafe(reason)
        self._send_rc()
        if connected and armed and mode != "QLOITER":
            try:
                self._send_mode_command("QLOITER")
            except Exception as error:
                self.logs.add("external-rc", "error", f"External RC failsafe mode request: {error}")
        self.logs.add("external-rc", "error", f"{reason}; holding neutral RC and requesting QLOITER")

    def _external_arm_worker(self, request_arm: bool) -> None:
        try:
            state = self.snapshot()
            if request_arm:
                if state.get("armed"):
                    self._external_rc.set_action("SWA HIGH: máy bay đã ARMED")
                    return
                channels = self._external_rc.snapshot()["channels"]
                if int(channels[2]) > 1100:
                    raise RuntimeError("CH3 phải ở LOW trước khi ARM bằng SWA")
                self.arm(True)
                self._external_rc.set_action("SWA HIGH: ARM thành công")
            else:
                if not state.get("armed"):
                    self._external_rc.set_action("SWA LOW: máy bay đã DISARMED")
                    return
                altitude = float(state.get("position", {}).get("relativeAlt", 0.0))
                if altitude > EXTERNAL_RC_LANDED_ALTITUDE_M:
                    self._external_rc.set_action(
                        f"SWA LOW bị bỏ qua khi đang bay ({altitude:.1f} m); dùng SWB/CH8 để QRTL"
                    )
                    return
                self.arm(False)
                self._external_rc.set_action("SWA LOW: DISARM thành công")
        except Exception as error:
            self._external_rc.set_action(f"Lệnh SWA bị từ chối: {error}")
            self.logs.add("external-rc", "error", f"SWA arm request: {error}")
        finally:
            if not request_arm:
                with self._lock:
                    self._external_disarm_inflight = False
                    self._external_disarm_next_retry = (
                        time.monotonic() + EXTERNAL_RC_DISARM_RETRY_S
                    )

    def _external_mode_worker(self, mode: str) -> None:
        try:
            if mode == "QRTL":
                if not self.snapshot().get("armed"):
                    raise RuntimeError("Không thể QRTL khi máy bay đang DISARMED")
                self.preempt_flight_operation("External SWB QRTL")
                self.set_mode("QRTL")
                self._resume_external_rc("SWB HIGH: đã chuyển sang QRTL")
                return

            self.preempt_flight_operation("External SWC QLOITER")
            self.abort_to_qloiter()
            self._resume_external_rc("SWC LOW: đã về QLOITER")
        except Exception as error:
            self._external_rc.set_action(f"Lệnh {mode} bị từ chối: {error}")
            self.logs.add("external-rc", "error", f"External {mode} request: {error}")

    def _external_fixed_wing_worker(self, request: str) -> None:
        try:
            state = self.snapshot()
            current_mode = str(state.get("mode", "UNKNOWN")).upper()
            if current_mode in {"FBWA", "FBWB"}:
                if current_mode != "FBWB":
                    self.set_mode("FBWB")
                self._resume_external_rc("SWC HIGH: giữ chế độ cánh bằng FBWB")
                return
            self.transition_to_fixed_wing(return_to_external=True)
            self._external_rc.set_action(
                "SWC MID/HIGH: đang chạy workflow chuyển tiếp an toàn"
            )
        except Exception as error:
            self._external_rc.set_action(f"Chuyển cánh bằng bị từ chối: {error}")
            self.logs.add("external-rc", "error", f"External fixed-wing request: {error}")

    def _hover_stability_age(self, state: dict[str, Any]) -> float:
        """Track continuous calm QLOITER time for the transition interlock."""
        now = time.monotonic()
        is_hover = (
            bool(state.get("armed"))
            and str(state.get("mode", "")).upper() == "QLOITER"
            and hover_attitude_is_stable(state)
        )
        with self._lock:
            if not is_hover:
                self._hover_stable_since = None
                return 0.0
            if self._hover_stable_since is None:
                self._hover_stable_since = now
            return now - self._hover_stable_since

    def _shape_external_hover_channels(
        self, requested: list[int], state: dict[str, Any]
    ) -> list[int]:
        """Limit pilot roll/pitch in hover and hold neutral during recovery.

        This is a simulation safety envelope, not an ArduPilot gain change.
        It lets the pilot translate the vehicle in QLOITER but prevents one
        abrupt full-stick input from leaving the folded-wing model in a large
        self-sustaining pitch/roll oscillation.
        """
        now = time.monotonic()
        is_hover = (
            bool(state.get("armed"))
            and str(state.get("mode", "")).upper() == "QLOITER"
        )
        if not is_hover:
            with self._lock:
                self._external_hover_recovery_active = False
                self._external_hover_recovery_stable_since = None
                self._external_hover_axes = [int(requested[0]), int(requested[1])]
                self._external_hover_axes_updated_at = now
            return requested

        attitude = state.get("attitude") or {}
        exceeds_envelope = (
            abs(float(attitude.get("roll", 0.0))) > EXTERNAL_HOVER_RECOVERY_ENTER_TILT_DEG
            or abs(float(attitude.get("pitch", 0.0))) > EXTERNAL_HOVER_RECOVERY_ENTER_TILT_DEG
            or abs(float(attitude.get("rollSpeed", 0.0))) > EXTERNAL_HOVER_RECOVERY_ENTER_RATE_DPS
            or abs(float(attitude.get("pitchSpeed", 0.0))) > EXTERNAL_HOVER_RECOVERY_ENTER_RATE_DPS
        )
        stable = hover_attitude_is_stable(state)
        entered_recovery = False
        completed_recovery = False
        with self._lock:
            if exceeds_envelope and not self._external_hover_recovery_active:
                self._external_hover_recovery_active = True
                self._external_hover_recovery_stable_since = None
                self._external_hover_axes = [1500, 1500]
                self._external_hover_axes_updated_at = now
                entered_recovery = True
            if self._external_hover_recovery_active:
                if stable:
                    if self._external_hover_recovery_stable_since is None:
                        self._external_hover_recovery_stable_since = now
                    elif (
                        now - self._external_hover_recovery_stable_since
                        >= EXTERNAL_HOVER_RECOVERY_HOLD_S
                    ):
                        self._external_hover_recovery_active = False
                        self._external_hover_recovery_stable_since = None
                        self._external_hover_axes = [1500, 1500]
                        self._external_hover_axes_updated_at = now
                        completed_recovery = True
                else:
                    self._external_hover_recovery_stable_since = None
                recovery_active = self._external_hover_recovery_active
            else:
                recovery_active = False

            if recovery_active:
                shaped = [1500, 1500, 1500, 1500]
            else:
                elapsed = min(0.10, max(0.0, now - self._external_hover_axes_updated_at))
                max_step = max(1, int(EXTERNAL_HOVER_AXIS_SLEW_PWM_PER_S * elapsed))
                for index in (0, 1):
                    target = int(clamp(
                        requested[index],
                        1500 - EXTERNAL_HOVER_AXIS_LIMIT_PWM,
                        1500 + EXTERNAL_HOVER_AXIS_LIMIT_PWM,
                    ))
                    previous = self._external_hover_axes[index]
                    self._external_hover_axes[index] = int(clamp(
                        target, previous - max_step, previous + max_step
                    ))
                self._external_hover_axes_updated_at = now
                shaped = [
                    self._external_hover_axes[0],
                    self._external_hover_axes[1],
                    int(requested[2]),
                    int(requested[3]),
                ]

        if entered_recovery:
            self._external_rc.set_action(
                "HOVER RECOVERY: giữ RC trung tính đến khi máy bay ổn định"
            )
            self.logs.add(
                "external-rc", "warning",
                "Hover attitude envelope exceeded; holding neutral RC in QLOITER",
            )
        elif completed_recovery:
            self._external_rc.set_action(
                "HOVER RECOVERY hoàn tất: tay lái QLOITER được giới hạn trở lại"
            )
            self.logs.add("external-rc", "info", "Hover recovery completed")
        return shaped

    def _external_hover_recovery_is_active(self) -> bool:
        with self._lock:
            return self._external_hover_recovery_active

    def _resume_external_rc(self, reason: str = "") -> None:
        external = self._external_rc.snapshot()
        if not external["enabled"] or not external["fresh"]:
            raise RuntimeError("MKR/iBUS input is no longer fresh")
        channels = [int(value) for value in external["channels"][:4]]
        channels = self._shape_external_hover_channels(channels, self.snapshot())
        with self._lock:
            self._rc_owner = "EXTERNAL"
            self._rc_generation += 1
            self._rc[:4] = channels
            self._rc_override_active = True
        self._send_rc()
        if reason:
            self._external_rc.set_action(reason)

    def _service_external_rc(self) -> None:
        """Service guarded MKR switches and stream RC1..RC4 at up to 100 Hz."""
        external = self._external_rc.snapshot()
        if not external["enabled"]:
            return
        if not external["fresh"]:
            self._trip_external_rc_failsafe("External USB/iBUS input timed out")
            return

        # Recovery commands have priority and may interrupt an automated
        # transition. This makes SWB/QRTL a real safety switch.
        event = self._external_rc.pop_event()
        if event == "qrtl":
            threading.Thread(
                target=self._external_mode_worker,
                args=("QRTL",),
                daemon=True,
                name="external-rc-qrtl",
            ).start()
            return
        if event == "qloiter":
            threading.Thread(
                target=self._external_mode_worker,
                args=("QLOITER",),
                daemon=True,
                name="external-rc-qloiter",
            ).start()
            return

        vehicle = self.snapshot()
        stable_duration = self._hover_stability_age(vehicle)
        landed_disarm = (
            external.get("armSwitch") == "LOW"
            and bool(vehicle.get("armed"))
            and float(vehicle.get("position", {}).get("relativeAlt", 0.0))
            <= EXTERNAL_RC_LANDED_ALTITUDE_M
        )
        with self._lock:
            owner = self._rc_owner
        if owner != "EXTERNAL" and event != "arm_low" and not landed_disarm:
            # Automated takeoff/transition owns RC until it completes. A QRTL
            # or QLOITER event above is the only deliberate interruption.
            return

        if event == "arm_high":
            threading.Thread(
                target=self._external_arm_worker,
                args=(True,),
                daemon=True,
                name="external-rc-arm",
            ).start()
            return
        # SWA LOW is edge-triggered for responsiveness and level-triggered
        # while landed for reliability.  If a disarm command is rejected or a
        # MAVLink response is missed, the next service pass retries it after
        # the previous worker has finished.  Above 2 m the worker refuses the
        # command, so an airborne vehicle cannot be accidentally disarmed.
        if event == "arm_low" or landed_disarm:
            with self._lock:
                if getattr(self, "_external_disarm_inflight", False):
                    return
                if (
                    event != "arm_low"
                    and time.monotonic()
                    < getattr(self, "_external_disarm_next_retry", 0.0)
                ):
                    return
                self._external_disarm_inflight = True
            threading.Thread(
                target=self._external_arm_worker,
                args=(False,),
                daemon=True,
                name="external-rc-disarm",
            ).start()
            return
        if event in {"transition_fw", "fixed_wing"}:
            current_mode = str(vehicle.get("mode", "UNKNOWN")).upper()
            if current_mode == "QLOITER" and self._external_hover_recovery_is_active():
                self._external_rc.set_action(
                    "SWC blocked: HOVER RECOVERY is holding the vehicle level"
                )
                return
            if (
                current_mode == "QLOITER"
                and stable_duration < EXTERNAL_HOVER_RECOVERY_HOLD_S
            ):
                self._external_rc.set_action(
                    "SWC blocked: keep QLOITER stable for 1.5 seconds before fixed-wing transition"
                )
                return
            threading.Thread(
                target=self._external_fixed_wing_worker,
                args=(event,),
                daemon=True,
                name="external-rc-fixed-wing",
            ).start()
            return

        if time.monotonic() - self._last_rc_send < EXTERNAL_RC_SEND_INTERVAL_S:
            return
        channels = [int(value) for value in external["channels"][:4]]
        channels = self._shape_external_hover_channels(channels, vehicle)
        with self._lock:
            if self._rc_owner != "EXTERNAL":
                return
            self._rc[:4] = channels
            self._rc_override_active = True
        self._send_rc()

    def takeoff(self, altitude: float) -> dict[str, Any]:
        target = float(clamp(altitude, 3.0, 120.0))
        state = self.snapshot()
        if not state["connected"]:
            raise RuntimeError("MAVLink disconnected")
        if state["gps"]["fixType"] < 3:
            raise RuntimeError("GPS fix required for takeoff")
        if abs(state["attitude"]["roll"]) >= 10 or abs(state["attitude"]["pitch"]) >= 10:
            raise RuntimeError("Vehicle is not level")
        if state["position"]["relativeAlt"] >= 2.0:
            raise RuntimeError("Vehicle is already airborne")
            
        context = self._operation.start("takeoff", target)
        threading.Thread(
            target=self._takeoff_worker,
            args=(context, target,),
            daemon=True,
            name="takeoff",
        ).start()
        return {
            "operation": "takeoff",
            "status": "running",
            "targetAltitude": target,
        }

    def _takeoff_worker(self, context: OperationContext, target_altitude: float) -> None:
        lease: 'RcLease | None' = None
        takeoff_succeeded = False
        return_to_external = bool(self._external_rc.snapshot()["enabled"])
        try:
            context.raise_if_cancelled()
            self.logs.add("flight", "info", f"Automatic QLOITER takeoff requested to {target_altitude:.1f} m")
            
            lease = self.acquire_rc(
                "TAKEOFF",
                preserve_external=return_to_external,
            )
            self._rc[:4] = [1500, 1500, 1000, 1500]
            self._rc_override_active = True
            self.send_rc_with_lease(lease, self._rc[:4])
            
            self._send_mode_command("QLOITER")
            self._wait_for_mode("QLOITER", 5.0)
            context.raise_if_cancelled()
            
            if not self.snapshot()["armed"]:
                self._operation.finish_if_current(context, "running", "Đang ARM máy bay")
                self.arm(True)
            context.raise_if_cancelled()

            start_altitude = float(self.snapshot()["position"]["relativeAlt"])
            # Use throttle 1900 (high power) for climbing in SITL.
            # 1800 proved insufficient for slow-climbing VTOL SITL configs.
            throttle = 1900
            self._rc[2] = throttle
            self.send_rc_with_lease(lease, self._rc[:4])
            
            self._operation.finish_if_current(context, "running", f"Đang lên cao: {start_altitude:.1f}/{target_altitude:.1f} m")
            
            # Use a conservative 0.2 m/s expected climb rate + 60 s buffer.
            # SITL VTOL can climb as slowly as 0.3 m/s; the stall detector
            # (below) is the real safety net for a stuck vehicle — the
            # deadline is just a hard upper bound.
            timeout = clamp(abs(target_altitude - start_altitude) / 0.2 + 60, 30, 600)
            climb_deadline = time.monotonic() + timeout
            
            last_alt = start_altitude
            last_progress_time = time.monotonic()
            
            while time.monotonic() < climb_deadline:
                context.raise_if_cancelled()
                state = self.snapshot()
                if not state["connected"]:
                    raise RuntimeError("MAVLink disconnected during takeoff")
                if not state["armed"]:
                    raise RuntimeError("Vehicle disarmed during takeoff")
                if str(state["mode"]).upper() != "QLOITER":
                    raise RuntimeError("Mode changed from QLOITER during takeoff")
                    
                altitude = float(state["position"]["relativeAlt"])
                remaining = target_altitude - altitude
                
                if abs(altitude - last_alt) > 0.25:
                    last_alt = altitude
                    last_progress_time = time.monotonic()
                elif time.monotonic() - last_progress_time > 15.0:
                    raise RuntimeError("Takeoff stalled: no altitude progress for 15 seconds")
                
                # QLOITER interprets RC3 around 1500 as zero vertical speed.
                # Hand over only when genuinely close to the requested height;
                # the previous 1.5 m band completed a 5 m takeoff near 3.6 m.
                completion_tolerance_m = takeoff_altitude_tolerance_m(target_altitude)
                if remaining <= completion_tolerance_m:
                    self.logs.add("flight", "info", f"Takeoff altitude reached ({altitude:.1f} m); returning RC to the active controller")
                    self._operation.finish_if_current(context, "completed", f"Đã đạt độ cao {altitude:.1f} m; đang giữ vị trí")
                    takeoff_succeeded = True
                    return
                    
                # Taper before entering the completion band. Both branches must
                # remain wider than the completion tolerance so they are used.
                desired_throttle = takeoff_throttle_for_remaining(remaining)
                    
                if desired_throttle != throttle:
                    throttle = desired_throttle
                    self._rc[2] = throttle
                    self.send_rc_with_lease(lease, self._rc[:4])
                    
                self._operation.finish_if_current(context, "running", f"Đang lên cao: {altitude:.1f}/{target_altitude:.1f} m · RC3 {throttle}")
                time.sleep(0.2)
                
            raise RuntimeError(f"Takeoff altitude was not reached within {timeout} seconds")
        except OperationCancelled:
            self._operation.finish_if_current(context, "cancelled", "Đã hủy tác vụ")
        except Exception as error:
            self.logs.add("flight", "error", f"Takeoff automation: {error}")
            self._operation.finish_if_current(context, "failed", f"Cất cánh thất bại: {error}")
        finally:
            if lease is not None:
                # On SUCCESS: hand RC over to USER at neutral (1500) so that
                #   QLOITER continues to hold altitude.  Releasing the override
                #   entirely (all-zeros) drops RC3 to SITL default=1000 which
                #   makes QLOITER command maximum descent — causing the crash.
                # On FAILURE / CANCEL: release fully; if airborne keep throttle
                #   at 1500 to prevent an additional uncommanded drop.
                if takeoff_succeeded:
                    if return_to_external:
                        self.handover_rc_to_external(
                            lease,
                            reason="takeoff completed",
                            throttle=1500,
                        )
                    else:
                        self.handover_rc_to_user(lease, reason="takeoff completed")
                    lease = None  # prevent double-release
                else:
                    airborne = (
                        float(self.snapshot().get("position", {}).get("relativeAlt", 0)) > 2.0
                    )
                    self.neutralize_and_release_rc(
                        lease,
                        reason="takeoff finished",
                        hover_throttle=airborne,
                    )

    def climb_to_altitude(self, target_altitude: float) -> dict[str, Any]:
        target = float(clamp(target_altitude, 3.0, 150.0))
        current_mode = str(self.snapshot()["mode"]).upper()
        if current_mode != "FBWB":
            raise RuntimeError(f"Auto altitude-hold only works in FBWB (current mode: {current_mode})")
            
        context = self._operation.start("altitude", target)
        threading.Thread(
            target=self._climb_worker,
            args=(context, target,),
            daemon=True,
            name="altitude-hold",
        ).start()
        return {"operation": "altitude", "status": "running", "targetAltitude": target}

    def transition_to_fixed_wing(
        self,
        throttle: int = TRANSITION_RC3,
        minimum_altitude: float = 25.0,
        *,
        return_to_external: bool = False,
    ) -> dict[str, Any]:
        throttle = int(clamp(throttle, 1600, 1850))
        minimum_altitude = float(clamp(minimum_altitude, 25.0, 100.0))
        state = self.snapshot()
        stable_duration = self._hover_stability_age(state)
        reason = fixed_wing_transition_precondition(
            state, minimum_altitude, stable_duration
        )
        if reason:
            raise RuntimeError(reason)

        context = self._operation.start("transition_fw", None)
        threading.Thread(
            target=self._transition_to_fixed_wing_worker,
            args=(context, throttle, minimum_altitude, return_to_external),
            daemon=True,
            name="transition-fw",
        ).start()
        return {
            "operation": "transition_fw",
            "status": "running",
            "transitionThrottle": throttle,
            "cruiseThrottle": FIXED_WING_CRUISE_RC3,
            "minimumAltitude": minimum_altitude,
            "requiredStableSeconds": EXTERNAL_HOVER_RECOVERY_HOLD_S,
        }

    def _transition_to_fixed_wing_worker(
        self,
        context: OperationContext,
        throttle: int,
        minimum_altitude: float,
        return_to_external: bool = False,
    ) -> None:
        lease: RcLease | None = None
        handed_over = False
        start_altitude = 0.0
        requested_at = now_ms()
        airspeed_reached = False
        transition_done = False
        try:
            context.raise_if_cancelled()
            state = self.snapshot()
            stable_duration = self._hover_stability_age(state)
            reason = fixed_wing_transition_precondition(
                state, minimum_altitude, stable_duration
            )
            if reason:
                raise RuntimeError(reason)
            start_altitude = float(state["position"]["relativeAlt"])

            lease = self.acquire_rc(
                "TRANSITION",
                allow_user_handover=True,
                preserve_external=return_to_external,
            )
            self._rc[:4] = [1500, 1500, throttle, 1500]
            self._rc_override_active = True
            self.send_rc_with_lease(lease, self._rc[:4])
            requested_at = now_ms()
            self.logs.add(
                "flight",
                "info",
                f"Bounded FBWB transition requested at {start_altitude:.1f} m "
                f"with RC3 {throttle}; cruise will return to RC3 "
                f"{FIXED_WING_CRUISE_RC3}",
            )
            self._send_mode_command("FBWB")
            self._wait_for_mode("FBWB", 5.0)

            deadline = time.monotonic() + 50.0
            while time.monotonic() < deadline:
                context.raise_if_cancelled()
                state = self.snapshot()
                if not state["connected"]:
                    raise RuntimeError(
                        "MAVLink disconnected during transition"
                    )
                if not state["armed"]:
                    raise RuntimeError(
                        "Vehicle disarmed during transition"
                    )
                if str(state["mode"]).upper() != "FBWB":
                    raise RuntimeError(
                        f"Mode changed away from FBWB ({state['mode']})"
                    )

                altitude = float(state["position"]["relativeAlt"])
                roll = abs(float(state["attitude"]["roll"]))
                pitch = abs(float(state["attitude"]["pitch"]))
                if altitude < 12.0 or altitude < start_altitude - 15.0:
                    raise RuntimeError(
                        f"Unsafe altitude loss: {altitude:.1f} m"
                    )
                if roll > 35.0 or pitch > 25.0:
                    raise RuntimeError(
                        f"Unsafe attitude: roll {roll:.1f}, pitch {pitch:.1f}"
                    )

                for message in state.get("statusMessages", []):
                    if int(message.get("timestamp", 0)) < requested_at:
                        continue
                    text = str(message.get("text", "")).lower()
                    if text.startswith("transition airspeed reached"):
                        airspeed_reached = True
                    if text.startswith("transition") and "done" in text:
                        transition_done = True

                if airspeed_reached and transition_done:
                    if return_to_external:
                        self.handover_rc_to_external(
                            lease,
                            reason="fixed-wing transition completed",
                            throttle=FIXED_WING_CRUISE_RC3,
                        )
                    else:
                        self.handover_rc_to_user(
                            lease,
                            reason="fixed-wing transition completed",
                            throttle=FIXED_WING_CRUISE_RC3,
                        )
                    handed_over = True
                    lease = None
                    self._operation.finish_if_current(
                        context,
                        "completed",
                        f"Đã chuyển cánh bằng ở {altitude:.1f} m; "
                        f"FBWB giữ RC3 {FIXED_WING_CRUISE_RC3} (cruise)",
                    )
                    return

                self._operation.finish_if_current(
                    context,
                    "running",
                    f"Đang chuyển cánh bằng: {altitude:.1f} m · "
                    f"airspeed {'OK' if airspeed_reached else 'đang chờ'}",
                )
                time.sleep(0.2)

            raise RuntimeError(
                "ArduPilot did not confirm a completed transition within 50 s"
            )
        except OperationCancelled:
            self._operation.finish_if_current(
                context, "cancelled", "Đã hủy chuyển cánh bằng"
            )
        except Exception as error:
            self.logs.add(
                "flight", "error", f"Fixed-wing transition: {error}"
            )
            self._operation.finish_if_current(
                context,
                "failed",
                f"Chuyển cánh bằng thất bại: {error}; đang về QLOITER",
            )
            try:
                if lease is not None:
                    self._rc[:4] = [1500, 1500, 1500, 1500]
                    self.send_rc_with_lease(lease, self._rc[:4])
                if self.snapshot().get("connected"):
                    self._send_mode_command("QLOITER")
                    self._wait_for_mode("QLOITER", 5.0)
            except Exception as recovery_error:
                self.logs.add(
                    "flight",
                    "error",
                    f"Transition recovery: {recovery_error}",
                )
        finally:
            if lease is not None and not handed_over:
                try:
                    if return_to_external:
                        self._resume_external_rc("Transition kết thúc; tay cầm vẫn được giữ quyền")
                    else:
                        self.handover_rc_to_user(
                            lease,
                            reason="fixed-wing transition recovery",
                            throttle=1500,
                        )
                except Exception:
                    self.release_rc_lease(
                        "fixed-wing transition cleanup"
                    )

    def _climb_worker(self, context: OperationContext, target_altitude: float) -> None:
        max_offset = 220
        tolerance_m = 1.5
        try:
            context.raise_if_cancelled()
            self.logs.add("flight", "info", f"Auto altitude-hold requested to {target_altitude:.1f} m (FBWB)")
            
            lease = self.acquire_rc("ALTITUDE")
            
            deadline = time.monotonic() + 120
            while time.monotonic() < deadline:
                context.raise_if_cancelled()
                state = self.snapshot()
                if not state["connected"]:
                    raise RuntimeError("MAVLink disconnected during altitude change")
                if str(state["mode"]).upper() != "FBWB":
                    raise RuntimeError("Mode changed away from FBWB during altitude change")
                    
                altitude = float(state["position"]["relativeAlt"])
                error = target_altitude - altitude
                if abs(error) <= tolerance_m:
                    self._rc[1] = 1500
                    self.send_rc_with_lease(lease, self._rc[:4])
                    self.logs.add("flight", "info", f"Target altitude reached ({altitude:.1f} m); pitch stick released")
                    self._operation.finish_if_current(context, "completed", f"Đã đạt độ cao {altitude:.1f} m; FBWB đang giữ nguyên")
                    return
                    
                offset = int(clamp(abs(error) * 25, 60, max_offset))
                pitch = 1500 - offset if error > 0 else 1500 + offset
                
                self._rc[1] = pitch
                self.send_rc_with_lease(lease, self._rc[:4])
                self._operation.finish_if_current(context, "running", f"Đang chỉnh độ cao: {altitude:.1f}/{target_altitude:.1f} m · RC2 {pitch}")
                time.sleep(0.2)
                
            raise RuntimeError("Altitude target was not reached within 120 seconds")
        except OperationCancelled:
            self._operation.finish_if_current(context, "cancelled", "Đã hủy tác vụ")
        except Exception as error:
            self.logs.add("flight", "error", f"Altitude-hold automation: {error}")
            self._operation.finish_if_current(context, "failed", f"Chỉnh độ cao thất bại: {error}")
        finally:
            if 'lease' in locals() and lease is not None:
                self.neutralize_and_release_rc(lease, reason="climb finished")

    def preempt_flight_operation(self, reason: str, *, wait_timeout_s: float = 1.0) -> None:
        context = self._operation.cancel_active(reason)
        if context is not None:
            deadline = time.monotonic() + wait_timeout_s
            while time.monotonic() < deadline and self._operation.is_current(context):
                time.sleep(0.05)
        self.neutralize_and_release_rc(None, reason)
        self._release_rc_internal()

    def qland(self) -> Any:
        self.preempt_flight_operation("QLAND override")
        return self.set_mode("QLAND")

    def abort_to_qloiter(self) -> Any:
        self.preempt_flight_operation("ABORT override")
        def action(context: 'CommandContext' = None) -> dict[str, Any]:
            self._require_connection()
            # Do not clear the override while airborne.  In SITL that returns
            # RC3 to the default 1000 and can turn a QLOITER recovery into an
            # uncommanded descent before the RC timeout is noticed.
            with self._lock:
                self._rc_generation += 1
                self._rc_owner = "USER"
                self._rc[:4] = [1500, 1500, 1500, 1500]
                self._rc_override_active = True
            self._send_rc()
            self._set_mode_and_wait("QLOITER", 5.0, context)
            return {"confirmed": True, "mode": "QLOITER"}
        return self._execute(action, timeout=7.0)

    def clear_mission(self) -> Any:
        def action() -> dict[str, Any]:
            self._require_connection()
            self._connection.mav.mission_clear_all_send(
                self._target_system, self._target_component
            )
            self._wait_until(
                lambda: self._state["mission"]["total"] == 0,
                3.0,
                "FC did not ack mission clear"
            )
            return {"cleared": True}

        return self._execute(action)

    def start_mission(self, expected_mission_id: int | None = None) -> Any:
        def action(context: 'CommandContext' = None) -> dict[str, Any]:
            self._require_connection()
            if self._state["gps"]["fixType"] < 3:
                raise RuntimeError("GPS fix required to start mission")
            if not self._state.get("homeSet", False):
                raise RuntimeError(
                    "EKF/AHRS chưa thiết lập Home. Hãy chờ trạng thái "
                    "pre-arm chuyển sang Sẵn sàng rồi thử lại."
                )

            if expected_mission_id is not None and self._uploaded_mission_id != expected_mission_id:
                raise RuntimeError("Khác phiên bản mission. Vui lòng upload lại trước khi bay.")

            if self._state["mission"]["total"] < 2:
                raise RuntimeError("No accepted mission is loaded on FC")

            # --- Auto-ARM if disarmed and prearm checks pass ---
            if not self._state["armed"]:
                self.logs.add("flight", "info", "start_mission: vehicle not armed — attempting ARM now")
                requested_at = now_ms()
                self._connection.mav.command_long_send(
                    self._target_system, self._target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1.0, 0, 0, 0, 0, 0, 0,
                )
                # Wait up to 8 s for ARM confirmation via heartbeat
                arm_deadline = time.monotonic() + 8.0
                arm_confirmed = False
                rejection: str | None = None
                rejection_deadline = 0.0
                while time.monotonic() < arm_deadline:
                    msg = self._receive_message(min(0.3, arm_deadline - time.monotonic()))
                    if msg is not None and msg.get_type() == "COMMAND_ACK":
                        if int(msg.command) == int(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM):
                            result = int(msg.result)
                            if result not in {
                                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
                            }:
                                rejection = self._mav_result_name(result)
                                rejection_deadline = time.monotonic() + 0.8
                    with self._lock:
                        if bool(self._state["armed"]):
                            arm_confirmed = True
                            break
                    if rejection and time.monotonic() >= rejection_deadline:
                        break
                if not arm_confirmed:
                    reason = self._recent_arm_failure(requested_at)
                    detail = f": {reason}" if reason else (f" (FC rejected: {rejection})" if rejection else "")
                    raise RuntimeError(f"Auto-ARM thất bại{detail}. Kiểm tra prearm checks.")
                self.logs.add("flight", "info", "start_mission: ARM confirmed — continuing to AUTO")

            self._release_rc_internal()
            self._connection.mav.mission_set_current_send(
                self._target_system, self._target_component, 1
            )
            self._wait_until(
                lambda: self._state["mission"]["current"] == 1,
                4.0,
                "FC did not select mission item 1",
                context
            )
            self._set_mode_and_wait("AUTO", 5.0, context)
            return {
                "confirmed": True,
                "mode": self._state["mode"],
                "missionCurrent": self._state["mission"]["current"],
            }
        return self._execute(action, timeout=20.0)




    def upload_mission(self, raw_waypoints: list[dict[str, Any]]) -> Any:
        mission_items = self._normalise_mission(raw_waypoints)

        def action() -> dict[str, Any]:
            self._require_connection()
            self._write_waypoint_file(mission_items)
            return self._upload_items(mission_items)

        return self._execute(action, timeout=20.0)

    def _normalise_mission(
        self, raw_waypoints: list[dict[str, Any]]
    ) -> list[dict[str, Any]]:
        if not isinstance(raw_waypoints, list) or not raw_waypoints:
            raise ValueError("At least one mission item is required")
            
        max_items = 100
        operating_radius_m = 500.0
        max_alt = 150.0
        
        try:
            max_items = _site.mission_policy.max_items
            operating_radius_m = _site.mission_policy.operating_radius_m
            max_alt = _site.mission_policy.max_relative_altitude_m
        except Exception:
            pass

        if len(raw_waypoints) + 1 > max_items:
            raise ValueError(f"Mission exceeds {max_items} items (including HOME)")

        state = self.snapshot()
        home_lat = HOME_POSITION["lat"]
        home_lon = HOME_POSITION["lon"]
        items: list[dict[str, Any]] = [
            {
                "commandName": "HOME",
                "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                "lat": home_lat,
                "lon": home_lon,
                "alt": HOME_POSITION["alt"],
                "params": [0.0, 0.0, 0.0, 0.0],
            }
        ]
        for index, raw in enumerate(raw_waypoints, start=1):
            command_name = str(raw.get("command", "WAYPOINT")).upper()
            if command_name not in MISSION_COMMANDS:
                raise ValueError(f"Unsupported command at item {index}")
            command = MISSION_COMMANDS[command_name]
            positional = command_name not in {"TRANSITION_FW", "TRANSITION_VTOL"}
            lat = float(raw.get("lat", home_lat))
            lon = float(raw.get("lon", home_lon))
            alt = float(raw.get("alt", 50.0))
            if positional:
                if not -90 <= lat <= 90 or not -180 <= lon <= 180:
                    raise ValueError(f"Invalid coordinates at item {index}")
                if not 0 <= alt <= max_alt:
                    raise ValueError(f"Altitude must be 0-{max_alt} m at item {index}")
                dist = haversine_m(home_lat, home_lon, lat, lon)
                if dist > operating_radius_m:
                    raise ValueError(f"Waypoint {index} is outside geofence (dist: {dist:.1f} m > {operating_radius_m} m)")
            else:
                lat, lon, alt = 0.0, 0.0, 0.0
            hold = float(raw.get("hold", 0.0))
            acceptance = float(raw.get("acceptance", 20.0))
            params = [hold, acceptance, 0.0, 0.0]
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            if command_name == "VTOL_TAKEOFF":
                params = [0.0, 0.0, 0.0, 0.0]
            elif command_name == "VTOL_LAND":
                params = [0.0, 0.0, 0.0, 0.0]
            elif command_name == "LOITER_TIME":
                params[0] = max(0.0, hold)
            elif command_name == "TRANSITION_FW":
                frame = mavutil.mavlink.MAV_FRAME_MISSION
                params = [float(mavutil.mavlink.MAV_VTOL_STATE_FW), 0.0, 0.0, 0.0]
            elif command_name == "TRANSITION_VTOL":
                frame = mavutil.mavlink.MAV_FRAME_MISSION
                params = [
                    float(mavutil.mavlink.MAV_VTOL_STATE_MC),
                    0.0,
                    0.0,
                    0.0,
                ]
            items.append(
                {
                    "commandName": command_name,
                    "command": command,
                    "frame": frame,
                    "lat": lat,
                    "lon": lon,
                    "alt": alt,
                    "params": params,
                }
            )
        return items

    def _write_waypoint_file(self, items: list[dict[str, Any]]) -> None:
        self.mission_output.parent.mkdir(parents=True, exist_ok=True)
        lines = ["QGC WPL 110"]
        for seq, item in enumerate(items):
            current = 1 if seq == 0 else 0
            p1, p2, p3, p4 = item["params"]
            lines.append(
                "\t".join(
                    [
                        str(seq),
                        str(current),
                        str(item["frame"]),
                        str(item["command"]),
                        f"{p1:.6f}",
                        f"{p2:.6f}",
                        f"{p3:.6f}",
                        f"{p4:.6f}",
                        f"{item['lat']:.9f}",
                        f"{item['lon']:.9f}",
                        f"{item['alt']:.6f}",
                        "1",
                    ]
                )
            )
        self.mission_output.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def _send_mission_item(self, seq: int, item: dict[str, Any]) -> None:
        p1, p2, p3, p4 = item["params"]
        positional = item["frame"] in {
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT,
        }
        x = int(round(item["lat"] * 1e7)) if positional else int(item["lat"])
        y = int(round(item["lon"] * 1e7)) if positional else int(item["lon"])
        self._connection.mav.mission_item_int_send(
            self._target_system,
            self._target_component,
            seq,
            item["frame"],
            item["command"],
            1 if seq == 0 else 0,
            1,
            p1,
            p2,
            p3,
            p4,
            x,
            y,
            item["alt"],
        )

    def _upload_items(self, items: list[dict[str, Any]]) -> dict[str, Any]:
        self.logs.add(
            "mission", "info", f"Uploading {len(items)} mission items"
        )
        self._connection.mav.mission_count_send(
            self._target_system,
            self._target_component,
            len(items),
        )
        deadline = time.monotonic() + 12.0
        last_progress = time.monotonic()
        retries = 0
        sent: set[int] = set()
        while time.monotonic() < deadline:
            message = self._connection.recv_match(blocking=True, timeout=0.35)
            if message is None:
                if time.monotonic() - last_progress > 1.5:
                    retries += 1
                    if retries > 5:
                        break
                    self._connection.mav.mission_count_send(
                        self._target_system,
                        self._target_component,
                        len(items),
                    )
                    last_progress = time.monotonic()
                continue
            message_type = message.get_type()
            self._handle_message(message)
            if message_type in {"MISSION_REQUEST_INT", "MISSION_REQUEST"}:
                seq = int(message.seq)
                if not 0 <= seq < len(items):
                    raise RuntimeError(f"Vehicle requested invalid item {seq}")
                self._send_mission_item(seq, items[seq])
                sent.add(seq)
                last_progress = time.monotonic()
                retries = 0
            elif message_type == "MISSION_ACK":
                if message.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    raise RuntimeError(f"Mission rejected with result {message.type}")
                with self._lock:
                    self._state["mission"]["total"] = len(items)
                    self._state["mission"]["lastUpload"] = now_ms()
                    self._uploaded_mission_id += 1
                self.logs.add(
                    "mission",
                    "info",
                    f"Mission accepted ({len(sent)}/{len(items)} items requested)",
                )
                return {"uploaded": len(items), "accepted": True}
        raise TimeoutError("Mission upload did not receive MISSION_ACK")

    def _loop(self) -> None:
        while self._running:
            try:
                for _ in range(10):
                    try:
                        task = self._tasks.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        self._run_task(task)
                    except Exception as error:
                        task.error = error
                        self.logs.add("mavlink", "error", str(error))
                        task.completed.set()

                # Keep the fixed-wing interlock based on continuous telemetry,
                # not just the instant the pilot presses the transition switch.
                self._hover_stability_age(self.snapshot())
                self._service_external_rc()

                if (
                    self._rc_override_active
                    and self._last_heartbeat
                    and time.monotonic() - self._last_heartbeat < 4.0
                    and time.monotonic() - self._last_rc_send >= 0.25
                ):
                    # ArduPilot expires RC overrides unless they are refreshed.
                    # MAVProxy does this automatically; the dashboard must too.
                    self._send_rc()

                message = self._connection.recv_match(blocking=False)
                if message is not None:
                    self._handle_message(message)
                else:
                    time.sleep(0.01)
            except Exception as error:  # pragma: no cover - transport resilience
                self.logs.add("mavlink", "error", f"Receive loop: {error}")
                time.sleep(0.2)

    def _handle_message(self, message: Any) -> None:
        message_type = message.get_type()
        if message_type == "BAD_DATA":
            return
        with self._lock:
            if message_type == "HEARTBEAT":
                heartbeat_was_stale = (
                    not self._last_heartbeat
                    or time.monotonic() - self._last_heartbeat >= 4.0
                )
                self._target_system = message.get_srcSystem()
                self._target_component = message.get_srcComponent()
                self._connection.target_system = self._target_system
                self._connection.target_component = self._target_component
                self._last_heartbeat = time.monotonic()
                if heartbeat_was_stale:
                    # A new SITL session must never inherit an old throttle.
                    self._rc = [
                        1500,
                        1500,
                        1000,
                        1500,
                        65535,
                        65535,
                        65535,
                        65535,
                    ]
                    self._rc_owner = None
                    self._rc_generation += 1
                    self._rc_override_active = False
                    self._last_rc_send = 0.0
                    self._state["homeSet"] = False
                    self._connection.mav.request_data_stream_send(
                        self._target_system,
                        self._target_component,
                        mavutil.mavlink.MAV_DATA_STREAM_ALL,
                        4,
                        1
                    )
                self._state["connected"] = True
                self._state["armed"] = bool(
                    message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                self._state["mode"] = mavutil.mode_string_v10(message)
                self._state["systemStatus"] = int(message.system_status)
            elif message_type == "ATTITUDE":
                self._state["attitude"] = {
                    "roll": math.degrees(message.roll),
                    "pitch": math.degrees(message.pitch),
                    "yaw": math.degrees(message.yaw) % 360,
                    "rollSpeed": math.degrees(message.rollspeed),
                    "pitchSpeed": math.degrees(message.pitchspeed),
                    "yawSpeed": math.degrees(message.yawspeed),
                }
            elif message_type == "GLOBAL_POSITION_INT":
                self._state["gps"]["lat"] = message.lat / 1e7
                self._state["gps"]["lon"] = message.lon / 1e7
                self._state["position"] = {
                    "altMsl": message.alt / 1000.0,
                    "relativeAlt": message.relative_alt / 1000.0,
                    "vx": message.vx / 100.0,
                    "vy": message.vy / 100.0,
                    "vz": message.vz / 100.0,
                    "heading": (
                        message.hdg / 100.0 if message.hdg != 65535 else None
                    ),
                }
            elif message_type == "GPS_RAW_INT":
                self._state["gps"].update(
                    {
                        "lat": message.lat / 1e7,
                        "lon": message.lon / 1e7,
                        "fixType": int(message.fix_type),
                        "satellites": int(message.satellites_visible),
                        "hdop": (
                            message.eph / 100.0 if message.eph != 65535 else None
                        ),
                    }
                )
            elif message_type == "HOME_POSITION":
                self._state["homeSet"] = True
                self._state["home"] = {
                    "lat": message.latitude / 1e7,
                    "lon": message.longitude / 1e7,
                    "altMsl": message.altitude / 1000.0,
                }
            elif message_type == "VFR_HUD":
                self._state["hud"] = {
                    "airspeed": float(message.airspeed),
                    "groundspeed": float(message.groundspeed),
                    "throttle": int(message.throttle),
                    "climb": float(message.climb),
                }
            elif message_type == "NAV_CONTROLLER_OUTPUT":
                self._state["nav"] = {
                    "valid": True,
                    "updatedAt": now_ms(),
                    "navBearing": float(message.nav_bearing),
                    "targetBearing": float(message.target_bearing),
                    "wpDist": float(message.wp_dist)
                }
            elif message_type == "SYS_STATUS":
                self._state["battery"] = {
                    "voltage": (
                        message.voltage_battery / 1000.0
                        if message.voltage_battery not in {0, 65535}
                        else None
                    ),
                    "current": (
                        message.current_battery / 100.0
                        if message.current_battery != -1
                        else None
                    ),
                    "remaining": (
                        int(message.battery_remaining)
                        if message.battery_remaining != -1
                        else None
                    ),
                }
            elif message_type == "RC_CHANNELS":
                self._state["rc"] = {
                    "roll": int(message.chan1_raw),
                    "pitch": int(message.chan2_raw),
                    "throttle": int(message.chan3_raw),
                    "yaw": int(message.chan4_raw),
                }
            elif message_type == "MISSION_CURRENT":
                self._state["mission"].update(
                    {
                        "current": int(message.seq),
                        "total": int(getattr(message, "total", 0)),
                        "state": int(getattr(message, "mission_state", 0)),
                    }
                )
            elif message_type == "MISSION_ITEM_REACHED":
                self._state["mission"]["lastReached"] = int(message.seq)
            elif message_type == "COMMAND_ACK":
                acknowledgement = {
                    "command": int(message.command),
                    "result": int(message.result),
                    "timestamp": now_ms(),
                }
                self._state["lastCommandAck"] = acknowledgement
                self.logs.add(
                    "mavlink",
                    "info",
                    f"COMMAND_ACK command={message.command} result={message.result}",
                )
            elif message_type == "STATUSTEXT":
                text = message.text
                if isinstance(text, bytes):
                    text = text.decode("utf-8", errors="replace")
                clean_text = str(text).rstrip("\x00")
                if "origin set" in clean_text.lower():
                    self._state["homeSet"] = True
                status = {
                    "timestamp": now_ms(),
                    "severity": int(message.severity),
                    "text": clean_text,
                }
                self._status_messages.append(status)
                level = "error" if message.severity <= 3 else "info"
                self.logs.add("vehicle", level, status["text"])


class ExperimentManager:
    def __init__(self, repo_dir: Path):
        from Simulation.experiments.run_store import RunStore
        self.store = RunStore(repo_dir / "Simulation/runs")

    def create_run(self, request_data: dict[str, Any]) -> dict[str, Any]:
        raise RuntimeError(
            "Experiment campaigns are locked until the P4 aerodynamic model "
            "and transition controller have been identified"
        )

        # Retained as scaffolding for the later calibrated experiment runner.
        from Simulation.experiments.schemas import (
            ExperimentRequest, ExperimentTimeouts, SafetyLimits, RunManifest, RunPhase
        )
        from datetime import datetime, timezone
        
        timeouts = ExperimentTimeouts(**request_data.get("timeouts", {
            "stack_ready_s": 30.0, "arm_s": 10.0, "takeoff_s": 20.0, 
            "mission_s": 600.0, "landing_s": 30.0
        }))
        safety = SafetyLimits(**request_data.get("safety", {
            "max_altitude_m": 120.0, "max_distance_home_m": 500.0, 
            "max_roll_deg": 45.0, "max_pitch_deg": 30.0
        }))
        req = ExperimentRequest(
            schema_version=request_data.get("schema_version", 1),
            case=request_data.get("case", "baseline"),
            mission=request_data.get("mission", "gui_mission"),
            profile=request_data.get("profile", "default"),
            repetitions=request_data.get("repetitions", 1),
            seed_base=request_data.get("seed_base", 42),
            timeouts=timeouts,
            safety=safety
        )
        run = self.store.begin(req, 1) # hardcode repetition 1 for now
        # Mocking the finalize
        manifest = RunManifest(
            schema_version=1,
            run_id=run.run_id,
            case_name=req.case,
            repetition=1,
            seed=req.seed_base,
            hashes={"mission": "dummy_hash"},
            versions={"simulation": "0.1.0"},
            started_at_utc=datetime.now(timezone.utc).isoformat()
        )
        
        # Save a mock result.json with run phase as COMPLETED
        import json
        with open(run.path / "result.json", "w", encoding="utf-8") as f:
            json.dump({"phase": RunPhase.COMPLETED.value, "metrics": {}}, f)

        final_path = self.store.finalize(run, manifest)
        return {"run_id": run.run_id, "status": "completed"}

    def list_runs(self) -> list[dict[str, Any]]:
        runs = []
        import json
        for path in self.store.root.iterdir():
            if path.is_dir() and not path.name.endswith(".partial"):
                manifest_path = path / "manifest.json"
                if manifest_path.exists():
                    try:
                        with open(manifest_path, "r", encoding="utf-8") as f:
                            data = json.load(f)
                            runs.append(data)
                    except Exception:
                        pass
        return runs

    def get_run(self, run_id: str) -> dict[str, Any]:
        path = self.store.root / run_id
        if not path.is_dir() or path.name.endswith(".partial"):
            raise ValueError(f"Run {run_id} not found")
        import json
        with open(path / "manifest.json", "r", encoding="utf-8") as f:
            manifest = json.load(f)
        result_path = path / "result.json"
        result = {}
        if result_path.exists():
            with open(result_path, "r", encoding="utf-8") as f:
                result = json.load(f)
        return {"manifest": manifest, "result": result}

    def abort_run(self, run_id: str) -> dict[str, Any]:
        return {"status": "aborted"}

    def get_artifacts(self, run_id: str) -> list[str]:
        path = self.store.root / run_id
        if not path.is_dir() or path.name.endswith(".partial"):
            raise ValueError(f"Run {run_id} not found")
        return [p.name for p in path.iterdir() if p.is_file()]

    def compare_runs(self, case_names: list[str]) -> list[dict[str, Any]]:
        runs = self.list_runs()
        if case_names:
            runs = [r for r in runs if r.get("case_name") in case_names]
        return runs


class ControlStation:
    def __init__(self, repo: Path) -> None:
        self.repo = repo
        self.static_dir = repo / "Simulation/control_station/static"
        
        bind_host = os.environ.get("P4_TRANSWING_BIND_HOST", "127.0.0.1")
        lan_secret = os.environ.get("P4_TRANSWING_LAN_SECRET")
        self.security = RequestSecurity(bind_host, lan_secret)
        self.logs = LogBroker()
        self.processes = ProcessManager(repo, self.logs)
        self.experiments = ExperimentManager(repo)
        self.mavlink = MavlinkManager(
            self.logs,
            repo / "Simulation/ardupilot/missions/gui_mission.waypoints",
        )
        self.started_at = now_ms()
        # P4's runner gives sim_vehicle.py a dedicated --use-dir, keeping its
        # EEPROM and DataFlash logs isolated from MiniHawk and other vehicles.
        # Override with P4_TRANSWING_FLIGHT_LOGS_DIR if required.
        self.flight_log_dir = Path(
            os.environ.get(
                "P4_TRANSWING_FLIGHT_LOGS_DIR",
                str(repo / "Simulation/ardupilot/sitl_state/logs"),
            )
        )

    def flight_logs(self) -> list[dict[str, Any]]:
        """List recent ArduPilot dataflash logs, newest first (max 50)."""
        if not self.flight_log_dir.is_dir():
            return []
        entries = []
        for path in self.flight_log_dir.glob("*.BIN"):
            if not FLIGHT_LOG_NAME.fullmatch(path.name):
                continue
            try:
                stat = path.stat()
            except OSError:
                continue
            entries.append(
                {
                    "name": path.name,
                    "sizeBytes": stat.st_size,
                    "modifiedAt": int(stat.st_mtime * 1000),
                }
            )
        entries.sort(key=lambda item: item["modifiedAt"], reverse=True)
        return entries[:50]

    def flight_log_path(self, name: str) -> Path | None:
        """Resolve a flight-log filename to a safe path, or None if invalid."""
        if not FLIGHT_LOG_NAME.fullmatch(name):
            return None
        log_dir = self.flight_log_dir.resolve()
        candidate = (self.flight_log_dir / name).resolve()
        try:
            candidate.relative_to(log_dir)
        except ValueError:
            return None
        return candidate if candidate.is_file() else None

    def config(self) -> dict[str, Any]:
        cases = []
        test_case_dir = self.repo / "Simulation/gazebo/test_cases"
        for path in sorted(test_case_dir.glob("*.json")):
            if path.name == "live_weather.json":
                continue
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
                cases.append(
                    {
                        "name": data["name"],
                        "description": data.get("description", ""),
                        "payloadKg": data.get("payload", {}).get("mass_kg", 0),
                        "windEnu": data.get("environment", {}).get(
                            "wind_enu_mps", [0, 0, 0]
                        ),
                    }
                )
            except Exception as error:
                self.logs.add("control-station", "error", f"{path.name}: {error}")
        mission_planner = self.processes._mission_planner_path()
        return {
            "version": VERSION,
            "home": HOME_POSITION,
            "cases": cases,
            "modes": sorted(ALLOWED_MODES),
            "missionPlannerFound": bool(
                mission_planner is not None and mission_planner.is_file()
            ),
            "missionPlannerPath": (
                str(mission_planner) if mission_planner else "not configured"
            ),
            "mavlinkEndpoint": MAVLINK_ENDPOINT,
            "csrfToken": self.security.csrf_token,
            "missionPolicy": {
                "operatingRadiusM": _site.mission_policy.operating_radius_m if hasattr(_site, "mission_policy") else 500.0,
                "maxRelativeAltitudeM": _site.mission_policy.max_relative_altitude_m if hasattr(_site, "mission_policy") else 150.0,
                "maxItems": _site.mission_policy.max_items if hasattr(_site, "mission_policy") else 100
            }
        }

    def state(self) -> dict[str, Any]:
        return {
            "version": VERSION,
            "uptimeSeconds": (now_ms() - self.started_at) / 1000.0,
            "simulation": self.processes.snapshot(),
            "vehicle": self.mavlink.snapshot(),
        }


class ApiHandler(BaseHTTPRequestHandler):
    server_version = "P4TranswingControlStation/0.1"
    # Keep the MKR bridge's local HTTP connection alive.  This avoids a TCP
    # handshake for every 100 Hz RC frame; all normal responses include an
    # explicit Content-Length, so HTTP/1.1 framing remains unambiguous.
    protocol_version = "HTTP/1.1"

    @property
    def app(self) -> ControlStation:
        return self.server.app  # type: ignore[attr-defined]

    def log_message(self, format_string: str, *args: Any) -> None:
        self.app.logs.add("http", "info", format_string % args)

    def _json(self, status: int, payload: Any) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    # --- CSRF / cross-origin guard -------------------------------------
    _LOCAL_HOSTS = {"127.0.0.1", "localhost"}

    def _reject_untrusted_post(self) -> None:
        self.app.security.validate_mutation(self)

    @staticmethod
    def _int_param(query: dict[str, list[str]], key: str, default: int) -> int:
        raw = query.get(key, [str(default)])[0]
        try:
            return int(raw)
        except (TypeError, ValueError) as error:
            raise ValueError(f"Query parameter '{key}' must be an integer") from error

    def _read_json(self) -> dict[str, Any]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as error:
            raise ValueError("Invalid Content-Length") from error
        if length > 1_000_000:
            raise ValueError("Request body is too large")
        if length == 0:
            return {}
        data = json.loads(self.rfile.read(length).decode("utf-8"))
        if not isinstance(data, dict):
            raise ValueError("JSON body must be an object")
        return data

    def do_GET(self) -> None:  # noqa: N802
        parsed = urllib.parse.urlparse(self.path)
        try:
            if parsed.path == "/api/health":
                self._json(HTTPStatus.OK, {"ok": True, "version": VERSION})
                return
            if parsed.path == "/api/config":
                self._json(HTTPStatus.OK, self.app.config())
                return
            if parsed.path == "/api/state":
                self._json(HTTPStatus.OK, self.app.state())
                return
            if parsed.path == "/api/logs":
                query = urllib.parse.parse_qs(parsed.query)
                after = self._int_param(query, "after", 0)
                limit = self._int_param(query, "limit", 300)
                self._json(
                    HTTPStatus.OK,
                    {"entries": self.app.logs.read(after=after, limit=limit)},
                )
                return
            if parsed.path == "/api/logs/flight":
                self._json(HTTPStatus.OK, {"entries": self.app.flight_logs()})
                return
            if parsed.path.startswith("/api/logs/flight/"):
                name = urllib.parse.unquote(parsed.path[len("/api/logs/flight/"):])
                self._serve_flight_log(name)
                return
            if parsed.path == "/api/v1/experiments":
                self._json(HTTPStatus.OK, {"ok": True, "result": self.app.experiments.list_runs()})
                return
            if parsed.path == "/api/v1/experiments/compare":
                query = urllib.parse.parse_qs(parsed.query)
                cases = query.get("case", [])
                self._json(HTTPStatus.OK, {"ok": True, "result": self.app.experiments.compare_runs(cases)})
                return
            if parsed.path.startswith("/api/v1/experiments/"):
                suffix = parsed.path[len("/api/v1/experiments/"):]
                if "/" not in suffix:
                    self._json(HTTPStatus.OK, {"ok": True, "result": self.app.experiments.get_run(suffix)})
                    return
                run_id, sub = suffix.split("/", 1)
                if sub == "artifacts":
                    self._json(HTTPStatus.OK, {"ok": True, "result": self.app.experiments.get_artifacts(run_id)})
                    return
            if parsed.path.startswith("/api/"):
                self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "Endpoint not found"})
                return
            self._serve_static(parsed.path)
        except ValueError as error:
            self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(error)})
        except Exception as error:  # pragma: no cover - defensive API boundary
            self.app.logs.add("http", "error", traceback.format_exc())
            self._json(
                HTTPStatus.INTERNAL_SERVER_ERROR, {"ok": False, "error": str(error)}
            )

    def do_POST(self) -> None:  # noqa: N802
        parsed = urllib.parse.urlparse(self.path)
        try:
            self._reject_untrusted_post()          # CSRF guard (FIX-03)
            body = self._read_json()
            result = self._dispatch_post(parsed.path, body)
            self._json(HTTPStatus.OK, {"ok": True, "result": result})
        except PermissionError as error:           # cross-origin / wrong content-type
            self.app.logs.add("http", "error", f"Blocked POST {parsed.path}: {error}")
            self._json(HTTPStatus.FORBIDDEN, {"ok": False, "error": str(error)})
        except CommandQueueFull as error:
            self._json(HTTPStatus.TOO_MANY_REQUESTS, {"ok": False, "error": str(error)})
        except CommandTimeout as error:
            self._json(HTTPStatus.GATEWAY_TIMEOUT, {"ok": False, "error": str(error)})
        except (ValueError, RuntimeError, TimeoutError) as error:
            self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(error)})
        except Exception as error:  # pragma: no cover - defensive API boundary
            self.app.logs.add("http", "error", traceback.format_exc())
            self._json(
                HTTPStatus.INTERNAL_SERVER_ERROR,
                {"ok": False, "error": str(error)},
            )

    def _dispatch_post(self, path: str, body: dict[str, Any]) -> Any:
        if path == "/api/simulation/start":
            self.app.processes.start(body)
            return {"status": "starting"}
        if path == "/api/simulation/stop":
            self.app.processes.stop()
            return {"status": "stopped"}
        if path == "/api/vehicle/arm":
            return self.app.mavlink.arm(True)
        if path == "/api/vehicle/disarm":
            return self.app.mavlink.arm(False)
        if path == "/api/vehicle/mode":
            return self.app.mavlink.set_mode(str(body.get("mode", "")))
        if path == "/api/vehicle/rc":
            return self.app.mavlink.set_rc(
                int(body.get("channel", 0)), int(body.get("value", 0))
            )
        if path == "/api/vehicle/center":
            return self.app.mavlink.center_controls(
                int(body.get("throttle", 1500))
            )
        if path == "/api/vehicle/release-rc":
            return self.app.mavlink.release_rc()
        if path == "/api/controller/external/frame":
            line = body.get("line")
            if not isinstance(line, str):
                raise ValueError("External RC request requires string field 'line'")
            return self.app.mavlink.ingest_external_rc_line(line)
        if path == "/api/controller/external/enable":
            return self.app.mavlink.set_external_rc_enabled(True)
        if path == "/api/controller/external/disable":
            return self.app.mavlink.set_external_rc_enabled(False)
        if path == "/api/vehicle/takeoff":
            return self.app.mavlink.takeoff(float(body.get("altitude", 20.0)))
        if path == "/api/vehicle/climb":
            return self.app.mavlink.climb_to_altitude(float(body.get("altitude", 50.0)))
        if path == "/api/vehicle/transition-fw":
            return self.app.mavlink.transition_to_fixed_wing(
                int(body.get("throttle", TRANSITION_RC3)),
                float(body.get("minimumAltitude", 25.0)),
            )
        if path == "/api/vehicle/qland":
            return self.app.mavlink.qland()
        if path == "/api/vehicle/abort":
            return self.app.mavlink.abort_to_qloiter()
        if path == "/api/mission/upload":
            waypoints = body.get("waypoints", [])
            if not isinstance(waypoints, list):
                raise ValueError("waypoints must be an array")
            return self.app.mavlink.upload_mission(waypoints)
        if path == "/api/mission/start":
            expected = body.get("expected_mission_id")
            if expected is not None:
                expected = int(expected)
            return self.app.mavlink.start_mission(expected)
        if path == "/api/mission/clear":
            return self.app.mavlink.clear_mission()
        if path == "/api/v1/experiments":
            return self.app.experiments.create_run(body)
        if path.startswith("/api/v1/experiments/"):
            suffix = path[len("/api/v1/experiments/"):]
            if suffix.endswith("/abort"):
                run_id = suffix[:-6]
                return self.app.experiments.abort_run(run_id)
        raise ValueError(f"Unknown API endpoint: {path}")

    def _serve_flight_log(self, name: str) -> None:
        path = self.app.flight_log_path(name)
        if path is None:
            self.send_error(HTTPStatus.NOT_FOUND, "Flight log not found")
            return
        size = path.stat().st_size
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(size))
        self.send_header(
            "Content-Disposition", f'attachment; filename="{path.name}"'
        )
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        try:
            with path.open("rb") as stream:
                while True:
                    chunk = stream.read(1 << 20)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
        except (BrokenPipeError, ConnectionResetError):
            # Client cancelled the download partway through; nothing left to do.
            pass

    def _serve_static(self, request_path: str) -> None:
        relative = request_path.lstrip("/") or "index.html"
        candidate = (self.app.static_dir / relative).resolve()
        try:
            candidate.relative_to(self.app.static_dir.resolve())
        except ValueError:
            self.send_error(HTTPStatus.FORBIDDEN)
            return
        if not candidate.is_file():
            candidate = self.app.static_dir / "index.html"
        data = candidate.read_bytes()
        content_type = mimetypes.guess_type(candidate.name)[0] or "application/octet-stream"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(data)


class AppServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self, address: tuple[str, int], handler: type[BaseHTTPRequestHandler], app: ControlStation
    ) -> None:
        super().__init__(address, handler)
        self.app = app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="P4 Transwing local control station")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--repo",
        type=Path,
        default=Path(
            os.environ.get(
                "P4_TRANSWING_REPO",
                str(Path(__file__).resolve().parents[2]),
            )
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo = args.repo.resolve()
    if not (repo / "Simulation/gazebo").is_dir():
        raise SystemExit(f"P4 Transwing repository not found: {repo}")
    app = ControlStation(repo)
    server = AppServer((args.host, args.port), ApiHandler, app)
    app.logs.add(
        "control-station",
        "info",
        f"Dashboard ready at http://{args.host}:{args.port}",
    )
    print(f"P4 Transwing Control Station {VERSION}")
    print(f"Open http://{args.host}:{args.port}")
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        app.processes.stop()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
