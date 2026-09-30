#!/usr/bin/env python3
"""Forward RC-only P4RC frames from an MKR USB COM port to Control Station.

Run this on Windows, not inside WSL:

    py -3 Simulation\tools\bridge_mkr_usb.py --port COM5

The MKR emits ten FlySky iBUS channels as P4RC over USB.  This bridge forwards
only the newest valid line to the local Control Station.  It never writes to
the MKR and has no MAVLink or telemetry-return path.
"""

from __future__ import annotations

import argparse
import http.client
import json
import sys
import threading
import time
import urllib.parse


class ControlStationHttpError(RuntimeError):
    def __init__(self, code: int, body: str) -> None:
        self.code = code
        self.body = body
        super().__init__(f"HTTP {code}")


class ControlStationClient:
    """Small persistent HTTP client for the high-rate RC stream."""

    def __init__(self, base_url: str) -> None:
        parsed = urllib.parse.urlsplit(base_url)
        if parsed.scheme not in {"http", "https"} or not parsed.hostname:
            raise ValueError(f"Unsupported Control Station URL: {base_url}")
        self._parsed = parsed
        self._origin = f"{parsed.scheme}://{parsed.netloc}"
        self._prefix = parsed.path.rstrip("/")
        self._connection: http.client.HTTPConnection | None = None

    def _connect(self) -> None:
        connection_type = (
            http.client.HTTPSConnection
            if self._parsed.scheme == "https"
            else http.client.HTTPConnection
        )
        self._connection = connection_type(
            self._parsed.hostname, self._parsed.port, timeout=0.75
        )

    def close(self) -> None:
        if self._connection is not None:
            try:
                self._connection.close()
            finally:
                self._connection = None

    def _request(
        self,
        method: str,
        path: str,
        *,
        body: bytes | None = None,
        headers: dict[str, str] | None = None,
    ) -> str:
        if self._connection is None:
            self._connect()
        assert self._connection is not None
        try:
            self._connection.request(method, path, body=body, headers=headers or {})
            response = self._connection.getresponse()
            text = response.read().decode("utf-8", errors="replace")
        except Exception:
            self.close()
            raise
        if response.status >= 400:
            self.close()
            raise ControlStationHttpError(response.status, text)
        return text

    def get_config(self) -> dict[str, object]:
        payload = json.loads(self._request("GET", f"{self._prefix}/api/config"))
        if not isinstance(payload, dict):
            raise RuntimeError("Control Station returned an invalid config payload")
        return payload

    def post_frame(self, csrf_token: str, line: str) -> None:
        body = json.dumps({"line": line}, separators=(",", ":")).encode("utf-8")
        payload_text = self._request(
            "POST",
            f"{self._prefix}/api/controller/external/frame",
            body=body,
            headers={
                "Content-Type": "application/json",
                "Origin": self._origin,
                "X-P4-Transwing-CSRF": csrf_token,
            },
        )
        payload = json.loads(payload_text)
        if not payload.get("ok", False):
            raise RuntimeError(str(payload.get("error", "Control Station rejected frame")))


def rc_forward_worker(
    *,
    base_url: str,
    latest: dict[str, object],
    latest_lock: threading.Lock,
    updated: threading.Event,
    stop_event: threading.Event,
) -> None:
    """Post only the newest frame without blocking the USB reader.

    Sequence gaps are intentional: flight control needs current stick/switch
    state, not historical USB frames.  This prevents slow HTTP from causing
    the MKR's iBUS serial decoder to miss a frame.
    """
    client = ControlStationClient(base_url)
    csrf_token = ""
    last_generation = -1
    forwarded = 0
    last_rate_report = time.monotonic()
    last_error_report = 0.0
    last_error_key = ""
    try:
        while not stop_event.is_set():
            updated.wait(0.025)
            updated.clear()
            with latest_lock:
                generation = int(latest.get("generation", 0))
                line = latest.get("line")
            if not isinstance(line, str) or generation == last_generation:
                continue
            try:
                if not csrf_token:
                    config = client.get_config()
                    csrf_token = str(config["csrfToken"])
                client.post_frame(csrf_token, line)
                last_generation = generation
                forwarded += 1
            except ControlStationHttpError as error:
                if error.code == 403:
                    csrf_token = ""
                    continue
                detail = error.body.strip() or "no response body"
                try:
                    detail = str(json.loads(detail).get("error", detail))
                except (TypeError, ValueError, AttributeError):
                    pass
                key = f"HTTP {error.code} ({detail})"
                now = time.monotonic()
                if key != last_error_key or now - last_error_report >= 2.0:
                    print(f"Control Station rejected frame: {key}", file=sys.stderr)
                    last_error_key = key
                    last_error_report = now
            except Exception as error:
                key = str(error)
                now = time.monotonic()
                if key != last_error_key or now - last_error_report >= 2.0:
                    print(f"P4RC forwarding error: {key}", file=sys.stderr)
                    last_error_key = key
                    last_error_report = now

            now = time.monotonic()
            if now - last_rate_report >= 2.0:
                print(f"P4RC forward rate: {forwarded / (now - last_rate_report):.1f} Hz")
                forwarded = 0
                last_rate_report = now
    finally:
        client.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MKR P4RC USB-to-Control-Station bridge")
    parser.add_argument("--port", required=True, help="Windows COM port, e.g. COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--url", default="http://127.0.0.1:8765")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    base_url = args.url.rstrip("/")
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError:
        print("Missing pyserial. Install with: py -3 -m pip install pyserial", file=sys.stderr)
        return 2

    try:
        probe_client = ControlStationClient(base_url)
        probe_client.get_config()
        probe_client.close()
    except Exception as error:
        print(f"Cannot reach Control Station at {base_url}: {error}", file=sys.stderr)
        return 3

    try:
        port = serial.Serial(args.port, args.baud, timeout=0.025)
    except Exception as error:
        print(f"Cannot open {args.port}: {error}", file=sys.stderr)
        return 4

    rc_stop = threading.Event()
    rc_latest: dict[str, object] = {"generation": 0, "line": None}
    rc_latest_lock = threading.Lock()
    rc_updated = threading.Event()
    rc_thread = threading.Thread(
        target=rc_forward_worker,
        kwargs={
            "base_url": base_url,
            "latest": rc_latest,
            "latest_lock": rc_latest_lock,
            "updated": rc_updated,
            "stop_event": rc_stop,
        },
        name="p4rc-forward",
        daemon=True,
    )
    rc_thread.start()
    ingested = 0
    last_ingest_report = time.monotonic()
    print(f"Forwarding MKR P4RC from {args.port} to {base_url}. Press Ctrl+C to stop.")
    try:
        while True:
            raw = port.readline()
            if raw:
                try:
                    line = raw.decode("ascii", errors="strict").strip()
                except UnicodeDecodeError:
                    continue
                if line.startswith("P4RC,"):
                    with rc_latest_lock:
                        rc_latest["line"] = line
                        rc_latest["generation"] = int(rc_latest["generation"]) + 1
                    rc_updated.set()
                    ingested += 1

            now = time.monotonic()
            if now - last_ingest_report >= 2.0:
                print(f"P4RC USB ingest rate: {ingested / (now - last_ingest_report):.1f} Hz")
                ingested = 0
                last_ingest_report = now
    except KeyboardInterrupt:
        print("USB bridge stopped.")
        return 0
    finally:
        rc_stop.set()
        rc_updated.set()
        rc_thread.join(timeout=1.0)
        port.close()


if __name__ == "__main__":
    raise SystemExit(main())
