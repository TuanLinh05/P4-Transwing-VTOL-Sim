#!/usr/bin/env bash
set -uo pipefail

echo "P4 remote WSL diagnostics"
printf 'DISPLAY=%s\n' "${DISPLAY:-<empty>}"
printf 'WAYLAND_DISPLAY=%s\n' "${WAYLAND_DISPLAY:-<empty>}"
printf 'WSL_INTEROP=%s\n' "${WSL_INTEROP:-<empty>}"
printf 'Gazebo=%s\n' "$(gz sim --versions 2>/dev/null | tr '\n' ' ' || true)"

echo
echo "WSLg renderer:"
if [[ -e /dev/dxg ]]; then
  echo "/dev/dxg is present (WSL GPU forwarding is available)."
else
  echo "/dev/dxg is absent (WSL GPU forwarding is not exposed to this session)."
fi
if command -v glxinfo >/dev/null 2>&1; then
  glxinfo -B 2>&1 | grep -E 'OpenGL vendor|OpenGL renderer|OpenGL version' || true
else
  echo "glxinfo is unavailable; install mesa-utils to identify the active renderer."
fi

echo
echo "Managed processes:"
ps -eo pid,ppid,cmd | grep -E 'gz sim|arduplane|sim_vehicle|MAVProxy|MissionPlanner' | grep -v grep || true

echo
echo "Dashboard state:"
if command -v curl >/dev/null 2>&1; then
  curl -fsS http://127.0.0.1:8765/api/state 2>&1 || true
else
  echo "curl is unavailable"
fi

echo
echo "Recent dashboard logs:"
if command -v curl >/dev/null 2>&1 && command -v python3 >/dev/null 2>&1; then
  curl -fsS 'http://127.0.0.1:8765/api/logs?after=0&limit=700' 2>/dev/null |
    python3 -c '
import json
import sys

try:
    entries = json.load(sys.stdin).get("entries", [])
except (json.JSONDecodeError, BrokenPipeError) as error:
    print(f"Could not decode dashboard logs: {error}")
    raise SystemExit(0)

interesting = {
    "control-station", "gazebo-server", "gazebo-gui", "ardupilot-sitl",
    "mission-planner", "preflight"
}
for entry in entries:
    if entry.get("source") in interesting:
        print(f"[{entry.get('source')}] {entry.get('level')}: {entry.get('message')}")
' || true
elif command -v curl >/dev/null 2>&1; then
  curl -fsS 'http://127.0.0.1:8765/api/logs?after=0&limit=120' 2>&1 || true
fi
