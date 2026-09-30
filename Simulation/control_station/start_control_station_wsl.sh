#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
P4_TRANSWING_REPO="${P4_TRANSWING_REPO:-$(cd "$SCRIPT_DIR/../.." && pwd -P)}"
SERVER="$P4_TRANSWING_REPO/Simulation/control_station/server.py"

if [[ ! -f "$SERVER" ]]; then
  echo "Control-station server was not found: $SERVER" >&2
  exit 1
fi

if ! python3 -c "from pymavlink import mavutil" >/dev/null 2>&1; then
  echo "Python package pymavlink is missing in Ubuntu 22.04." >&2
  echo "Run the ArduPilot prerequisite installer, then try again." >&2
  exit 1
fi

# The Windows launcher normally forwards this path through WSLENV. Also
# discover the standard native Windows installation here so the supported
# direct-WSL launch path can open Mission Planner without extra setup.
if [[ -z "${P4_TRANSWING_MISSION_PLANNER:-}" ]] && command -v wslpath >/dev/null 2>&1; then
  for windows_candidate in \
    'C:\Program Files (x86)\Mission Planner\MissionPlanner.exe' \
    'C:\Program Files\Mission Planner\MissionPlanner.exe'; do
    mission_planner_candidate="$(wslpath -a -u "$windows_candidate" 2>/dev/null || true)"
    if [[ -n "$mission_planner_candidate" && -f "$mission_planner_candidate" ]]; then
      export P4_TRANSWING_MISSION_PLANNER="$mission_planner_candidate"
      echo "[p4-control-station] Found Mission Planner: $mission_planner_candidate"
      break
    fi
  done
fi

export P4_TRANSWING_REPO
export GZ_PARTITION="${P4_TRANSWING_GZ_PARTITION:-${GZ_PARTITION:-p4_transwing}}"
export PYTHONUNBUFFERED=1

# WSL2 forwards services bound to localhost to Windows. Keep the control API
# local by default; an explicit non-loopback override is handled by the
# server's LAN authentication policy.
BIND_HOST="${P4_TRANSWING_BIND_HOST:-127.0.0.1}"
export P4_TRANSWING_BIND_HOST="$BIND_HOST"

# server.py imports modules through the top-level ``Simulation`` package.
# Run it from the repository root as a module so that Python resolves that
# package correctly. Executing the file path directly only places
# Simulation/control_station on sys.path and breaks those imports.
cd "$P4_TRANSWING_REPO"
exec python3 -m Simulation.control_station.server \
  --repo "$P4_TRANSWING_REPO" \
  --host "$BIND_HOST" \
  --port 8765
