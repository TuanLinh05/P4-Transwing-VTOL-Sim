#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
P4_REPO="${P4_TRANSWING_REPO:-$(cd "$SCRIPT_DIR/../.." && pwd -P)}"
PARAM_FILE="$P4_REPO/Simulation/ardupilot/p4_transwing_sitl.param"
ARDUPILOT_HOME="${P4_ARDUPILOT_HOME:-$HOME/ardupilot}"
SIM_VEHICLE="$ARDUPILOT_HOME/Tools/autotest/sim_vehicle.py"
SITL_HOME="${P4_TRANSWING_SITL_HOME:-10.772100,106.657900,10,0}"
SITL_STATE_DIR="${P4_TRANSWING_SITL_STATE_DIR:-$P4_REPO/Simulation/ardupilot/sitl_state}"
PARAM_RUNTIME="/tmp/p4_transwing_sitl_${UID}.param"

wipe_args=()
if [[ "${1:-}" == "--wipe" ]]; then
  wipe_args=(-w)
elif [[ $# -ne 0 ]]; then
  echo "Usage: run_p4_sitl_wsl.sh [--wipe]" >&2
  exit 2
fi

if [[ ! -x "$SIM_VEHICLE" ]]; then
  echo "sim_vehicle.py was not found at: $SIM_VEHICLE" >&2
  exit 1
fi

if [[ ! -f "$PARAM_FILE" ]]; then
  echo "P4 SITL parameter file was not found at: $PARAM_FILE" >&2
  exit 1
fi

# Keep EEPROM, parameters and DataFlash logs separate from MiniHawk and other
# ArduPilot vehicles that may use $HOME/ardupilot/ArduPlane as their cwd.
mkdir -p "$SITL_STATE_DIR/tmp"

# ArduPilot's run_in_terminal_window.sh expands the child command with $*
# instead of preserving argv quoting. A --defaults path containing spaces is
# therefore truncated (this repository has "P4 Transwing Clone Drone" in its
# name). Give sim_vehicle a short runtime copy with no spaces.
install -m 0644 "$PARAM_FILE" "$PARAM_RUNTIME"

out_args=()
# sim_vehicle.py detects WSL2 and adds the Windows-host UDP 14550 output.
# Only add the dashboard's independent MAVLink endpoint here.
if [[ -n "${P4_TRANSWING_EXTRA_MAVLINK_OUT:-}" ]]; then
  out_args+=(--out="${P4_TRANSWING_EXTRA_MAVLINK_OUT}")
fi

console_args=()
if [[ "${P4_TRANSWING_MAVPROXY_CONSOLE:-1}" != "0" ]]; then
  console_args=(--console)
fi

# The JSON backend can take a few seconds to expose SERIAL0 after Gazebo has
# loaded all mesh and aerodynamic plugins. Override sim_vehicle's short
# default of five immediate MAVProxy retries so startup is deterministic.
mavproxy_args=(
  --mavproxy-args="--retries=${P4_TRANSWING_MAVPROXY_RETRIES:-30}"
)

cd "$ARDUPILOT_HOME/ArduPlane"
export SITL_RITW_MINIMIZE="${SITL_RITW_MINIMIZE:-0}"
export PATH="$HOME/.local/bin:$PATH"
export TMPDIR="$SITL_STATE_DIR/tmp"

# In one-click / CI mode, prevent sim_vehicle.py from launching ArduPlane in
# an XTerm that can disappear with WSLg focus changes. Its terminal helper
# instead writes to the dedicated $TMPDIR/ArduPlane.log.
if [[ "${P4_TRANSWING_HEADLESS:-0}" == "1" ]]; then
  unset DISPLAY
  unset WAYLAND_DISPLAY
fi

exec "$SIM_VEHICLE" \
  -v ArduPlane \
  -f gazebo-zephyr \
  --model JSON \
  --custom-location="$SITL_HOME" \
  --use-dir="$SITL_STATE_DIR" \
  -N \
  "${wipe_args[@]}" \
  "${console_args[@]}" \
  "${mavproxy_args[@]}" \
  "${out_args[@]}" \
  --add-param-file="$PARAM_RUNTIME"
