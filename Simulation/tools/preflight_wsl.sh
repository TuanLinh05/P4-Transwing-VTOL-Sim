#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
P4_REPO="${P4_TRANSWING_REPO:-$(cd "$SCRIPT_DIR/../.." && pwd -P)}"
SIM_ROOT="$P4_REPO/Simulation"
COMPATIBILITY="$SIM_ROOT/config/compatibility.json"
P4_REFERENCE="$SIM_ROOT/config/p4_reference.json"
ARDUPILOT_HOME="${P4_ARDUPILOT_HOME:-$HOME/ardupilot}"
ARDUPILOT_GAZEBO_HOME="${P4_ARDUPILOT_GAZEBO_HOME:-$HOME/ardupilot_gazebo}"
PLUGIN="$ARDUPILOT_GAZEBO_HOME/build/libArduPilotPlugin.so"
SIM_VEHICLE="$ARDUPILOT_HOME/Tools/autotest/sim_vehicle.py"
ARDUPLANE="$ARDUPILOT_HOME/build/sitl/bin/arduplane"
MODEL="$SIM_ROOT/gazebo/models/p4_transwing_ardupilot/model.sdf"
WORLD="$SIM_ROOT/gazebo/worlds/p4_ardupilot.sdf"
STATE_DIR="$SIM_ROOT/ardupilot/sitl_state"

deep=0
quiet=0
check_ports=1
strict_versions="${P4_TRANSWING_STRICT_VERSIONS:-0}"

usage() {
  cat <<'EOF'
Usage: preflight_wsl.sh [--deep] [--quiet] [--no-port-check]

  --deep           Also run the Python unit-test suite.
  --quiet          Only print warnings, errors and the final summary.
  --no-port-check  Skip UDP 9002 / TCP 5760 availability checks.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --deep) deep=1 ;;
    --quiet) quiet=1 ;;
    --no-port-check) check_ports=0 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown preflight option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

errors=0
warnings=0

ok() {
  if [[ "$quiet" -ne 1 ]]; then
    printf '[ OK ] %s\n' "$1"
  fi
}

warn() {
  warnings=$((warnings + 1))
  printf '[WARN] %s\n' "$1" >&2
}

fail() {
  errors=$((errors + 1))
  printf '[FAIL] %s\n' "$1" >&2
}

require_command() {
  local command="$1"
  local label="$2"
  if command -v "$command" >/dev/null 2>&1; then
    ok "$label: $(command -v "$command")"
  else
    fail "$label is missing (command: $command)"
  fi
}

if [[ ! -d "$SIM_ROOT/gazebo" || ! -f "$COMPATIBILITY" ]]; then
  fail "Repository layout is incomplete: $P4_REPO"
else
  ok "Repository: $P4_REPO"
fi

require_command python3 "Python 3"
require_command gz "Gazebo command"
require_command git "Git"

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
  case "${VERSION_ID:-}" in
    22.04|24.04) ok "Ubuntu ${VERSION_ID} is supported" ;;
    *) warn "Ubuntu ${VERSION_ID:-unknown} is not in the tested 22.04/24.04 set" ;;
  esac
else
  warn "Unable to read /etc/os-release"
fi

if command -v gz >/dev/null 2>&1; then
  gazebo_versions="$(gz sim --versions 2>/dev/null || true)"
  if grep -Eq '(^|[[:space:]])8\.[0-9]+' <<<"$gazebo_versions"; then
    ok "Gazebo Harmonic detected: $(tr '\n' ' ' <<<"$gazebo_versions")"
  else
    fail "Gazebo Sim 8.x (Harmonic) is required; found: ${gazebo_versions:-none}"
  fi
fi

if [[ -x "$SIM_VEHICLE" ]]; then
  ok "sim_vehicle.py: $SIM_VEHICLE"
else
  fail "sim_vehicle.py was not found: $SIM_VEHICLE"
fi

if [[ -x "$ARDUPLANE" ]]; then
  ok "ArduPlane SITL binary: $ARDUPLANE"
else
  fail "ArduPlane SITL is not built: $ARDUPLANE"
fi

if [[ -f "$PLUGIN" ]]; then
  ok "ArduPilot Gazebo plugin: $PLUGIN"
  missing_libraries="$(ldd "$PLUGIN" 2>/dev/null | awk '/not found/{print $1}' | paste -sd, -)"
  if [[ -n "$missing_libraries" ]]; then
    fail "ArduPilotPlugin has unresolved libraries: $missing_libraries"
  else
    ok "ArduPilotPlugin shared libraries resolve"
  fi
else
  fail "ArduPilotPlugin was not found: $PLUGIN"
fi

if python3 -c 'from pymavlink import mavutil' >/dev/null 2>&1; then
  ok "pymavlink import succeeded"
else
  fail "Python package pymavlink is missing"
fi

export PATH="$HOME/.local/bin:$PATH"
require_command mavproxy.py "MAVProxy"

if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  warn "WSLg display variables are absent; headless simulation works, Gazebo GUI may not"
else
  ok "WSLg display is available"
fi
for helper in xdotool wmctrl; do
  if ! command -v "$helper" >/dev/null 2>&1; then
    warn "$helper is missing; automatic Gazebo window recovery is disabled"
  fi
done

for asset in \
  "$MODEL" \
  "$WORLD" \
  "$SIM_ROOT/gazebo/models/p4_transwing_estimated/meshes/p4_fuselage.obj" \
  "$SIM_ROOT/gazebo/models/p4_transwing_estimated/meshes/p4_left_wing.obj" \
  "$SIM_ROOT/gazebo/models/p4_transwing_estimated/meshes/p4_right_wing.obj" \
  "$SIM_ROOT/gazebo/models/p4_transwing_estimated/meshes/p4_texture.jpg" \
  "$P4_REFERENCE" \
  "$SIM_ROOT/ardupilot/p4_transwing_sitl.param"; do
  if [[ ! -s "$asset" ]]; then
    fail "Required project asset is missing or empty: $asset"
  fi
done

if command -v python3 >/dev/null 2>&1; then
  if python3 - "$COMPATIBILITY" "$P4_REFERENCE" <<'PY'
import json
import sys

for path in sys.argv[1:]:
    with open(path, encoding="utf-8") as stream:
        json.load(stream)
PY
  then
    ok "Compatibility and P4 reference manifests are valid JSON"
  else
    fail "Compatibility or P4 reference manifest is invalid JSON"
  fi
fi

if command -v gz >/dev/null 2>&1 && [[ -s "$MODEL" && -s "$WORLD" ]]; then
  export GZ_VERSION=harmonic
  export GZ_SIM_RESOURCE_PATH="$SIM_ROOT/gazebo/models:$ARDUPILOT_GAZEBO_HOME/models:$ARDUPILOT_GAZEBO_HOME/worlds:${GZ_SIM_RESOURCE_PATH:-}"
  # `gz sdf` resolves model:// includes through SDF_PATH. Gazebo Sim itself
  # uses GZ_SIM_RESOURCE_PATH, so validate with both search paths configured.
  export SDF_PATH="$SIM_ROOT/gazebo/models:$ARDUPILOT_GAZEBO_HOME/models:$ARDUPILOT_GAZEBO_HOME/worlds:${SDF_PATH:-}"
  if gz sdf -k "$MODEL" >/dev/null 2>&1 && gz sdf -k "$WORLD" >/dev/null 2>&1; then
    ok "SDF model and world validation passed"
  else
    fail "SDF validation failed; run 'gz sdf -k' on the P4 model/world for details"
  fi
fi

mkdir -p "$STATE_DIR" 2>/dev/null || true
write_probe="$STATE_DIR/.preflight_write_$$"
if (umask 077 && : >"$write_probe") 2>/dev/null; then
  rm -f "$write_probe"
  ok "SITL state directory is writable"
else
  fail "SITL state directory is not writable: $STATE_DIR"
fi

if [[ "$check_ports" -eq 1 ]] && command -v python3 >/dev/null 2>&1; then
  if python3 - <<'PY'
import socket

checks = [
    (socket.SOCK_DGRAM, "127.0.0.1", 9002),
    (socket.SOCK_STREAM, "127.0.0.1", 5760),
]
for kind, host, port in checks:
    sock = socket.socket(socket.AF_INET, kind)
    try:
        if kind == socket.SOCK_STREAM:
            # A recently stopped SITL leaves TCP 5760 in TIME_WAIT. ArduPlane
            # can safely reuse that local address, so do not report it as a
            # stale live process.
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, port))
    finally:
        sock.close()
PY
  then
    ok "SITL ports UDP 9002 and TCP 5760 are available"
  else
    fail "UDP 9002 or TCP 5760 is already in use; stop stale Gazebo/SITL processes"
  fi
fi

if [[ -f "$COMPATIBILITY" ]] && command -v python3 >/dev/null 2>&1; then
  expected_ardupilot="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["tested_environment"]["ardupilot_commit"])' "$COMPATIBILITY")"
  expected_plugin="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["tested_environment"]["ardupilot_gazebo_commit"])' "$COMPATIBILITY")"
  expected_gazebo="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["tested_environment"]["gazebo_sim"])' "$COMPATIBILITY")"
  expected_python="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["tested_environment"]["python"])' "$COMPATIBILITY")"
  expected_pymavlink="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["tested_environment"]["pymavlink"])' "$COMPATIBILITY")"
  expected_mavproxy="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["tested_environment"]["mavproxy"])' "$COMPATIBILITY")"
  actual_ardupilot="$(git -C "$ARDUPILOT_HOME" rev-parse HEAD 2>/dev/null || true)"
  actual_plugin="$(git -C "$ARDUPILOT_GAZEBO_HOME" rev-parse HEAD 2>/dev/null || true)"
  for component in ardupilot plugin; do
    if [[ "$component" == "ardupilot" ]]; then
      expected="$expected_ardupilot"
      actual="$actual_ardupilot"
    else
      expected="$expected_plugin"
      actual="$actual_plugin"
    fi
    if [[ -n "$actual" && "$actual" != "$expected" ]]; then
      message="$component commit differs from tested manifest: ${actual:0:12} vs ${expected:0:12}"
      if [[ "$strict_versions" == "1" ]]; then fail "$message"; else warn "$message"; fi
    elif [[ "$actual" == "$expected" ]]; then
      ok "$component commit matches tested manifest (${actual:0:12})"
    else
      warn "Unable to identify $component git commit"
    fi
  done

  compare_tested_version() {
    local component="$1"
    local actual="$2"
    local expected="$3"
    if [[ -z "$actual" ]]; then
      warn "Unable to identify $component version"
    elif [[ "$actual" != "$expected" ]]; then
      local message="$component version differs from tested manifest: $actual vs $expected"
      if [[ "$strict_versions" == "1" ]]; then fail "$message"; else warn "$message"; fi
    else
      ok "$component version matches tested manifest ($actual)"
    fi
  }

  actual_gazebo="$(grep -Eo '8\.[0-9]+(\.[0-9]+)?' <<<"${gazebo_versions:-}" | head -n 1 || true)"
  actual_python="$(python3 -c 'import platform; print(platform.python_version())' 2>/dev/null || true)"
  actual_pymavlink="$(python3 -c 'import importlib.metadata; print(importlib.metadata.version("pymavlink"))' 2>/dev/null || true)"
  actual_mavproxy="$(python3 -c 'import importlib.metadata; print(importlib.metadata.version("MAVProxy"))' 2>/dev/null || true)"
  compare_tested_version "Gazebo Sim" "$actual_gazebo" "$expected_gazebo"
  compare_tested_version "Python" "$actual_python" "$expected_python"
  compare_tested_version "pymavlink" "$actual_pymavlink" "$expected_pymavlink"
  compare_tested_version "MAVProxy" "$actual_mavproxy" "$expected_mavproxy"
fi

if [[ "$deep" -eq 1 && "$errors" -eq 0 ]]; then
  if (cd "$P4_REPO" && python3 -m unittest Simulation.control_station.test_control_station -q); then
    ok "Control Station unit tests passed"
  else
    fail "Control Station unit tests failed"
  fi
fi

if [[ "$errors" -gt 0 ]]; then
  printf 'P4 preflight FAILED: %d error(s), %d warning(s).\n' "$errors" "$warnings" >&2
  exit 1
fi

printf 'P4 preflight PASSED: 0 errors, %d warning(s).\n' "$warnings"
