#!/usr/bin/env bash
set -euo pipefail

# Connection-only integration test. It starts a private Gazebo transport
# partition and a P4-only SITL state directory, verifies the JSON handshake,
# and exits without arming the aircraft.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
P4_REPO="${P4_TRANSWING_REPO:-$(cd "$SCRIPT_DIR/../.." && pwd -P)}"
GAZEBO_RUNNER="$P4_REPO/Simulation/gazebo/run_p4_ardupilot_server_wsl.sh"
SITL_RUNNER="$P4_REPO/Simulation/ardupilot/run_p4_sitl_wsl.sh"
LOG_DIR="$P4_REPO/Simulation/ardupilot/sitl_state/smoke_logs"
mkdir -p "$LOG_DIR"
LOG_ROOT="$LOG_DIR/p4_ardupilot_smoke_$$"
GAZEBO_LOG="${LOG_ROOT}_gazebo.log"
SITL_LOG="${LOG_ROOT}_sitl.log"
SITL_STDIN="${TMPDIR:-/tmp}/p4_ardupilot_smoke_${UID}_$$_stdin"
ARDUPLANE_LOG="/tmp/ArduPlane.log"
PARAM_SNAPSHOT="$P4_REPO/Simulation/ardupilot/sitl_state/mav.parm"
WORLD_PATH="$P4_REPO/Simulation/gazebo/worlds/p4_ardupilot.sdf"
PARAM_PROCESS_TAG="p4_transwing_sitl_${UID}.param"
MAVPROXY_PROCESS_TAG="127.0.0.1:14551"

gazebo_pid=""
sitl_pid=""

terminate_process_group() {
  local pid="${1:-}"
  if [[ -z "$pid" ]] || ! kill -0 -- "-$pid" 2>/dev/null; then
    return
  fi
  kill -TERM -- "-$pid" 2>/dev/null || true
  for _ in $(seq 1 20); do
    if ! kill -0 -- "-$pid" 2>/dev/null; then
      return
    fi
    sleep 0.1
  done
  kill -KILL -- "-$pid" 2>/dev/null || true
}

cleanup() {
  terminate_process_group "$sitl_pid"
  terminate_process_group "$gazebo_pid"
  # WSL's /init may re-parent sim_vehicle helpers, so their PID is not always
  # the PID returned by $!. These signatures are unique to this P4 smoke
  # stack; the preflight check below prevents touching a user-started stack.
  pkill -INT -f -- "$PARAM_PROCESS_TAG" 2>/dev/null || true
  pkill -INT -f -- "$MAVPROXY_PROCESS_TAG" 2>/dev/null || true
  pkill -INT -f -- "$WORLD_PATH" 2>/dev/null || true
  sleep 0.5
  pkill -KILL -f -- "$PARAM_PROCESS_TAG" 2>/dev/null || true
  pkill -KILL -f -- "$MAVPROXY_PROCESS_TAG" 2>/dev/null || true
  pkill -KILL -f -- "$WORLD_PATH" 2>/dev/null || true
  exec 9>&- 2>/dev/null || true
  rm -f "$SITL_STDIN"
}

for required in "$GAZEBO_RUNNER" "$SITL_RUNNER"; do
  if [[ ! -f "$required" ]]; then
    echo "Required runner was not found: $required" >&2
    exit 1
  fi
done

if pgrep -f -- "$WORLD_PATH" >/dev/null ||
   pgrep -f -- "$PARAM_PROCESS_TAG" >/dev/null ||
   pgrep -f -- "$MAVPROXY_PROCESS_TAG" >/dev/null; then
  echo "A P4 Gazebo/SITL stack is already running; stop it before this smoke test." >&2
  exit 1
fi

trap cleanup EXIT INT TERM

# A private transport partition prevents stale Gazebo topics from making the
# test pass. UDP 9002 remains the standard ardupilot_gazebo JSON endpoint.
export GZ_PARTITION="p4_ardupilot_smoke_$$"
export P4_TRANSWING_REPO="$P4_REPO"

# Give each stack component a private process group so cleanup also reaches
# helpers re-parented by WSL /init.
setsid bash "$GAZEBO_RUNNER" >"$GAZEBO_LOG" 2>&1 &
gazebo_pid=$!

for _ in $(seq 1 150); do
  if grep -Fq "World [" "$GAZEBO_LOG" 2>/dev/null; then
    break
  fi
  if ! kill -0 "$gazebo_pid" 2>/dev/null; then
    echo "Gazebo exited during startup." >&2
    tail -120 "$GAZEBO_LOG" >&2
    exit 1
  fi
  sleep 0.2
done

if ! grep -Fq "World [" "$GAZEBO_LOG" 2>/dev/null; then
  echo "Gazebo did not report world readiness within 30 seconds." >&2
  tail -120 "$GAZEBO_LOG" >&2
  exit 1
fi

# MAVProxy is interactive and exits cleanly on stdin EOF. The dashboard keeps
# a pipe open for it; reproduce that behavior here with a private FIFO.
rm -f "$ARDUPLANE_LOG"
rm -f "$PARAM_SNAPSHOT"
mkfifo "$SITL_STDIN"
exec 9<>"$SITL_STDIN"
setsid env \
  P4_TRANSWING_MAVPROXY_CONSOLE=0 \
  P4_TRANSWING_HEADLESS=1 \
  P4_TRANSWING_EXTRA_MAVLINK_OUT=127.0.0.1:14551 \
  SITL_RITW_MINIMIZE=1 \
  bash "$SITL_RUNNER" --wipe <&9 >"$SITL_LOG" 2>&1 &
sitl_pid=$!

connected=0
for _ in $(seq 1 300); do
  # gzlog may route the plugin's "Connected" line to ~/.gz instead of stdout.
  # A vehicle heartbeat plus ArduPlane's "JSON received" block proves both
  # the MAVLink side and the Gazebo JSON/FDM side are online.
  if grep -Fq "Detected vehicle" "$SITL_LOG" 2>/dev/null &&
     grep -Fq "JSON received:" "$ARDUPLANE_LOG" 2>/dev/null; then
    connected=1
    break
  fi
  if ! kill -0 "$sitl_pid" 2>/dev/null; then
    echo "ArduPlane SITL exited before the Gazebo handshake." >&2
    tail -160 "$SITL_LOG" >&2
    if [[ -f "$ARDUPLANE_LOG" ]]; then
      echo "--- ArduPlane ---" >&2
      tail -160 "$ARDUPLANE_LOG" >&2
    fi
    exit 1
  fi
  if ! kill -0 "$gazebo_pid" 2>/dev/null; then
    echo "Gazebo exited before the ArduPilot handshake." >&2
    tail -160 "$GAZEBO_LOG" >&2
    exit 1
  fi
  sleep 0.2
done

if [[ "$connected" -ne 1 ]]; then
  echo "Gazebo and ArduPilot did not complete the JSON handshake in 60 seconds." >&2
  echo "--- Gazebo ---" >&2
  tail -160 "$GAZEBO_LOG" >&2
  echo "--- SITL ---" >&2
  tail -160 "$SITL_LOG" >&2
  exit 1
fi

# Give MAVProxy enough time to download and atomically finish writing the
# parameter snapshot. Heartbeat readiness happens earlier, so checking only
# "Detected vehicle" is racy on slower disks and fresh WSL machines.
parameter_snapshot_ready=0
for _ in $(seq 1 150); do
  if [[ -s "$PARAM_SNAPSHOT" ]] &&
     grep -Fq "Saved " "$SITL_LOG" 2>/dev/null; then
    parameter_snapshot_ready=1
    break
  fi
  if ! kill -0 "$sitl_pid" 2>/dev/null; then
    break
  fi
  sleep 0.2
done

if grep -Eqi \
  "unknown parameter|invalid parameter|failed to set|parameter .* not found" \
  "$SITL_LOG" "$ARDUPLANE_LOG"; then
  echo "ArduPlane rejected at least one P4 parameter." >&2
  grep -Ei \
    "unknown parameter|invalid parameter|failed to set|parameter .* not found" \
    "$SITL_LOG" "$ARDUPLANE_LOG" >&2
  exit 1
fi

if [[ "$parameter_snapshot_ready" -ne 1 ]]; then
  echo "MAVProxy did not finish the loaded-parameter snapshot within 30 seconds." >&2
  tail -120 "$SITL_LOG" >&2
  exit 1
fi

require_parameter() {
  local name="$1"
  local expected="$2"
  local actual
  actual="$(awk -v target="$name" '$1 == target {print $2; exit}' "$PARAM_SNAPSHOT")"
  if [[ "$actual" != "$expected" ]]; then
    echo "Unexpected loaded parameter: $name=${actual:-missing}, expected $expected" >&2
    exit 1
  fi
}

require_parameter Q_FRAME_CLASS 1
require_parameter Q_FRAME_TYPE 1
require_parameter Q_TILT_ENABLE 1
require_parameter Q_TILT_MASK 15
require_parameter Q_TILT_TYPE 0
require_parameter Q_TILT_MAX 20
require_parameter Q_TILT_RATE_DN 20
require_parameter Q_M_THST_HOVER 0.600000
require_parameter Q_PILOT_SPD_UP 0.700000
require_parameter Q_LOIT_ANG_MAX 6.000000
require_parameter Q_P_ANGLE_MAX 6.000000
require_parameter Q_A_ANGLE_MAX 15.000000
require_parameter Q_LOIT_ACC_MAX_M 1.000000
require_parameter Q_LOIT_BRK_ACC_M 0.300000
require_parameter Q_LOIT_BRK_JRK_M 1.000000
require_parameter Q_P_JERK_NE 1.000000
require_parameter Q_A_ANG_PIT_P 2.500000
require_parameter Q_A_RAT_PIT_I 0.120000
require_parameter ARSPD_TYPE 100
require_parameter Q_OPTIONS 4096
require_parameter Q_ASSIST_ANGLE 0
require_parameter Q_ASSIST_SPEED -1.000000
require_parameter AIRSPEED_MIN 27
require_parameter AIRSPEED_CRUISE 34.000000
require_parameter SCALING_SPEED 34.000000
require_parameter PTCH_RATE_P 0.080000
require_parameter PTCH_RATE_I 0.500000
require_parameter PTCH_RATE_FF 0.500000
require_parameter ROLL_LIMIT_DEG 25.000000
require_parameter NAVL1_PERIOD 35.000000
require_parameter NAVL1_DAMPING 0.900000
require_parameter RLL2SRV_RMAX 45.000000
require_parameter RLL2SRV_TCONST 0.800000
require_parameter RLL_RATE_FF 0.200000
require_parameter RLL_RATE_P 0.040000
require_parameter RLL_RATE_I 0.100000
require_parameter RLL_RATE_D 0.003000
require_parameter RLL_RATE_SMAX 90.000000
require_parameter Q_TRANSITION_MS 12000
require_parameter Q_TRANS_FAIL 30
require_parameter SERVO5_FUNCTION 41
require_parameter SERVO6_FUNCTION 4
require_parameter SERVO7_FUNCTION 79
require_parameter SERVO8_FUNCTION 80
require_parameter MIXING_GAIN 1.000000
require_parameter ARSPD_USE 1

if grep -Eqi \
  "XML_ERROR|Unable to find uri|Failed to load system plugin|Error Code [0-9]+|imu_sensor .*not found" \
  "$GAZEBO_LOG"; then
  echo "Gazebo reported a model, resource, plugin or IMU error." >&2
  tail -180 "$GAZEBO_LOG" >&2
  exit 1
fi

joint_topic="$(gz topic -l | grep '/joint_state$' | grep 'p4_transwing_ardupilot' | head -n 1 || true)"
if [[ -z "$joint_topic" ]]; then
  echo "P4 ArduPilot joint-state topic was not found." >&2
  gz topic -l >&2 || true
  exit 1
fi

joint_sample="$(timeout 5s gz topic -e -t "$joint_topic" -n 1)"
for joint in left_fold_joint right_fold_joint \
  rotor_right_inner_joint rotor_left_outer_joint \
  rotor_left_inner_joint rotor_right_outer_joint; do
  if [[ "$joint_sample" != *"$joint"* ]]; then
    echo "Joint was missing after the SITL handshake: $joint" >&2
    exit 1
  fi
done

echo "P4 ArduPilot integration smoke test passed."
echo "Gazebo loaded the model and IMU, SITL accepted the parameter file,"
echo "and the JSON controller handshake completed while the aircraft stayed disarmed."
echo "Gazebo log: $GAZEBO_LOG"
echo "SITL log:   $SITL_LOG"
