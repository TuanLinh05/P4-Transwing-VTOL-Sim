#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
P4_SIM_ROOT="${P4_SIM_ROOT:-$(cd "$SCRIPT_DIR/.." && pwd -P)}"
WORLD="$P4_SIM_ROOT/gazebo/worlds/p4_flight.sdf"
MODEL_PATH="$P4_SIM_ROOT/gazebo/models"
COMMAND_SCRIPT="$P4_SIM_ROOT/gazebo/test_p4_commands.sh"
LOG_FILE="${TMPDIR:-/tmp}/p4_gazebo_flight_smoke_$$.log"

export GZ_SIM_RESOURCE_PATH="$MODEL_PATH:${GZ_SIM_RESOURCE_PATH:-}"

# Start paused so the folded state and motor command are established before
# gravity advances the first physics step.
gz sim --force-version 8 -s -v3 --iterations 50000 "$WORLD" >"$LOG_FILE" 2>&1 &
server_pid=$!

cleanup() {
  kill "$server_pid" >/dev/null 2>&1 || true
}
trap cleanup EXIT

for _ in $(seq 1 80); do
  if gz topic -l 2>/dev/null | grep -qx "/p4/command/motor_speed"; then
    break
  fi
  if ! kill -0 "$server_pid" 2>/dev/null; then
    echo "Gazebo flight world exited during startup."
    tail -120 "$LOG_FILE"
    exit 1
  fi
  sleep 0.1
done

bash "$COMMAND_SCRIPT" hover
bash "$COMMAND_SCRIPT" motors-hover

gz service -s /world/p4_flight/control \
  --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req "pause: false" >/dev/null

# Keep this deliberately short. Open-loop multirotors are dynamically
# unstable without an attitude controller; this smoke test only verifies
# the thrust direction, folding-joint integrity and model loading.
P4_FLIGHT_RUN_SECONDS="${P4_FLIGHT_RUN_SECONDS:-0.8}"
sleep "$P4_FLIGHT_RUN_SECONDS"

joint_topic="$(gz topic -l | grep '/joint_state$' | grep 'p4_transwing' | head -n 1 || true)"
if [[ -z "$joint_topic" ]]; then
  echo "P4 joint-state topic was not found in the flight world."
  exit 1
fi

joint_sample="$(timeout 4s gz topic -e -t "$joint_topic" -n 1)"
if [[ "$joint_sample" != *"left_fold_joint"* ]] ||
   [[ "$joint_sample" != *"right_fold_joint"* ]]; then
  echo "Fold joints were missing in the flight-world state."
  exit 1
fi

model_z="$(awk '/^pose \{/{pose=1} pose && /z:/{print $2; exit}' <<<"$joint_sample")"
if [[ -z "$model_z" ]] || ! awk -v z="$model_z" 'BEGIN { exit !(z > 1.5) }'; then
  echo "P4 did not remain airborne during the short open-loop thrust smoke test."
  echo "Reported model Z: ${model_z:-missing}"
  exit 1
fi

bash "$COMMAND_SCRIPT" motors-stop

if grep -Eq "(XML_ERROR|Unable to find uri|Failed to load|Error Code [0-9]+)" "$LOG_FILE"; then
  echo "Gazebo reported a flight-world model or resource error."
  tail -120 "$LOG_FILE"
  exit 1
fi

echo "P4 flight-world smoke test passed."
echo "Hover state and CG-trimmed rotor command produced upward thrust before attitude divergence."
