#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
P4_SIM_ROOT="${P4_SIM_ROOT:-$(cd "$SCRIPT_DIR/.." && pwd -P)}"
WORLD="$P4_SIM_ROOT/gazebo/worlds/p4_articulation.sdf"
MODEL_PATH="$P4_SIM_ROOT/gazebo/models"
COMMAND_SCRIPT="$P4_SIM_ROOT/gazebo/test_p4_commands.sh"
LOG_FILE="${TMPDIR:-/tmp}/p4_gazebo_smoke_$$.log"

export GZ_SIM_RESOURCE_PATH="$MODEL_PATH:${GZ_SIM_RESOURCE_PATH:-}"

gz sim --force-version 8 -s -r -v3 --iterations 50000 "$WORLD" >"$LOG_FILE" 2>&1 &
server_pid=$!

cleanup() {
  kill "$server_pid" >/dev/null 2>&1 || true
}
trap cleanup EXIT

for _ in $(seq 1 80); do
  if gz topic -l 2>/dev/null | grep -qx "/p4/fold_left_cmd"; then
    break
  fi
  if ! kill -0 "$server_pid" 2>/dev/null; then
    echo "Gazebo exited before P4 control topics became available."
    tail -100 "$LOG_FILE"
    exit 1
  fi
  sleep 0.2
done

required_topics=(
  /p4/fold_left_cmd
  /p4/fold_right_cmd
  /p4/command/motor_speed
)
for topic in "${required_topics[@]}"; do
  if ! gz topic -l | grep -qx "$topic"; then
    echo "Required topic was not found: $topic"
    tail -100 "$LOG_FILE"
    exit 1
  fi
done

joint_topic="$(gz topic -l | grep '/joint_state$' | grep 'p4_transwing' | head -n 1 || true)"
if [[ -z "$joint_topic" ]]; then
  echo "P4 JointStatePublisher topic was not found."
  tail -100 "$LOG_FILE"
  exit 1
fi

fold_position() {
  local sample="$1"
  local joint="$2"
  awk -v target="$joint" '
    index($0, "name: \"" target "\"") { found=1 }
    found && /axis1 \{/ { axis=1 }
    found && axis && /position:/ { print $2; exit }
  ' <<<"$sample"
}

bash "$COMMAND_SCRIPT" cruise
sleep 7
cruise_sample="$(timeout 4s gz topic -e -t "$joint_topic" -n 1)"
for joint in left_fold_joint right_fold_joint; do
  position="$(fold_position "$cruise_sample" "$joint")"
  if [[ -z "$position" ]] ||
     ! awk -v p="$position" 'BEGIN { d=p+2.059432253; if (d<0) d=-d; exit !(d<0.05) }'; then
    echo "Fold joint did not reach the cruise endpoint: $joint (${position:-missing})"
    exit 1
  fi
done

bash "$COMMAND_SCRIPT" hover
sleep 7
joint_sample="$(timeout 4s gz topic -e -t "$joint_topic" -n 1)"
for joint in left_fold_joint right_fold_joint rotor_left_inner_joint rotor_right_inner_joint; do
  if [[ "$joint_sample" != *"$joint"* ]]; then
    echo "Joint was missing from the joint-state sample: $joint"
    exit 1
  fi
done
for joint in left_fold_joint right_fold_joint; do
  position="$(fold_position "$joint_sample" "$joint")"
  if [[ -z "$position" ]] ||
     ! awk -v p="$position" 'BEGIN { if (p<0) p=-p; exit !(p<0.05) }'; then
    echo "Fold joint did not return to the hover endpoint: $joint (${position:-missing})"
    exit 1
  fi
done

bash "$COMMAND_SCRIPT" motors-idle
sleep 1
bash "$COMMAND_SCRIPT" motors-stop

if grep -Eq "(XML_ERROR|Unable to find uri|Failed to load|Error Code [0-9]+)" "$LOG_FILE"; then
  echo "Gazebo reported a model or resource-loading error."
  tail -120 "$LOG_FILE"
  exit 1
fi

echo "P4 Gazebo smoke test passed."
echo "Meshes, fold endpoints, four motor plugins and joint states loaded."
