#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
TRANSWING_SIM_ROOT="${TRANSWING_SIM_ROOT:-$(cd "$SCRIPT_DIR/.." && pwd -P)}"
WORLD="$TRANSWING_SIM_ROOT/gazebo/worlds/transwing_articulation.sdf"
MODEL_PATH="$TRANSWING_SIM_ROOT/gazebo/models"
COMMAND_SCRIPT="$TRANSWING_SIM_ROOT/gazebo/test_transwing_joints.sh"
LOG_FILE="${TMPDIR:-/tmp}/transwing_gazebo_smoke_$$.log"

export GZ_SIM_RESOURCE_PATH="$MODEL_PATH:${GZ_SIM_RESOURCE_PATH:-}"

# A finite run guarantees that the smoke test cannot leave a server behind.
# A generous finite iteration cap is a second cleanup guard in case a failing
# test cannot signal the server normally.
gz sim --force-version 8 -s -r -v2 --iterations 30000 "$WORLD" >"$LOG_FILE" 2>&1 &
server_pid=$!

cleanup() {
  kill "$server_pid" >/dev/null 2>&1 || true
}
trap cleanup EXIT

for _ in $(seq 1 50); do
  if gz topic -l 2>/dev/null | grep -qx "/transwing/fold_left_cmd"; then
    break
  fi
  if ! kill -0 "$server_pid" 2>/dev/null; then
    echo "Gazebo exited before control topics became available."
    tail -80 "$LOG_FILE"
    exit 1
  fi
  sleep 0.2
done

if ! gz topic -l | grep -qx "/transwing/fold_left_cmd"; then
  echo "Timed out waiting for the Transwing control topics."
  tail -80 "$LOG_FILE"
  exit 1
fi

bash "$COMMAND_SCRIPT" hover
sleep 4

joint_topic="$(gz topic -l | grep '/joint_state$' | grep 'transwing_surrogate' | head -n 1)"
if [[ -z "$joint_topic" ]]; then
  echo "JointStatePublisher topic was not found."
  exit 1
fi

joint_sample="$(timeout 3s gz topic -e -t "$joint_topic" -n 1)"
if [[ "$joint_sample" != *"left_fold_joint"* ]] ||
   [[ "$joint_sample" != *"right_fold_joint"* ]]; then
  echo "Fold joints were missing from the joint-state sample."
  echo "$joint_sample"
  exit 1
fi

bash "$COMMAND_SCRIPT" cruise

echo "Gazebo smoke test passed."
echo "Control topics loaded, hover/cruise commands published, and both fold joints reported state."
