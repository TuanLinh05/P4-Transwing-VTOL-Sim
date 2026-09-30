#!/usr/bin/env bash
set -euo pipefail

readonly FOLD_CRUISE="0.0"
readonly FOLD_MID="1.047197551"
readonly FOLD_HOVER="2.094395102"

usage() {
  cat <<'EOF'
Usage: test_transwing_joints.sh ACTION

Actions:
  cruise          Wings horizontal, propeller axes forward (0 deg)
  transition      Intermediate folding position (60 deg)
  hover           Wings vertical, propeller axes upward (120 deg)
  demo            Sweep cruise -> transition -> hover -> cruise
  roll-left       Opposite aileron deflection
  roll-right      Opposite aileron deflection
  pitch-up        Elevator up
  pitch-down      Elevator down
  yaw-left        Rudder left
  yaw-right       Rudder right
  neutral         Cruise configuration and neutral control surfaces
EOF
}

publish_twice() {
  local topic="$1"
  local value="$2"
  gz topic -t "$topic" -m gz.msgs.Double -p "data:${value}"
  sleep 0.20
  gz topic -t "$topic" -m gz.msgs.Double -p "data:${value}"
}

set_fold() {
  local value="$1"
  publish_twice /transwing/fold_left_cmd "$value"
  publish_twice /transwing/fold_right_cmd "$value"
}

neutral_surfaces() {
  publish_twice /transwing/aileron_left_cmd 0.0
  publish_twice /transwing/aileron_right_cmd 0.0
  publish_twice /transwing/elevator_cmd 0.0
  publish_twice /transwing/rudder_cmd 0.0
}

if [[ $# -ne 1 ]]; then
  usage
  exit 2
fi

if ! gz topic -l | grep -qx "/transwing/fold_left_cmd"; then
  echo "Transwing joint-control topics were not found."
  echo "Start worlds/transwing_articulation.sdf and press Play first."
  exit 1
fi

case "$1" in
  cruise)
    set_fold "$FOLD_CRUISE"
    ;;
  transition)
    set_fold "$FOLD_MID"
    ;;
  hover)
    set_fold "$FOLD_HOVER"
    ;;
  demo)
    set_fold "$FOLD_CRUISE"
    sleep 2
    set_fold "$FOLD_MID"
    sleep 2
    set_fold "$FOLD_HOVER"
    sleep 3
    set_fold "$FOLD_MID"
    sleep 2
    set_fold "$FOLD_CRUISE"
    ;;
  roll-left)
    publish_twice /transwing/aileron_left_cmd 0.22
    publish_twice /transwing/aileron_right_cmd -0.22
    ;;
  roll-right)
    publish_twice /transwing/aileron_left_cmd -0.22
    publish_twice /transwing/aileron_right_cmd 0.22
    ;;
  pitch-up)
    publish_twice /transwing/elevator_cmd 0.25
    ;;
  pitch-down)
    publish_twice /transwing/elevator_cmd -0.25
    ;;
  yaw-left)
    publish_twice /transwing/rudder_cmd 0.28
    ;;
  yaw-right)
    publish_twice /transwing/rudder_cmd -0.28
    ;;
  neutral)
    set_fold "$FOLD_CRUISE"
    neutral_surfaces
    ;;
  *)
    usage
    exit 2
    ;;
esac

echo "Applied Transwing surrogate joint test: $1"
