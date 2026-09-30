#!/usr/bin/env bash
set -euo pipefail

readonly FOLD_CRUISE="-2.059432253"
readonly FOLD_TRANSITION="-1.029716127"
readonly FOLD_HOVER="0.0"
readonly MOTOR_HOVER_INNER_ESTIMATE="500.0"
readonly MOTOR_HOVER_OUTER_ESTIMATE="730.0"

usage() {
  cat <<'EOF'
Usage: test_p4_commands.sh ACTION

Geometry:
  cruise       Wings extended; propeller thrust axes point forward
  transition   Half fold
  hover        Wings folded; propeller thrust axes point upward
  demo         Sweep cruise -> transition -> hover -> cruise

Propulsion:
  motors-stop  Command all rotors to zero
  motors-idle  Command all rotors to 150 rad/s
  motors-hover Command MTOW hover test: inner 500, outer 730 rad/s

Control surfaces:
  roll-left / roll-right
  pitch-up / pitch-down
  yaw-left / yaw-right
  neutral
EOF
}

publish_double() {
  local topic="$1"
  local value="$2"
  gz topic -t "$topic" -m gz.msgs.Double -p "data:${value}"
  sleep 0.15
  gz topic -t "$topic" -m gz.msgs.Double -p "data:${value}"
}

set_fold() {
  publish_double /p4/fold_left_cmd "$1"
  publish_double /p4/fold_right_cmd "$1"
}

set_motors() {
  local speed="$1"
  gz topic -t /p4/command/motor_speed -m gz.msgs.Actuators \
    -p "velocity:[$speed,$speed,$speed,$speed]"
}

set_hover_motors() {
  # Actuator order follows the UAFM: RIB, ROB, LOB, LIB.
  # Outer rotors need more thrust because the estimated folded CG is aft of
  # the midpoint between the forward (inner) and aft (outer) rotor rows.
  gz topic -t /p4/command/motor_speed -m gz.msgs.Actuators \
    -p "velocity:[$MOTOR_HOVER_INNER_ESTIMATE,$MOTOR_HOVER_OUTER_ESTIMATE,$MOTOR_HOVER_OUTER_ESTIMATE,$MOTOR_HOVER_INNER_ESTIMATE]"
}

set_ailerons() {
  publish_double /p4/aileron_left_inboard_cmd "$1"
  publish_double /p4/aileron_left_outboard_cmd "$1"
  publish_double /p4/aileron_right_inboard_cmd "$2"
  publish_double /p4/aileron_right_outboard_cmd "$2"
}

set_ruddervators() {
  publish_double /p4/ruddervator_left_cmd "$1"
  publish_double /p4/ruddervator_right_cmd "$2"
}

neutral_surfaces() {
  set_ailerons 0.0 0.0
  set_ruddervators 0.0 0.0
}

if [[ $# -ne 1 ]]; then
  usage
  exit 2
fi

if ! gz topic -l | grep -qx "/p4/fold_left_cmd"; then
  echo "P4 control topics were not found."
  echo "Start p4_articulation.sdf or p4_flight.sdf first."
  exit 1
fi

case "$1" in
  cruise) set_fold "$FOLD_CRUISE" ;;
  transition) set_fold "$FOLD_TRANSITION" ;;
  hover) set_fold "$FOLD_HOVER" ;;
  demo)
    set_fold "$FOLD_CRUISE"; sleep 3
    set_fold "$FOLD_TRANSITION"; sleep 4
    set_fold "$FOLD_HOVER"; sleep 7
    set_fold "$FOLD_TRANSITION"; sleep 4
    set_fold "$FOLD_CRUISE"
    ;;
  motors-stop) set_motors 0.0 ;;
  motors-idle) set_motors 150.0 ;;
  motors-hover) set_hover_motors ;;
  roll-left) set_ailerons 0.20 -0.20 ;;
  roll-right) set_ailerons -0.20 0.20 ;;
  pitch-up) set_ruddervators 0.18 0.18 ;;
  pitch-down) set_ruddervators -0.18 -0.18 ;;
  yaw-left) set_ruddervators 0.16 -0.16 ;;
  yaw-right) set_ruddervators -0.16 0.16 ;;
  neutral)
    set_fold "$FOLD_CRUISE"
    neutral_surfaces
    set_motors 0.0
    ;;
  *)
    usage
    exit 2
    ;;
esac

echo "Applied P4 estimated-model command: $1"
