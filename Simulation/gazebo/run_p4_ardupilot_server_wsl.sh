#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
P4_REPO="${P4_TRANSWING_REPO:-$(cd "$SCRIPT_DIR/../.." && pwd -P)}"
WORLD="$P4_REPO/Simulation/gazebo/worlds/p4_ardupilot.sdf"
MODEL_PATH="$P4_REPO/Simulation/gazebo/models"
ARDUPILOT_GAZEBO_HOME="${P4_ARDUPILOT_GAZEBO_HOME:-$HOME/ardupilot_gazebo}"
PLUGIN_DIR="$ARDUPILOT_GAZEBO_HOME/build"

if ! command -v gz >/dev/null 2>&1; then
  echo "Gazebo 'gz' was not found. Install Gazebo Harmonic first." >&2
  exit 1
fi

if [[ ! -f "$PLUGIN_DIR/libArduPilotPlugin.so" ]]; then
  echo "ArduPilotPlugin was not found under: $PLUGIN_DIR" >&2
  echo "Build the official ardupilot_gazebo plugin before starting this world." >&2
  exit 1
fi

export GZ_VERSION=harmonic
export GZ_PARTITION="${P4_TRANSWING_GZ_PARTITION:-${GZ_PARTITION:-p4_transwing}}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PLUGIN_DIR:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$MODEL_PATH:$ARDUPILOT_GAZEBO_HOME/models:$ARDUPILOT_GAZEBO_HOME/worlds:${GZ_SIM_RESOURCE_PATH:-}"

exec gz sim --force-version 8 -s -r -v4 "$WORLD"
