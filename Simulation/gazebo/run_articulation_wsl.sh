#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
TRANSWING_SIM_ROOT="${TRANSWING_SIM_ROOT:-$(cd "$SCRIPT_DIR/.." && pwd -P)}"
WORLD="$TRANSWING_SIM_ROOT/gazebo/worlds/transwing_articulation.sdf"
MODEL_PATH="$TRANSWING_SIM_ROOT/gazebo/models"

if ! command -v gz >/dev/null 2>&1; then
  echo "Gazebo 'gz' was not found. Install Gazebo Harmonic or source its environment."
  exit 1
fi

if [[ ! -f "$WORLD" ]]; then
  echo "World file was not found: $WORLD"
  exit 1
fi

export GZ_SIM_RESOURCE_PATH="$MODEL_PATH:${GZ_SIM_RESOURCE_PATH:-}"
exec gz sim --force-version 8 -v4 "$WORLD"
