#!/usr/bin/env bash
set -euo pipefail

# Direct-WSL launcher for the heavier remote-workstation scenery world.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
export P4_TRANSWING_GAZEBO_WORLD="scenery"
exec bash "$SCRIPT_DIR/start_control_station_wsl.sh" "$@"
