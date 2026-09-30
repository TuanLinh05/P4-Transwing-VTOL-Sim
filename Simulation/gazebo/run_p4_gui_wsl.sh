#!/usr/bin/env bash

# WSLg / Gazebo GUI helper adapted from the MiniHawk control station.
set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
export P4_TRANSWING_REPO="${P4_TRANSWING_REPO:-$(cd "$SCRIPT_DIR/../.." && pwd -P)}"
export GZ_PARTITION="${P4_TRANSWING_GZ_PARTITION:-${GZ_PARTITION:-p4_transwing}}"
export GZ_SIM_RESOURCE_PATH="${P4_TRANSWING_REPO}/Simulation/gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"

if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "[p4-wsl-gui] WSLg display is unavailable (DISPLAY and WAYLAND_DISPLAY are empty)." >&2
  echo "[p4-wsl-gui] Start this from a WSLg-enabled Windows session, or run headless." >&2
  exit 3
fi

if command -v xdotool >/dev/null 2>&1 && command -v wmctrl >/dev/null 2>&1; then
(
  window_id=""
  for _ in $(seq 1 30); do
    # Gazebo versions and WSLg can append a world name to the title. Match the
    # application name rather than requiring the exact old title.
    window_id="$(xdotool search --name '[Gg]azebo' 2>/dev/null | tail -n 1)"
    if [[ -n "$window_id" ]]; then
      break
    fi
    sleep 1
  done

  if [[ -z "$window_id" ]]; then
    echo "[p4-wsl-gui] Gazebo window was not found after 30 seconds." >&2
    exit 1
  fi

  window_hex="$(printf '0x%x' "$window_id")"
  wmctrl -i -r "$window_hex" -b remove,maximized_vert,maximized_horz 2>/dev/null || true
  wmctrl -i -r "$window_hex" -e 0,120,80,1280,800 2>/dev/null || true
  xdotool windowmap "$window_id" 2>/dev/null || true
  xdotool windowraise "$window_id" 2>/dev/null || true
  wmctrl -i -a "$window_hex" 2>/dev/null || true
) &
window_helper_pid=$!
else
  echo "[p4-wsl-gui] xdotool/wmctrl are unavailable; skipping window placement." >&2
  window_helper_pid=""
fi

software_rendering="${P4_TRANSWING_SOFTWARE_RENDERING:-1}"
render_engine="${P4_TRANSWING_RENDER_ENGINE:-ogre}"

LIBGL_ALWAYS_SOFTWARE="$software_rendering" \
QT_X11_NO_MITSHM=1 \
QT_QPA_PLATFORM=xcb \
QT_SCALE_FACTOR=1 \
gz sim --force-version 8 -g -v4 --render-engine-gui "$render_engine"

gui_status=$?
if [[ -n "$window_helper_pid" ]]; then
  kill "$window_helper_pid" 2>/dev/null || true
  wait "$window_helper_pid" 2>/dev/null || true
fi
exit "$gui_status"
