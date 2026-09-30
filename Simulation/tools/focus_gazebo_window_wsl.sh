#!/usr/bin/env bash
set -euo pipefail

if ! command -v xdotool >/dev/null 2>&1 || ! command -v wmctrl >/dev/null 2>&1; then
  echo "Missing xdotool or wmctrl. Install once with:" >&2
  echo "  sudo apt-get update && sudo apt-get install -y xdotool wmctrl" >&2
  exit 1
fi

window_id=""
for _ in $(seq 1 10); do
  window_id="$(xdotool search --name '[Gg]azebo' 2>/dev/null | tail -n 1 || true)"
  if [[ -n "$window_id" ]]; then
    break
  fi
  sleep 1
done

if [[ -z "$window_id" ]]; then
  echo "Gazebo GUI window was not found. Windows visible to WSLg:" >&2
  wmctrl -l >&2 || true
  exit 1
fi

window_hex="$(printf '0x%x' "$window_id")"
wmctrl -i -r "$window_hex" -b remove,maximized_vert,maximized_horz 2>/dev/null || true
wmctrl -i -r "$window_hex" -e 0,120,80,1280,800 2>/dev/null || true
xdotool windowmap "$window_id" 2>/dev/null || true
xdotool windowraise "$window_id" 2>/dev/null || true
wmctrl -i -a "$window_hex" 2>/dev/null || true
echo "Requested foreground for Gazebo window $window_hex."
