#!/usr/bin/env bash
# Auto-follow the P4 Transwing drone in Gazebo GUI.
# Called after Gazebo GUI has started.
# Uses gz service to set camera tracking on the drone model.

set -uo pipefail

export GZ_PARTITION="${P4_TRANSWING_GZ_PARTITION:-${GZ_PARTITION:-p4_transwing}}"

# Wait for GUI to be ready
sleep 3

echo "[p4-follow] Setting camera to follow p4_transwing_ardupilot..."

# Follow the drone model with close offset: 5m behind, 0m side, 2m above
gz service -s /gui/follow \
  --reqtype gz.msgs.StringMsg \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'data: "p4_transwing_ardupilot"' 2>/dev/null || true

sleep 0.5

# Set close follow offset (x=behind, y=side, z=above)
gz service -s /gui/follow/offset \
  --reqtype gz.msgs.Vector3d \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'x: -5.0, y: 0.0, z: 2.0' 2>/dev/null || true

echo "[p4-follow] Camera follow configured: 5m behind, 2m above drone."
