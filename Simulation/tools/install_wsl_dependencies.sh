#!/usr/bin/env bash
set -euo pipefail

# Reproducible installer for a NEW Ubuntu WSL environment. It deliberately
# refuses to modify existing ArduPilot source trees.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ARDUPILOT_COMMIT="4f50f1997fec263e648119402991beda1071bc7c"
ARDUPILOT_GAZEBO_COMMIT="082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5"
ARDUPILOT_HOME="${P4_ARDUPILOT_HOME:-$HOME/ardupilot}"
ARDUPILOT_GAZEBO_HOME="${P4_ARDUPILOT_GAZEBO_HOME:-$HOME/ardupilot_gazebo}"

if [[ "${1:-}" != "--yes" || $# -ne 1 ]]; then
  cat <<'EOF'
This script installs Gazebo Harmonic packages, clones the tested ArduPilot and
ardupilot_gazebo commits, installs ArduPilot prerequisites, and builds Plane
SITL plus ArduPilotPlugin.

It uses sudo, apt and the network. Review the script, then run:
  bash Simulation/tools/install_wsl_dependencies.sh --yes
EOF
  exit 2
fi

if [[ ! -r /etc/os-release ]]; then
  echo "Unable to identify Ubuntu version." >&2
  exit 1
fi
# shellcheck disable=SC1091
source /etc/os-release
case "${ID:-}:${VERSION_ID:-}" in
  ubuntu:22.04|ubuntu:24.04) ;;
  *)
    echo "Supported installer targets are Ubuntu 22.04 and 24.04; found ${PRETTY_NAME:-unknown}." >&2
    exit 1
    ;;
esac

for target in "$ARDUPILOT_HOME" "$ARDUPILOT_GAZEBO_HOME"; do
  if [[ -e "$target" ]]; then
    echo "Refusing to modify existing path: $target" >&2
    echo "Move it aside or use Simulation/tools/preflight_wsl.sh to inspect it." >&2
    exit 1
  fi
done

sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg git build-essential cmake \
  xdotool wmctrl libopencv-dev libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad \
  gstreamer1.0-libav gstreamer1.0-gl rapidjson-dev

sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" |
  sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
sudo apt-get update
sudo apt-get install -y gz-harmonic libgz-sim8-dev

git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_HOME"
git -C "$ARDUPILOT_HOME" checkout --detach "$ARDUPILOT_COMMIT"
git -C "$ARDUPILOT_HOME" submodule update --init --recursive

(cd "$ARDUPILOT_HOME" && Tools/environment_install/install-prereqs-ubuntu.sh -y)
export PATH="$HOME/.local/bin:$PATH"
(cd "$ARDUPILOT_HOME" && ./waf configure --board sitl && ./waf plane)

git clone https://github.com/ArduPilot/ardupilot_gazebo.git "$ARDUPILOT_GAZEBO_HOME"
git -C "$ARDUPILOT_GAZEBO_HOME" checkout --detach "$ARDUPILOT_GAZEBO_COMMIT"
export GZ_VERSION=harmonic
cmake -S "$ARDUPILOT_GAZEBO_HOME" -B "$ARDUPILOT_GAZEBO_HOME/build" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$ARDUPILOT_GAZEBO_HOME/build" --parallel "$(nproc)"

echo
echo "Dependencies built. Running the complete P4 compatibility check..."
bash "$SCRIPT_DIR/preflight_wsl.sh" --deep
