#!/usr/bin/env bash
# Ubuntu 22.04 + ROS 2 Humble. Tested path is WSL2.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -d "${HOME:-}" ]]; then
  export HOME="$(getent passwd "$(id -un)" | cut -d: -f6)"
fi
export DEBIAN_FRONTEND=noninteractive

wait_for_apt() {
  local i
  for i in $(seq 1 60); do
    if sudo apt-get update -qq; then
      return 0
    fi
    echo "apt is locked, retry ${i}/60"
    sleep 5
  done
  echo "apt stayed locked" >&2
  return 1
}

wait_for_apt
sudo apt-get install -y software-properties-common curl gnupg locales
sudo locale-gen en_US.UTF-8
if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt-get update
sudo apt-get install -y \
  ros-humble-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-venv \
  python3-pip \
  libyaml-cpp-dev \
  libpcap-dev \
  libeigen3-dev \
  libpcl-dev \
  ros-humble-v4l2-camera \
  ros-humble-rosbag2-storage-mcap \
  ros-humble-camera-calibration

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  for _try in 1 2 3 4 5; do
    if sudo rosdep init; then
      break
    fi
    echo "rosdep init failed, retry ${_try}/5"
    sleep 5
  done
fi
if ! rosdep update; then
  echo "rosdep update failed; apt dependencies are already installed, continuing"
fi

bash "${ROOT}/scripts/clone_third_party.sh"
bash "${ROOT}/scripts/build.sh"

VENV="${HOME}/e1r_calib/venv"
python3 -m venv "${VENV}"
# shellcheck disable=SC1091
source "${VENV}/bin/activate"
python -m pip install -U pip wheel
python -m pip install "${ROOT}/src/third_party/ros2_calib"
python -c "import ros2_calib, cv2, open3d, PySide6; print('ros2_calib import ok')"

echo
echo "Environment ready."
echo "  source ${HOME}/e1r_calib/install/setup.bash"
echo "  source ${VENV}/bin/activate"
