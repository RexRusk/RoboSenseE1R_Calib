#!/usr/bin/env bash
# Continue after ROS 2 Humble is already installed.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -d "${HOME:-}" ]]; then
  export HOME="$(getent passwd "$(id -un)" | cut -d: -f6)"
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
echo "finish_setup done"
