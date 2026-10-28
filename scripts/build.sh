#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -d "${HOME:-}" ]]; then
  export HOME="$(getent passwd "$(id -un)" | cut -d: -f6)"
fi
# Compile on the Linux disk. Sources stay in this workspace.
BUILD_ROOT="${HOME}/e1r_calib"
mkdir -p "${BUILD_ROOT}"

set +u
source /opt/ros/humble/setup.bash
set -u

cd "${ROOT}"
colcon build \
  --packages-select rslidar_msg rslidar_sdk e1r_calib_bringup \
  --build-base "${BUILD_ROOT}/build" \
  --install-base "${BUILD_ROOT}/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "Source this in every new shell:"
echo "  source ${BUILD_ROOT}/install/setup.bash"
