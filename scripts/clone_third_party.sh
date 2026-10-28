#!/usr/bin/env bash
# Clone upstream trees on the Linux filesystem, then copy them into src/third_party.
# Does not edit upstream files.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Windows-launched WSL sometimes exports HOME as a host path.
if [[ ! -d "${HOME:-}" ]]; then
  export HOME="$(getent passwd "$(id -un)" | cut -d: -f6)"
fi
DEST="${ROOT}/src/third_party"
CACHE="${HOME}/e1r_cache"
GIT=(git -c http.version=HTTP/1.1 -c http.postBuffer=524288000)

mkdir -p "${CACHE}" "${DEST}"

clone_repo() {
  local url="$1"
  local name="$2"
  if [[ ! -d "${CACHE}/${name}/.git" ]]; then
    "${GIT[@]}" clone --depth 1 "${url}" "${CACHE}/${name}"
  else
    echo "cache exists: ${name}"
  fi
}

clone_repo https://github.com/RoboSense-LiDAR/rslidar_sdk.git rslidar_sdk
if [[ ! -f "${CACHE}/rslidar_sdk/src/rs_driver/CMakeLists.txt" ]]; then
  git -C "${CACHE}/rslidar_sdk" -c http.version=HTTP/1.1 -c http.postBuffer=524288000 \
    submodule update --init --depth 1 src/rs_driver
fi
clone_repo https://github.com/RoboSense-LiDAR/rslidar_msg.git rslidar_msg
clone_repo https://github.com/ika-rwth-aachen/ros2_calib.git ros2_calib

copy_tree() {
  local name="$1"
  if [[ -d "${DEST}/${name}/.git" ]]; then
    echo "already in workspace: ${name}"
    return
  fi
  mkdir -p "${DEST}/${name}"
  cp -a "${CACHE}/${name}/." "${DEST}/${name}/"
}

copy_tree rslidar_sdk
copy_tree rslidar_msg
copy_tree ros2_calib

test -f "${DEST}/rslidar_sdk/src/rs_driver/CMakeLists.txt"
test -f "${DEST}/rslidar_msg/package.xml"
test -f "${DEST}/ros2_calib/pyproject.toml"
echo "third_party ready at ${DEST}"
