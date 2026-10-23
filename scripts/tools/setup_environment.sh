#!/usr/bin/env bash
# Install declared dependencies and build from this checkout on ROS 2 Humble.
set -Eeuo pipefail
repo_dir="$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
install_dependencies=true
case "${1:-}" in
  --skip-install) install_dependencies=false ;;
  -h|--help)
    printf 'Usage: bash %s [--skip-install]\nRequires Ubuntu 22.04 and ROS 2 Humble. Livox SDK/driver setup: docs/experiment_04_setup.md\n' "$0"
    exit 0 ;;
  '') ;;
  *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
esac
if (($# > 1)); then printf 'Too many arguments\n' >&2; exit 2; fi
if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  printf 'Install ROS 2 Humble first; see docs/experiment_04_setup.md\n' >&2
  exit 1
fi
# Avoid accidentally building against a previously sourced workspace.
if [[ -n "${AMENT_PREFIX_PATH:-}" && "${AMENT_PREFIX_PATH}" != /opt/ros/humble ]]; then
  printf 'Open a fresh terminal with only /opt/ros/humble sourced, then rerun. AMENT_PREFIX_PATH=%s\n' "$AMENT_PREFIX_PATH" >&2
  exit 1
fi
set +u
source /opt/ros/humble/setup.bash
set -u
cd "${repo_dir}"
if [[ "${install_dependencies}" == true ]]; then
  sudo apt-get update
  sudo apt-get install -y build-essential cmake git python3-colcon-common-extensions python3-rosdep
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
  fi
  rosdep update --rosdistro humble
  rosdep install --from-paths "${repo_dir}" --ignore-src --rosdistro humble -y
fi
colcon build --base-paths "${repo_dir}" --packages-select co_3dto2d_mapping \
  --symlink-install --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release
set +u
source "${repo_dir}/install/local_setup.bash"
set -u
python3 - <<'PY'
import cv2
import numpy
import rclpy
import rosbag2_py
import scipy
import yaml
from co_3dto2d_mapping.multi_agent.alignment import register_startup
from co_3dto2d_mapping.multi_agent import core
print('Mapping and replay Python dependencies: OK')
PY
for package_name in co_3dto2d_mapping rtabmap_odom imu_filter_madgwick rviz2; do
  ros2 pkg prefix "${package_name}"
done
printf '\nBuild complete. ROS_DOMAIN_ID=72 and ROS_LOCALHOST_ONLY=0 are used for the experiment.\n'
printf 'Livox SDK/driver and per-sensor network setup: %s/docs/experiment_04_setup.md\n' "${repo_dir}"
