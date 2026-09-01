#!/usr/bin/env bash

set -euo pipefail

usage() {
  printf 'Usage: %s ROBOT_NUMBER\n' "${0##*/}" >&2
  printf '  ROBOT_NUMBER: 1 for r0, 2 for r1\n' >&2
}

if (($# != 1)); then
  usage
  exit 2
fi

case "$1" in
  1)
    robot_namespace="r0"
    ;;
  2)
    robot_namespace="r1"
    ;;
  *)
    printf 'Invalid robot number: %s\n' "$1" >&2
    usage
    exit 2
    ;;
esac

ros_setup="${ROS_SETUP:-}"
if [[ -z "${ros_setup}" && -n "${ROS_DISTRO:-}" ]]; then
  candidate="/opt/ros/${ROS_DISTRO}/setup.bash"
  if [[ -r "${candidate}" ]]; then
    ros_setup="${candidate}"
  fi
fi
if [[ -z "${ros_setup}" ]]; then
  for candidate in /opt/ros/humble/setup.bash /opt/ros/foxy/setup.bash; do
    if [[ -r "${candidate}" ]]; then
      ros_setup="${candidate}"
      break
    fi
  done
fi
if [[ ! -r "${ros_setup}" ]]; then
  printf 'ROS setup not found; set ROS_SETUP=/path/to/setup.bash\n' >&2
  exit 1
fi

set +u
# shellcheck disable=SC1090
source "${ros_setup}"
set -u

if ! ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1; then
  livox_setup="${LIVOX_SETUP:-${HOME}/ws_livox/install/local_setup.bash}"
  if [[ ! -r "${livox_setup}" ]]; then
    printf 'Livox setup not found; set LIVOX_SETUP=/path/to/local_setup.bash\n' >&2
    exit 1
  fi
  set +u
  # shellcheck disable=SC1090
  source "${livox_setup}"
  set -u
fi

livox_share="$(ros2 pkg prefix --share livox_ros_driver2)"
livox_config="${LIVOX_CONFIG:-${livox_share}/config/MID360_config.json}"
if [[ ! -r "${livox_config}" ]]; then
  printf 'Livox config not found: %s\n' "${livox_config}" >&2
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-72}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

printf 'Starting physical robot %s as %s on ROS_DOMAIN_ID=%s\n' \
  "$1" "${robot_namespace}" "${ROS_DOMAIN_ID}"
printf 'LiDAR: /%s/livox/lidar  IMU: /%s/livox/imu\n' \
  "${robot_namespace}" "${robot_namespace}"

exec ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -r "__node:=livox_lidar_${robot_namespace}" \
  -r "/livox/lidar:=/${robot_namespace}/livox/lidar" \
  -r "/livox/imu:=/${robot_namespace}/livox/imu" \
  -p xfer_format:=0 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p "frame_id:=${robot_namespace}/livox_frame" \
  -p "user_config_path:=${livox_config}" \
  -p cmdline_input_bd_code:=livox0000000001
