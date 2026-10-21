#!/usr/bin/env bash

# Replay one Experiment 3 capture twice with identical mapping settings except
# for the single temporal dynamic-object filter switch.
set -Eeuo pipefail

readonly SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
readonly REPOSITORY_DIR="$(cd -- "$(dirname -- "${SCRIPT_PATH}")/.." && pwd)"
readonly EXPERIMENT_ID="experiment_03_dynamic_object_filtering"
readonly RVIZ_CONFIG="${REPOSITORY_DIR}/rviz/${EXPERIMENT_ID}_replay.rviz"

dynamic_filter="off"
run_id="$(date +%Y%m%d_%H%M%S_%N)"
domain_id="${ROS_DOMAIN_ID:-72}"
rate="1.0"
bag_dir=""
no_rviz="false"
loop_playback="false"

usage() {
  cat <<USAGE
Usage: $(basename "$0") --bag BAG_DIRECTORY [options]

Replay one dynamic-scene bag with the Experiment 3 baseline. The off/on runs
use the same fixed-height pose, rear sector filter, RTAB-Map queue, and PHT
history settings. Only dynamic_filter_enabled changes.

Options:
  --bag DIR                  rosbag2 directory containing metadata.yaml
  --dynamic-filter off|on    Temporal dynamic-object filter state (default: off)
  --run-id ID                Output folder name (default: timestamp)
  --domain-id ID             ROS domain (default: inherited or 72)
  --rate RATE                rosbag playback rate (default: 1.0)
  --loop                     Repeat forever and restart mapping per pass
  --no-rviz                  Do not open RViz
  -h, --help                 Show this help

Output:
  outputs/${EXPERIMENT_ID}/runs/RUN_ID/topdown/topdown_global_map_raw_cyan_corrected_orange.png
USAGE
}

require_value() {
  [[ -n "${2:-}" ]] || { printf 'error: %s requires a value\n' "$1" >&2; exit 2; }
}

while (($# > 0)); do
  case "$1" in
    --bag) require_value "$1" "${2:-}"; bag_dir="$2"; shift 2 ;;
    --dynamic-filter) require_value "$1" "${2:-}"; dynamic_filter="$2"; shift 2 ;;
    --run-id) require_value "$1" "${2:-}"; run_id="$2"; shift 2 ;;
    --domain-id) require_value "$1" "${2:-}"; domain_id="$2"; shift 2 ;;
    --rate) require_value "$1" "${2:-}"; rate="$2"; shift 2 ;;
    --loop) loop_playback="true"; shift ;;
    --no-rviz) no_rviz="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'error: unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n "${bag_dir}" ]] || { printf 'error: --bag is required\n' >&2; usage >&2; exit 2; }
[[ "${dynamic_filter}" == "off" || "${dynamic_filter}" == "on" ]] || {
  printf 'error: --dynamic-filter must be off or on\n' >&2; exit 2;
}
[[ "${run_id}" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]] || {
  printf 'error: run ID may contain letters, digits, underscore, and hyphen only\n' >&2; exit 2;
}
[[ "${domain_id}" =~ ^[0-9]+$ ]] || { printf 'error: --domain-id must be an integer\n' >&2; exit 2; }
[[ "${rate}" =~ ^[0-9]+([.][0-9]+)?$ ]] && awk "BEGIN { exit !(${rate} > 0) }" || {
  printf 'error: --rate must be positive\n' >&2; exit 2;
}

bag_dir="$(readlink -f -- "${bag_dir}")"
[[ -r "${bag_dir}/metadata.yaml" ]] || {
  printf 'error: rosbag2 metadata.yaml not found: %s\n' "${bag_dir}" >&2; exit 1;
}

case "${dynamic_filter}" in
  off) profile_name="dynamic_filter_off.yaml" ;;
  on) profile_name="dynamic_filter_on.yaml" ;;
esac
readonly CONFIG_FILE="${REPOSITORY_DIR}/config/experiments/${EXPERIMENT_ID}/${profile_name}"
readonly ODOMETRY_CONFIG="${REPOSITORY_DIR}/config/lidar_odometry.yaml"
readonly RUN_DIR="${REPOSITORY_DIR}/outputs/${EXPERIMENT_ID}/runs/${run_id}"
readonly EXPECTED_PNG="${RUN_DIR}/topdown/topdown_global_map_raw_cyan_corrected_orange.png"
[[ -r "${CONFIG_FILE}" && -r "${ODOMETRY_CONFIG}" && -r "${RVIZ_CONFIG}" ]] || {
  printf 'error: Experiment 3 config or RViz setting is missing\n' >&2; exit 1;
}
[[ ! -e "${RUN_DIR}" ]] || { printf 'error: run directory exists: %s\n' "${RUN_DIR}" >&2; exit 1; }

mkdir -p "${RUN_DIR}/logs"
cp -- "${CONFIG_FILE}" "${RUN_DIR}/config.yaml"
cp -- "${ODOMETRY_CONFIG}" "${RUN_DIR}/lidar_odometry.yaml"

export ROS_DOMAIN_ID="${domain_id}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
set +u
# shellcheck disable=SC1091
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
# shellcheck disable=SC1091
source "${MAPPING_WORKSPACE:-${REPOSITORY_DIR}}/install/local_setup.bash"
set -u

cat > "${RUN_DIR}/run.yaml" <<RUN_METADATA
experiment_id: "${EXPERIMENT_ID}"
run_id: "${run_id}"
source_bag: "${bag_dir}"
dynamic_filter_enabled: ${dynamic_filter}
config_profile: "${CONFIG_FILE#${REPOSITORY_DIR}/}"
pose_correction: "fixed_height"
icp_topic_queue_size: 1
local_pht_submap_pose_graph_mode: "full_se2"
pht_history_keyframes: 5
rear_sector_filter_enabled: true
playback_rate: ${rate}
loop_playback: ${loop_playback}
ros_domain_id: ${domain_id}
expected_topdown_png: "${EXPECTED_PNG}"
RUN_METADATA

launch_pid=""
exporter_pid=""
rviz_pid=""
stop_process() {
  local pid="$1" signal="$2"
  [[ -n "${pid}" ]] || return 0
  kill "-${signal}" -- "-${pid}" 2>/dev/null || kill "-${signal}" "${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}
cleanup() {
  trap - EXIT INT TERM
  stop_process "${rviz_pid}" TERM
  stop_process "${exporter_pid}" INT
  stop_process "${launch_pid}" TERM
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_mapping() {
  local cycle="$1"
  setsid ros2 launch co_3dto2d_mapping single_bag_mapping.launch.py \
    use_bag:=false use_sim_time:=true publish_tf_odom:=true publish_sensor_static_tf:=false \
    enable_rear_lidar_filter:=true imu_input_is_filtered:=false expected_update_rate:=20.0 \
    icp_topic_queue_size:=1 sensor_parent_frame:=r0/base_link sensor_child_frame:=r0/livox_frame \
    local_frame_id:=r0/base_link bag_lidar_topic:=/r0/livox/lidar \
    scan_cloud_topic:=/r0/mapping/lidar_rear_filtered icp_scan_cloud_topic:=/r0/livox/lidar \
    imu_raw_topic:=/r0/livox/imu imu_filtered_topic:=/r0/livox/imu_filtered \
    plane_filtered_cloud_topic:=/r0/mapping/plane_height_filtered \
    mapping_odometry_topic:=/r0/mapping/floor_based_odometry \
    corrected_odometry_topic:=/r0/mapping/occupancy_mapper_odometry \
    global_frame_id:=r0/odom local_pht_submap_pose_graph_mode:=full_se2 \
    odometry_config_file:="${RUN_DIR}/lidar_odometry.yaml" occupancy_config_file:="${RUN_DIR}/config.yaml" \
    >"${RUN_DIR}/logs/mapping_cycle_$(printf '%03d' "${cycle}").log" 2>&1 &
  launch_pid=$!
  sleep 3
}

start_topdown_exporter() {
  local cycle="$1"
  setsid python3 "${REPOSITORY_DIR}/scripts/export_experiment_02_topdown.py" --ros-args \
    -p output_directory:="${RUN_DIR}/topdown" \
    >"${RUN_DIR}/logs/topdown_exporter_cycle_$(printf '%03d' "${cycle}").log" 2>&1 &
  exporter_pid=$!
  sleep 1
}

cycle=1
start_mapping "${cycle}"
start_topdown_exporter "${cycle}"

if [[ "${no_rviz}" != "true" ]]; then
  setsid rviz2 -d "${RVIZ_CONFIG}" --ros-args -p use_sim_time:=true \
    >"${RUN_DIR}/logs/rviz.log" 2>&1 &
  rviz_pid=$!
fi

printf 'Experiment 3 replay: filter=%s, bag=%s, output=%s, loop=%s\n' \
  "${dynamic_filter}" "${bag_dir}" "${RUN_DIR}" "${loop_playback}"
while true; do
  printf 'Starting replay cycle %d\n' "${cycle}"
  ros2 bag play --clock --rate "${rate}" "${bag_dir}" \
    --topics /r0/livox/lidar /r0/livox/imu /tf_static \
    2>&1 | tee "${RUN_DIR}/logs/bag_play_cycle_$(printf '%03d' "${cycle}").log"

  # Let synchronized mapping callbacks publish the final grid before export.
  sleep 2
  stop_process "${exporter_pid}" INT
  exporter_pid=""
  [[ -s "${EXPECTED_PNG}" ]] || {
    printf 'error: final top-down PNG was not written: %s\n' "${EXPECTED_PNG}" >&2; exit 1;
  }
  printf 'Saved top-down comparison input: %s\n' "${EXPECTED_PNG}"

  if [[ "${loop_playback}" != "true" ]]; then
    break
  fi

  # /clock returns to the bag start on the next pass. Recreate the mapping
  # state so that its synchronizers, paths, and temporal evidence do too.
  stop_process "${launch_pid}" TERM
  launch_pid=""
  cycle=$((cycle + 1))
  start_mapping "${cycle}"
  start_topdown_exporter "${cycle}"
done
