#!/usr/bin/env bash

set -Eeuo pipefail

readonly SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
readonly REPOSITORY_DIR="$(cd -- "$(dirname -- "${SCRIPT_PATH}")/.." && pwd)"
readonly EXPERIMENT_ID="experiment_01_ground_plane"
readonly RVIZ_CONFIG="${REPOSITORY_DIR}/rviz/${EXPERIMENT_ID}_replay.rviz"

pose_correction="fixed_height"
run_id="$(date +%Y%m%d_%H%M%S_%N)"
domain_id="${ROS_DOMAIN_ID:-72}"
rate="1.0"
bag_dir=""
loop_playback="false"

usage() {
  cat <<EOF
Usage: $(basename "$0") --bag BAG_DIRECTORY [options]

Replay an existing Experiment 01 bag, regenerate the IMU/odometry/gravity-height
outputs, and open the matching RViz view. This does not start the LiDAR driver
or record a new bag.

Options:
  --bag DIR                  rosbag2 directory containing metadata.yaml
  --pose-correction off|on|fixed_height   Select the floor z/roll/pitch mode (default: fixed_height)
  --run-id ID                Name this replay output folder (default: timestamp)
  --domain-id ID             ROS domain (default: inherited or 72)
  --rate RATE                Bag playback rate (default: 1.0)
  --loop                     Repeat indefinitely, restarting mapping each pass
  -h, --help                 Show this help

Outputs:
  outputs/${EXPERIMENT_ID}/runs/RUN_ID/config.yaml
  outputs/${EXPERIMENT_ID}/runs/RUN_ID/run.yaml
  outputs/${EXPERIMENT_ID}/runs/RUN_ID/logs/

After playback ends, RViz and the mapping nodes stay open for inspection.
Press Ctrl-C to stop them.
EOF
}

require_value() {
  [[ -n "${2:-}" ]] || {
    printf 'error: %s requires a value\n' "$1" >&2
    exit 2
  }
}

while (($# > 0)); do
  case "$1" in
    --bag) require_value "$1" "${2:-}"; bag_dir="$2"; shift 2 ;;
    --pose-correction) require_value "$1" "${2:-}"; pose_correction="$2"; shift 2 ;;
    --run-id) require_value "$1" "${2:-}"; run_id="$2"; shift 2 ;;
    --domain-id) require_value "$1" "${2:-}"; domain_id="$2"; shift 2 ;;
    --rate) require_value "$1" "${2:-}"; rate="$2"; shift 2 ;;
    --loop) loop_playback="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'error: unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n "${bag_dir}" ]] || { printf 'error: --bag is required\n' >&2; usage >&2; exit 2; }
bag_dir="$(readlink -f -- "${bag_dir}")"
[[ -r "${bag_dir}/metadata.yaml" ]] || {
  printf 'error: rosbag2 metadata.yaml not found in: %s\n' "${bag_dir}" >&2
  exit 1
}
[[ "${pose_correction}" == "off" || "${pose_correction}" == "on" || "${pose_correction}" == "fixed_height" ]] || {
  printf 'error: --pose-correction must be off, on, or fixed_height\n' >&2; exit 2;
}
[[ "${run_id}" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]] || {
  printf 'error: run ID may contain letters, digits, underscore, and hyphen only\n' >&2; exit 2;
}
[[ "${domain_id}" =~ ^[0-9]+$ ]] || {
  printf 'error: ROS domain ID must be an integer\n' >&2; exit 2;
}
[[ "${rate}" =~ ^[0-9]+([.][0-9]+)?$ ]] && awk "BEGIN { exit !(${rate} > 0) }" || {
  printf 'error: playback rate must be a positive number\n' >&2; exit 2;
}

if [[ "${pose_correction}" == "fixed_height" ]]; then
  readonly CONFIG_FILE="${REPOSITORY_DIR}/config/experiments/${EXPERIMENT_ID}/fixed_height_imu_gravity.yaml"
else
  readonly CONFIG_FILE="${REPOSITORY_DIR}/config/experiments/${EXPERIMENT_ID}/pose_${pose_correction}.yaml"
fi
readonly ODOMETRY_CONFIG="${REPOSITORY_DIR}/config/lidar_odometry.yaml"
readonly ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
readonly MAPPING_WORKSPACE="${MAPPING_WORKSPACE:-${REPOSITORY_DIR}}"
readonly RUN_DIR="${REPOSITORY_DIR}/outputs/${EXPERIMENT_ID}/runs/${run_id}"

[[ -r "${CONFIG_FILE}" && -r "${ODOMETRY_CONFIG}" ]] || { printf 'error: experiment config is missing\n' >&2; exit 1; }
[[ -r "${RVIZ_CONFIG}" ]] || { printf 'error: RViz config not found: %s\n' "${RVIZ_CONFIG}" >&2; exit 1; }
[[ ! -e "${RUN_DIR}" ]] || { printf 'error: run directory already exists: %s\n' "${RUN_DIR}" >&2; exit 1; }
[[ -r "${ROS_SETUP}" && -r "${MAPPING_WORKSPACE}/install/local_setup.bash" ]] || {
  printf 'error: source ROS and build the workspace before replay\n' >&2; exit 1;
}

if [[ "${pose_correction}" == "on" ]]; then
  height_reference_mode="estimated_plane"
  lidar_mount_height_m=""
  fixed_base_link_height_m=""
  height_filter_min_m="0.20"
  height_filter_max_m="1.20"
  gravity_band_min_m=""
  gravity_band_max_m=""
else
  height_reference_mode="fixed_sensor_height"
  lidar_mount_height_m="0.30"
  fixed_base_link_height_m=$([[ "${pose_correction}" == "fixed_height" ]] && echo "0.30" || echo "")
  height_filter_min_m="0.30"
  height_filter_max_m="1.20"
  gravity_band_min_m="0.00"
  gravity_band_max_m="0.90"
fi

mkdir -p -- "${RUN_DIR}/logs"
cp -- "${CONFIG_FILE}" "${RUN_DIR}/config.yaml"
cp -- "${ODOMETRY_CONFIG}" "${RUN_DIR}/lidar_odometry.yaml"
export ROS_DOMAIN_ID="${domain_id}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${MAPPING_WORKSPACE}/install/local_setup.bash"
set -u

cat >"${RUN_DIR}/run.yaml" <<EOF
experiment_id: "${EXPERIMENT_ID}"
run_id: "${run_id}"
started_at: "$(date --iso-8601=seconds)"
source_bag: "${bag_dir}"
pose_correction: "${pose_correction}"
height_filter_enabled: true
height_reference_mode: "${height_reference_mode}"
lidar_mount_height_above_floor_m: ${lidar_mount_height_m:-null}
fixed_base_link_height_above_floor_m: ${fixed_base_link_height_m:-null}
height_filter_min_m: ${height_filter_min_m}
height_filter_max_m: ${height_filter_max_m}
gravity_aligned_sensor_band_min_m: ${gravity_band_min_m:-null}
gravity_aligned_sensor_band_max_m: ${gravity_band_max_m:-null}
rear_sector_filter_enabled: false
sensor_tf_roll_rad: 0.0
robot_namespace: "r0"
ros_domain_id: "${domain_id}"
playback_rate: ${rate}
loop_playback: ${loop_playback}
restart_mapping_each_loop: ${loop_playback}
config_snapshot: "config.yaml"
odometry_config_snapshot: "lidar_odometry.yaml"
rviz_config: "rviz/${EXPERIMENT_ID}_replay.rviz"
rviz_use_sim_time: true
repository_branch: "$(git -C "${REPOSITORY_DIR}" branch --show-current 2>/dev/null || true)"
repository_commit: "$(git -C "${REPOSITORY_DIR}" rev-parse HEAD 2>/dev/null || true)"
EOF

cd -- "${REPOSITORY_DIR}"
launch_pid=""
rviz_pid=""

stop_mapping() {
  if [[ -n "${launch_pid}" ]]; then
    kill -TERM -- "-${launch_pid}" 2>/dev/null || kill -TERM "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
    launch_pid=""
  fi
}

cleanup() {
  trap - INT TERM EXIT
  if [[ -n "${rviz_pid}" ]] && kill -0 "${rviz_pid}" 2>/dev/null; then
    kill -TERM -- "-${rviz_pid}" 2>/dev/null || true
    wait "${rviz_pid}" 2>/dev/null || true
  fi
  stop_mapping
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_mapping() {
  local cycle="$1"
  local cycle_label
  printf -v cycle_label '%04d' "${cycle}"
  printf 'Starting mapping for replay pass %s\n' "${cycle_label}"
  setsid ros2 launch co_3dto2d_mapping single_bag_mapping.launch.py \
    use_bag:=false use_sim_time:=true publish_tf_odom:=true publish_sensor_static_tf:=false \
    enable_rear_lidar_filter:=false imu_input_is_filtered:=false expected_update_rate:=20.0 \
    sensor_parent_frame:=r0/base_link sensor_child_frame:=r0/livox_frame local_frame_id:=r0/base_link \
    scan_cloud_topic:=/r0/livox/lidar imu_raw_topic:=/r0/livox/imu imu_filtered_topic:=/r0/livox/imu_filtered \
    plane_filtered_cloud_topic:=/r0/mapping/plane_height_filtered mapping_odometry_topic:=/r0/mapping/floor_based_odometry \
    corrected_odometry_topic:=/r0/mapping/occupancy_mapper_odometry global_frame_id:=odom \
    odometry_config_file:="${RUN_DIR}/lidar_odometry.yaml" occupancy_config_file:="${RUN_DIR}/config.yaml" \
    >"${RUN_DIR}/logs/launch_cycle_${cycle_label}.log" 2>&1 &
  launch_pid=$!
  sleep 3
}

start_mapping 1
setsid rviz2 -d "${RVIZ_CONFIG}" --ros-args -p use_sim_time:=true \
  >"${RUN_DIR}/logs/rviz.log" 2>&1 &
rviz_pid=$!
printf 'Experiment: %s\nRun: %s\nPose correction: %s\nBag: %s\nOutput: %s\n' \
  "${EXPERIMENT_ID}" "${run_id}" "${pose_correction}" "${bag_dir}" "${RUN_DIR}"

cycle=1
while true; do
  printf '\nStarting bag replay pass %04d\n' "${cycle}" | tee -a "${RUN_DIR}/logs/bag_play.log"
  ros2 bag play --clock --rate "${rate}" "${bag_dir}" \
    --topics /r0/livox/lidar /r0/livox/imu /tf_static \
    2>&1 | tee -a "${RUN_DIR}/logs/bag_play.log"

  if [[ "${loop_playback}" != "true" ]]; then
    printf '\nPlayback finished. RViz and mapping remain open; press Ctrl-C to stop.\n'
    wait "${launch_pid}"
    break
  fi

  printf 'Replay pass %04d complete; restarting mapping before the next timestamp sequence.\n' \
    "${cycle}" | tee -a "${RUN_DIR}/logs/bag_play.log"
  stop_mapping
  cycle=$((cycle + 1))
  sleep 1
  start_mapping "${cycle}"
done
