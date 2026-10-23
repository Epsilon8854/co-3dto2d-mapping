#!/usr/bin/env bash

# Replay one Experiment 01 raw bag with a fixed, auditable ICP cloud input.
# It never edits the source bag or the normal live-launch configuration.

set -Eeuo pipefail

readonly SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
readonly REPOSITORY_DIR="$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
readonly EXPERIMENT_ID="experiment_01_ground_plane"

bag_dir=""
condition=""
run_id="$(date +%Y%m%d_%H%M%S_%N)"
domain_id="${ROS_DOMAIN_ID:-75}"
rate="1.0"
deskewing="false"
bundle_adjustment="true"

usage() {
  cat <<EOF
Usage: $(basename "$0") --bag BAG_DIRECTORY --condition raw_only|rear_only|rear_height [options]

Replay a raw Experiment 01 bag and record ICP outputs for one input condition.
  raw_only    : unfiltered driver cloud before ICP
  rear_only   : rear 120-degree sector removal before ICP
  rear_height : rear removal plus IMU gravity / fixed-height filtering before ICP

Options:
  --run-id ID       Output subdirectory name
  --domain-id ID    Isolated ROS domain (default: inherited or 75)
  --rate RATE       rosbag playback speed (default: 1.0)
  --deskewing BOOL  RTAB-Map deskewing override: true or false (default: false)
  --bundle-adjustment BOOL
                    RTAB-Map OdomF2M bundle adjustment override (default: true)
EOF
}

require_value() {
  [[ -n "${2:-}" ]] || { printf 'error: %s requires a value\n' "$1" >&2; exit 2; }
}

while (($# > 0)); do
  case "$1" in
    --bag) require_value "$1" "${2:-}"; bag_dir="$2"; shift 2 ;;
    --condition) require_value "$1" "${2:-}"; condition="$2"; shift 2 ;;
    --run-id) require_value "$1" "${2:-}"; run_id="$2"; shift 2 ;;
    --domain-id) require_value "$1" "${2:-}"; domain_id="$2"; shift 2 ;;
    --rate) require_value "$1" "${2:-}"; rate="$2"; shift 2 ;;
    --deskewing) require_value "$1" "${2:-}"; deskewing="$2"; shift 2 ;;
    --bundle-adjustment) require_value "$1" "${2:-}"; bundle_adjustment="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'error: unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n "${bag_dir}" && -r "${bag_dir}/metadata.yaml" ]] || {
  printf 'error: --bag must name a rosbag2 directory with metadata.yaml\n' >&2; exit 2;
}
[[ "${condition}" == "raw_only" || "${condition}" == "rear_only" || "${condition}" == "rear_height" ]] || {
  printf 'error: --condition must be raw_only, rear_only, or rear_height\n' >&2; exit 2;
}
[[ "${run_id}" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]] || {
  printf 'error: invalid run id\n' >&2; exit 2;
}
[[ "${domain_id}" =~ ^[0-9]+$ ]] || { printf 'error: invalid ROS domain id\n' >&2; exit 2; }
[[ "${rate}" =~ ^[0-9]+([.][0-9]+)?$ ]] && awk "BEGIN { exit !(${rate} > 0) }" || {
  printf 'error: playback rate must be positive\n' >&2; exit 2;
}
[[ "${deskewing}" == true || "${deskewing}" == false ]] || {
  printf 'error: --deskewing must be true or false\n' >&2; exit 2;
}
[[ "${bundle_adjustment}" == true || "${bundle_adjustment}" == false ]] || {
  printf 'error: --bundle-adjustment must be true or false\n' >&2; exit 2;
}

bag_dir="$(readlink -f -- "${bag_dir}")"
bag_id="$(basename -- "${bag_dir}")"
readonly run_dir="${REPOSITORY_DIR}/outputs/${EXPERIMENT_ID}/diagnostics/icp_input_ablation/${bag_id}/${condition}_${run_id}"
readonly ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
readonly mapping_setup="${MAPPING_WORKSPACE:-${REPOSITORY_DIR}}/install/local_setup.bash"
readonly pose_config="${REPOSITORY_DIR}/config/experiment/${EXPERIMENT_ID}/pose_off.yaml"
readonly odometry_config="${REPOSITORY_DIR}/config/lidar_odometry.yaml"
readonly ablation_config="${REPOSITORY_DIR}/config/experiment/${EXPERIMENT_ID}/icp_input_ablation.yaml"

[[ ! -e "${run_dir}" ]] || { printf 'error: output already exists: %s\n' "${run_dir}" >&2; exit 2; }
[[ -r "${ros_setup}" && -r "${mapping_setup}" && -r "${pose_config}" && -r "${odometry_config}" && -r "${ablation_config}" ]] || {
  printf 'error: required ROS or experiment files are missing\n' >&2; exit 1;
}

mkdir -p "${run_dir}/logs"
cp -- "${pose_config}" "${run_dir}/pose_off.yaml"
cp -- "${odometry_config}" "${run_dir}/lidar_odometry.yaml"
cp -- "${ablation_config}" "${run_dir}/icp_input_ablation.yaml"
cp -- "${SCRIPT_PATH}" "${run_dir}/run.sh"
cp -- "${REPOSITORY_DIR}/scripts/experiment/common/pre_icp_gravity_height_filter.py" "${run_dir}/pre_icp_gravity_height_filter.py"
sed -Ei "s/^(    deskewing:).*/\\1 ${deskewing}/" "${run_dir}/lidar_odometry.yaml"
sed -Ei "s/^(    OdomF2M\/BundleAdjustment:).*/\\1 \"${bundle_adjustment}\"/" "${run_dir}/lidar_odometry.yaml"

cat >"${run_dir}/run.yaml" <<EOF
experiment_id: "${EXPERIMENT_ID}"
source_bag: "${bag_dir}"
condition: "${condition}"
rear_sector_filter: {axis: "-x", angle_deg: 120.0, enabled: $([[ "${condition}" == raw_only ]] && echo false || echo true)}
pre_icp_height_filter: $([[ "${condition}" == rear_height ]] && echo true || echo false)
height_reference: "fixed_sensor_height"
sensor_height_m: 0.30
floor_relative_height_band_m: [0.30, 1.20]
gravity_aligned_sensor_band_m: [0.00, 0.90]
pose_correction: false
deskewing: ${deskewing}
bundle_adjustment: ${bundle_adjustment}
playback_rate: ${rate}
ros_domain_id: ${domain_id}
repository_commit: "$(git -C "${REPOSITORY_DIR}" rev-parse HEAD)"
EOF

export ROS_DOMAIN_ID="${domain_id}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
set +u
# shellcheck disable=SC1090
source "${ros_setup}"
# shellcheck disable=SC1090
source "${mapping_setup}"
set -u
export PYTHONPATH="${REPOSITORY_DIR}:${PYTHONPATH:-}"

declare -a process_groups=()
start_group() {
  local output_name="$1"
  shift
  setsid "$@" >"${run_dir}/logs/${output_name}.log" 2>&1 &
  process_groups+=("$!")
}
stop_groups() {
  local pid
  for pid in "${process_groups[@]:-}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
  done
  for pid in "${process_groups[@]:-}"; do
    wait "${pid}" 2>/dev/null || true
  done
}
record_pid=""
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "${record_pid}" ]]; then
    kill -INT "${record_pid}" 2>/dev/null || true
    wait "${record_pid}" 2>/dev/null || true
  fi
  stop_groups
}
trap cleanup EXIT INT TERM

readonly raw_cloud="/r0/livox/lidar"
readonly raw_imu="/r0/livox/imu"
readonly rear_cloud="/r0/mapping/lidar_rear_filtered"
readonly filtered_imu="/r0/mapping/imu_filtered"
readonly height_cloud="/r0/mapping/lidar_icp_height_filtered"
if [[ "${condition}" == rear_height ]]; then
  icp_cloud="${height_cloud}"
elif [[ "${condition}" == rear_only ]]; then
  icp_cloud="${rear_cloud}"
else
  icp_cloud="${raw_cloud}"
fi

if [[ "${condition}" != raw_only ]]; then
  start_group rear_filter ros2 run co_3dto2d_mapping pointcloud_rear_sector_filter.py --ros-args \
    -p "input_topic:=${raw_cloud}" -p "output_topic:=${rear_cloud}" -p enabled:=true \
    -p rear_filter_angle_deg:=120.0 -p rear_axis:=-x -p min_xy_range_m:=0.0 \
    -p "output_frame_id:=r0/livox_frame"
fi

start_group mapping ros2 launch "${REPOSITORY_DIR}/scripts/experiment/support/launch/single_bag_mapping.launch.py" \
  use_bag:=false use_sim_time:=true publish_tf_odom:=true publish_sensor_static_tf:=false \
  enable_rear_lidar_filter:=false imu_input_is_filtered:=false expected_update_rate:=20.0 \
  sensor_parent_frame:=r0/base_link sensor_child_frame:=r0/livox_frame local_frame_id:=r0/base_link \
  "scan_cloud_topic:=${icp_cloud}" "imu_raw_topic:=${raw_imu}" "imu_filtered_topic:=${filtered_imu}" \
  plane_filtered_cloud_topic:=/r0/mapping/plane_height_filtered \
  mapping_odometry_topic:=/r0/mapping/floor_based_odometry \
  corrected_odometry_topic:=/r0/mapping/occupancy_mapper_odometry global_frame_id:=odom \
  "odometry_config_file:=${run_dir}/lidar_odometry.yaml" "occupancy_config_file:=${run_dir}/pose_off.yaml"

if [[ "${condition}" == rear_height ]]; then
  start_group pre_icp_filter /usr/bin/python3 "${REPOSITORY_DIR}/scripts/experiment/common/pre_icp_gravity_height_filter.py" --ros-args \
    -p "input_topic:=${rear_cloud}" -p "imu_topic:=${filtered_imu}" -p "output_topic:=${height_cloud}" \
    -p sensor_height_m:=0.30 -p min_height_m:=0.30 -p max_height_m:=1.20 -p imu_timeout_sec:=0.30 \
    -p min_output_points:=20 -p "stats_csv:=${run_dir}/pre_icp_filter_stats.csv"
fi

sleep 3
record_topics=(/r0/odom /r0/odom_info_lite "${filtered_imu}")
if [[ "${condition}" == raw_only ]]; then
  record_topics+=("${raw_cloud}")
else
  record_topics+=("${rear_cloud}")
fi
if [[ "${condition}" == rear_height ]]; then
  record_topics+=("${height_cloud}")
fi
ros2 bag record -s sqlite3 -o "${run_dir}/derived" "${record_topics[@]}" \
  >"${run_dir}/logs/record.log" 2>&1 &
record_pid=$!
sleep 1

ros2 bag play --clock --rate "${rate}" "${bag_dir}" \
  --topics "${raw_cloud}" "${raw_imu}" /tf_static \
  >"${run_dir}/logs/play.log" 2>&1
sleep 3
kill -INT "${record_pid}" 2>/dev/null || true
wait "${record_pid}" 2>/dev/null || true
record_pid=""
printf 'completed_at: "%s"\n' "$(date --iso-8601=seconds)" >>"${run_dir}/run.yaml"
printf 'ICP-input ablation complete: %s\n' "${run_dir}"
