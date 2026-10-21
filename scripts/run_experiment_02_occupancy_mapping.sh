#!/usr/bin/env bash

set -Eeuo pipefail

readonly SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
readonly REPOSITORY_DIR="$(cd -- "$(dirname -- "${SCRIPT_PATH}")/.." && pwd)"
readonly BASE_RUNNER="${SCRIPT_PATH%/*}/run_mapping_r0.sh"
readonly EXPERIMENT_ID="experiment_02_occupancy_mapping"
readonly RVIZ_CONFIG="${REPOSITORY_DIR}/rviz/${EXPERIMENT_ID}.rviz"

pose_correction="fixed_height"
run_id="$(date +%Y%m%d_%H%M%S_%N)"
domain_id="${ROS_DOMAIN_ID:-72}"
runner_args=()

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Start the r0 Livox driver and this experiment's single-robot mapping pipeline.
The rear-sector filter is enabled for inspection. The default profile uses a
fixed 0.30 m sensor height and IMU gravity to keep the 0.30–1.20 m
floor-relative obstacle band. It fixes base_link z and aligns roll/pitch to IMU gravity; no cloud plane is fitted.

Options:
  --pose-correction off|on|fixed_height   Select the z/roll/pitch mode (default: fixed_height)
  --run-id ID                Name this execution (default: timestamp)
  --domain-id ID             ROS domain (default: inherited or 72)
  --livox-workspace DIR      Forward Livox workspace path to the base runner
  --driver-config FILE       Forward MID-360 driver config path
  --odometry-config FILE     Forward LiDAR odometry config path
  -h, --help                 Show this help

Outputs:
  outputs/${EXPERIMENT_ID}/runs/RUN_ID/config.yaml
  outputs/${EXPERIMENT_ID}/runs/RUN_ID/run.yaml
  outputs/${EXPERIMENT_ID}/runs/RUN_ID/maps/

The experiment RViz configuration starts automatically and shows raw and
gravity-height-filtered point clouds. Press Ctrl-C to stop mapping and RViz.
EOF
}

require_value() {
  local option="$1"
  local value="${2:-}"
  [[ -n "${value}" ]] || {
    printf 'error: %s requires a value\n' "${option}" >&2
    exit 2
  }
}

while (($# > 0)); do
  case "$1" in
    --pose-correction)
      require_value "$1" "${2:-}"
      pose_correction="$2"
      shift 2
      ;;
    --run-id)
      require_value "$1" "${2:-}"
      run_id="$2"
      shift 2
      ;;
    --domain-id)
      require_value "$1" "${2:-}"
      domain_id="$2"
      shift 2
      ;;
    --livox-workspace|--driver-config|--odometry-config)
      require_value "$1" "${2:-}"
      runner_args+=("$1" "$2")
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'error: unknown option: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ "${pose_correction}" == "off" || "${pose_correction}" == "on" || "${pose_correction}" == "fixed_height" ]] || {
  printf 'error: --pose-correction must be off, on, or fixed_height\n' >&2
  exit 2
}
[[ "${run_id}" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]] || {
  printf 'error: run ID may contain letters, digits, underscore, and hyphen only\n' >&2
  exit 2
}
[[ "${domain_id}" =~ ^[0-9]+$ ]] || {
  printf 'error: ROS domain ID must be an integer\n' >&2
  exit 2
}
[[ -x "${BASE_RUNNER}" ]] || {
  printf 'error: base runner not found or not executable: %s\n' "${BASE_RUNNER}" >&2
  exit 1
}
[[ -r "${RVIZ_CONFIG}" ]] || {
  printf 'error: experiment RViz config not found: %s\n' "${RVIZ_CONFIG}" >&2
  exit 1
}

if [[ "${pose_correction}" == "fixed_height" ]]; then
  config_file="${REPOSITORY_DIR}/config/experiments/${EXPERIMENT_ID}/live_pht_history_5_full_se2.yaml"
else
  config_file="${REPOSITORY_DIR}/config/experiments/${EXPERIMENT_ID}/pose_${pose_correction}.yaml"
fi
[[ -r "${config_file}" ]] || {
  printf 'error: experiment config not found: %s\n' "${config_file}" >&2
  exit 1
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

run_dir="${REPOSITORY_DIR}/outputs/${EXPERIMENT_ID}/runs/${run_id}"
[[ ! -e "${run_dir}" ]] || {
  printf 'error: run directory already exists: %s\n' "${run_dir}" >&2
  exit 1
}
mkdir -p -- "${run_dir}/maps"
cp -- "${config_file}" "${run_dir}/config.yaml"

export ROS_DOMAIN_ID="${domain_id}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export MAPPING_WORKSPACE="${MAPPING_WORKSPACE:-${REPOSITORY_DIR}}"
export MAPPING_CONFIG="${config_file}"
export OCCUPANCY_PNG_OUTPUT_DIR="${run_dir}/maps"
export ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

[[ -r "${ROS_SETUP}" ]] || {
  printf 'error: ROS setup not found: %s\n' "${ROS_SETUP}" >&2
  exit 1
}
[[ -r "${MAPPING_WORKSPACE}/install/local_setup.bash" ]] || {
  printf 'error: mapping workspace setup not found: %s\n' \
    "${MAPPING_WORKSPACE}/install/local_setup.bash" >&2
  exit 1
}
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${MAPPING_WORKSPACE}/install/local_setup.bash"
set -u
command -v rviz2 >/dev/null || {
  printf 'error: rviz2 is unavailable after sourcing ROS and the workspace\n' >&2
  exit 1
}

cat >"${run_dir}/run.yaml" <<EOF
experiment_id: "${EXPERIMENT_ID}"
run_id: "${run_id}"
started_at: "$(date --iso-8601=seconds)"
pose_correction: "${pose_correction}"
icp_topic_queue_size: 1
local_pht_submap_pose_graph_mode: "full_se2"
pht_history_keyframes: 5
height_filter_enabled: true
height_reference_mode: "${height_reference_mode}"
lidar_mount_height_above_floor_m: ${lidar_mount_height_m:-null}
fixed_base_link_height_above_floor_m: ${fixed_base_link_height_m:-null}
height_filter_min_m: ${height_filter_min_m}
height_filter_max_m: ${height_filter_max_m}
gravity_aligned_sensor_band_min_m: ${gravity_band_min_m:-null}
gravity_aligned_sensor_band_max_m: ${gravity_band_max_m:-null}
rear_sector_filter_enabled: true
sensor_tf_roll_rad: 0.0
robot_namespace: "r0"
ros_domain_id: "${domain_id}"
config_profile: "${config_file#${REPOSITORY_DIR}/}"
rviz_config: "rviz/${EXPERIMENT_ID}.rviz"
repository_branch: "$(git -C "${REPOSITORY_DIR}" branch --show-current 2>/dev/null || true)"
repository_commit: "$(git -C "${REPOSITORY_DIR}" rev-parse HEAD 2>/dev/null || true)"
EOF

printf 'Experiment: %s\nRun: %s\nPose correction: %s\nConfig: %s\nOutput: %s\n' \
  "${EXPERIMENT_ID}" "${run_id}" "${pose_correction}" "${config_file}" "${run_dir}"

rviz2 -d "${RVIZ_CONFIG}" &
rviz_pid=$!
cleanup() {
  trap - INT TERM EXIT
  if kill -0 "${rviz_pid}" 2>/dev/null; then
    kill -TERM "${rviz_pid}" 2>/dev/null || true
  fi
  wait "${rviz_pid}" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

bash "${BASE_RUNNER}" \
  --domain-id "${domain_id}" \
  --launch-arg publish_tf_odom:=true \
  --launch-arg icp_topic_queue_size:=1 \
  --launch-arg sensor_tf_roll_0:=0.0 \
  --launch-arg enable_rear_lidar_filter:=true \
  "${runner_args[@]}"
