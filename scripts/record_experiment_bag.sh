#!/usr/bin/env bash

set -Eeuo pipefail

readonly SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
readonly REPOSITORY_DIR="$(cd -- "$(dirname -- "${SCRIPT_PATH}")/.." && pwd)"

experiment_id="experiment_01_ground_plane"
robot_id="0"
capture_id="$(date +%Y%m%d_%H%M%S_%N)"
motion_type=""
output_root="${REPOSITORY_DIR}/outputs"
lidar_topic=""
imu_topic=""
filtered_imu_topic=""
odom_topic=""
tf_static_topic="/tf_static"
config_ref=""
domain_id=""
storage_id="sqlite3"
mount_orientation="normal"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Record one robot's raw LiDAR/IMU and static TF into an experiment-specific folder.
Start the robot's sensor/mapping launch first so the topics and /tf_static exist.

Options:
  --experiment ID       Experiment folder name (default: ${experiment_id})
  --capture-id ID       Capture folder name (default: current timestamp)
  --motion-type TYPE    Required: static or dynamic; prefixes capture folder
  --robot-id ID         Robot namespace suffix (default: ${robot_id})
  --lidar-topic TOPIC   Override LiDAR topic (default: /r<ID>/livox/lidar)
  --imu-topic TOPIC     Override raw IMU topic (default: /r<ID>/livox/imu)
  --filtered-imu-topic T  Override filtered IMU monitor topic (default: /r<ID>/mapping/imu_filtered)
  --odom-topic TOPIC    Override odometry monitor topic (default: /r<ID>/odom)
  --tf-static-topic T   Static TF topic (default: ${tf_static_topic})
  --config-ref PATH     Config/launch profile reference saved in metadata
  --mount-orientation X Sensor orientation (default: normal)
  --output-root PATH    Output root (default: ${output_root})
  --domain-id ID        Override ROS_DOMAIN_ID (default: inherited or 72)
  --storage-id ID       rosbag2 storage (default: ${storage_id})
  -h, --help            Show this help

Output:
  OUTPUT_ROOT/EXPERIMENT/captures/MOTION_TYPE_CAPTURE_ID/MOTION_TYPE_CAPTURE_ID/
  OUTPUT_ROOT/EXPERIMENT/captures/MOTION_TYPE_CAPTURE_ID/capture.yaml
  OUTPUT_ROOT/EXPERIMENT/captures/MOTION_TYPE_CAPTURE_ID/timing_monitor.csv
  OUTPUT_ROOT/EXPERIMENT/captures/MOTION_TYPE_CAPTURE_ID/timing_monitor_summary.yaml
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
    --experiment) require_value "$1" "${2:-}"; experiment_id="$2"; shift 2 ;;
    --capture-id) require_value "$1" "${2:-}"; capture_id="$2"; shift 2 ;;
    --motion-type) require_value "$1" "${2:-}"; motion_type="$2"; shift 2 ;;
    --robot-id) require_value "$1" "${2:-}"; robot_id="$2"; shift 2 ;;
    --lidar-topic) require_value "$1" "${2:-}"; lidar_topic="$2"; shift 2 ;;
    --imu-topic) require_value "$1" "${2:-}"; imu_topic="$2"; shift 2 ;;
    --filtered-imu-topic) require_value "$1" "${2:-}"; filtered_imu_topic="$2"; shift 2 ;;
    --odom-topic) require_value "$1" "${2:-}"; odom_topic="$2"; shift 2 ;;
    --tf-static-topic) require_value "$1" "${2:-}"; tf_static_topic="$2"; shift 2 ;;
    --config-ref) require_value "$1" "${2:-}"; config_ref="$2"; shift 2 ;;
    --mount-orientation) require_value "$1" "${2:-}"; mount_orientation="$2"; shift 2 ;;
    --output-root) require_value "$1" "${2:-}"; output_root="$2"; shift 2 ;;
    --domain-id) require_value "$1" "${2:-}"; domain_id="$2"; shift 2 ;;
    --storage-id) require_value "$1" "${2:-}"; storage_id="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'error: unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

for id in "${experiment_id}" "${capture_id}"; do
  [[ "${id}" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]] || {
    printf 'error: IDs may contain letters, digits, underscore, and hyphen only: %s\n' "${id}" >&2
    exit 2
  }
done
[[ "${motion_type}" == "static" || "${motion_type}" == "dynamic" ]] || {
  printf 'error: --motion-type must be static or dynamic\n' >&2
  exit 2
}
capture_id="${motion_type}_${capture_id}"
[[ "${robot_id}" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]] || {
  printf 'error: invalid robot ID: %s\n' "${robot_id}" >&2
  exit 2
}

lidar_topic="${lidar_topic:-/r${robot_id}/livox/lidar}"
imu_topic="${imu_topic:-/r${robot_id}/livox/imu}"
filtered_imu_topic="${filtered_imu_topic:-/r${robot_id}/mapping/imu_filtered}"
odom_topic="${odom_topic:-/r${robot_id}/odom}"
for topic in "${lidar_topic}" "${imu_topic}" "${filtered_imu_topic}" "${odom_topic}" "${tf_static_topic}"; do
  [[ "${topic}" == /* ]] || {
    printf 'error: topic names must be absolute: %s\n' "${topic}" >&2
    exit 2
  }
done
[[ "${lidar_topic}" != "${imu_topic}" && "${lidar_topic}" != "${filtered_imu_topic}" && "${lidar_topic}" != "${odom_topic}" && "${imu_topic}" != "${filtered_imu_topic}" && "${imu_topic}" != "${odom_topic}" && "${filtered_imu_topic}" != "${odom_topic}" ]] || {
  printf 'error: LiDAR, raw IMU, filtered IMU, and odometry topics must be distinct\n' >&2
  exit 2
}

command -v ros2 >/dev/null || {
  printf 'error: ros2 not found; source ROS and the workspace setup first\n' >&2
  exit 1
}
if [[ -n "${domain_id}" ]]; then
  [[ "${domain_id}" =~ ^[0-9]+$ ]] || {
    printf 'error: ROS domain ID must be an integer\n' >&2
    exit 2
  }
  export ROS_DOMAIN_ID="${domain_id}"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-72}"

config_ref="${config_ref:-config/experiments/${experiment_id}/pose_off.yaml}"
if [[ "${config_ref}" = /* ]]; then
  config_source="${config_ref}"
  config_ref_for_metadata="${config_ref}"
else
  config_source="${REPOSITORY_DIR}/${config_ref}"
  config_ref_for_metadata="${config_ref}"
fi
[[ -r "${config_source}" ]] || {
  printf 'error: config reference is not readable: %s\n' "${config_source}" >&2
  exit 1
}

capture_dir="${output_root}/${experiment_id}/captures/${capture_id}"
bag_dir="${capture_dir}/${capture_id}"
[[ ! -e "${capture_dir}" ]] || {
  printf 'error: capture already exists: %s\n' "${capture_dir}" >&2
  exit 1
}
mkdir -p -- "${capture_dir}"

metadata_file="${capture_dir}/capture.yaml"
config_snapshot="${capture_dir}/config.yaml"
cp -- "${config_source}" "${config_snapshot}"
config_sha256="$(sha256sum -- "${config_snapshot}" | cut -d ' ' -f 1)"

yaml_quote() {
  local value="$1"
  value="${value//\\/\\\\}"
  value="${value//\"/\\\"}"
  printf '"%s"' "${value}"
}

cat >"${metadata_file}" <<EOF
experiment_id: $(yaml_quote "${experiment_id}")
capture_id: $(yaml_quote "${capture_id}")
motion_type: $(yaml_quote "${motion_type}")
recorded_at: $(yaml_quote "$(date --iso-8601=seconds)")
repository_branch: $(yaml_quote "$(git -C "${REPOSITORY_DIR}" branch --show-current 2>/dev/null || true)")
repository_commit: $(yaml_quote "$(git -C "${REPOSITORY_DIR}" rev-parse HEAD 2>/dev/null || true)")
robot_id: $(yaml_quote "${robot_id}")
ros_domain_id: $(yaml_quote "${ROS_DOMAIN_ID}")
storage_id: $(yaml_quote "${storage_id}")
topics:
  lidar: $(yaml_quote "${lidar_topic}")
  imu_raw: $(yaml_quote "${imu_topic}")
  imu_filtered_monitor: $(yaml_quote "${filtered_imu_topic}")
  odom_monitor: $(yaml_quote "${odom_topic}")
  tf_static: $(yaml_quote "${tf_static_topic}")
config_ref: $(yaml_quote "${config_ref_for_metadata}")
config_snapshot: "config.yaml"
config_sha256: $(yaml_quote "${config_sha256}")
bag_path: $(yaml_quote "${bag_dir}")
mount_orientation: $(yaml_quote "${mount_orientation}")
tf_static_recorded_in_bag: true
timing_monitor:
  csv: "timing_monitor.csv"
  summary: "timing_monitor_summary.yaml"
  header_gap_threshold_ms:
    imu: 20
    lidar: 250
    odom: 500
EOF

printf 'Waiting for all requested topics on ROS_DOMAIN_ID=%s' "${ROS_DOMAIN_ID}"
for attempt in {1..30}; do
  topic_list="$(ros2 topic list 2>/dev/null || true)"
  if grep -Fxq -- "${lidar_topic}" <<<"${topic_list}" &&
      grep -Fxq -- "${imu_topic}" <<<"${topic_list}" &&
      grep -Fxq -- "${tf_static_topic}" <<<"${topic_list}"; then
    printf '\n'
    break
  fi
  if ((attempt == 30)); then
    printf '\nerror: topics not discovered; check the live ROS domain and topic names\n' >&2
    rm -f -- "${metadata_file}"
    rmdir -- "${capture_dir}" 2>/dev/null || true
    exit 1
  fi
  printf '.'
  sleep 1
done

printf 'Experiment: %s\nCapture: %s\nOutput: %s\nTopics: %s %s %s\n' \
  "${experiment_id}" "${capture_id}" "${bag_dir}" \
  "${lidar_topic}" "${imu_topic}" "${tf_static_topic}"
printf 'Press Ctrl-C after completing the route to stop recording cleanly.\n'

timing_monitor_csv="${capture_dir}/timing_monitor.csv"
timing_monitor_summary="${capture_dir}/timing_monitor_summary.yaml"
python3 "${REPOSITORY_DIR}/scripts/monitor_experiment_timing.py" \
  --output-csv "${timing_monitor_csv}" \
  --summary "${timing_monitor_summary}" \
  --raw-imu-topic "${imu_topic}" \
  --filtered-imu-topic "${filtered_imu_topic}" \
  --lidar-topic "${lidar_topic}" \
  --odom-topic "${odom_topic}" \
  --imu-gap-threshold-ms 20 \
  --lidar-gap-threshold-ms 250 \
  --odom-gap-threshold-ms 500 &
timing_monitor_pid=$!

cleanup() {
  trap - EXIT INT TERM
  if kill -0 "${timing_monitor_pid}" 2>/dev/null; then
    kill -TERM "${timing_monitor_pid}" 2>/dev/null || true
  fi
  wait "${timing_monitor_pid}" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

ros2 bag record -s "${storage_id}" -o "${bag_dir}" \
  "${lidar_topic}" "${imu_topic}" "${tf_static_topic}"
