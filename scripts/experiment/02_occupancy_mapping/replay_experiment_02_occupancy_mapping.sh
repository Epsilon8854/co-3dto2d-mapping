#!/usr/bin/env bash

set -Eeuo pipefail

readonly SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
readonly REPOSITORY_DIR="$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
readonly EXPERIMENT_ID="experiment_02_occupancy_mapping"
readonly RVIZ_CONFIG="${REPOSITORY_DIR}/rviz/experiment/${EXPERIMENT_ID}_replay.rviz"

pose_correction="fixed_height"
local_window_icp="off"
local_submap_registration="off"
local_pose_graph="off"
local_line_pose_graph="off"
local_pht_submap_pose_graph="off"
pht_profile="length_1_5"
pht_history_keyframes="10"
icp_topic_queue_size="1"
run_id="$(date +%Y%m%d_%H%M%S_%N)"
domain_id="${ROS_DOMAIN_ID:-72}"
rate="1.0"
bag_dir=""
dataset=""
loop_playback="false"
no_rviz="false"
exit_after_playback="false"
canonical_mapper_input="false"
record_canonical_mapper_input_bag=""

usage() {
  cat <<EOF
Usage: $(basename "$0") (--bag BAG_DIRECTORY | --dataset airoom|hallway) [options]

Replay an existing Experiment 2 bag, regenerate the IMU/odometry/gravity-height
outputs, and open the matching RViz view. This does not start the LiDAR driver
or record a new bag.

Options:
  --bag DIR                  rosbag2 directory containing metadata.yaml
  --dataset airoom|hallway   Use an Experiment 2 dataset alias
  --pose-correction off|on|fixed_height   Select the floor z/roll/pitch mode (default: fixed_height)
  --local-window-icp off|on              Enable recent-scan x/y/yaw ICP for fixed-height replay (default: off)
  --local-submap-registration off|on      Enable occupancy local-submap x/y/yaw correction (default: off)
  --local-pose-graph off|on               Enable local raw-odometry + scan-factor pose graph (default: off)
  --local-line-pose-graph off|on          Enable local line-factor pose graph (default: off)
  --local-pht-submap-pose-graph visualization_only|single_line|full_se2
                              Enable cumulative PHT submap graph mode (default: off)
  --pht-profile length_1_5|length_0_7
                              Select PHT minimum-line-length profile (default: length_1_5)
  --pht-history-keyframes COUNT
                              Number of prior PHT keyframes in the cumulative target (default: 10)
  --icp-topic-queue-size COUNT
                              RTAB-Map ICP scan subscription queue (default: 1)
  --run-id ID                Name this replay output folder (default: timestamp)
  --domain-id ID             ROS domain (default: inherited or 72)
  --rate RATE                Bag playback rate (default: 1.0)
  --loop                     Repeat indefinitely, restarting mapping each pass
  --no-rviz                  Do not start RViz (useful for batch replay)
  --exit-after-playback      Stop mapping after a non-loop replay completes
  --canonical-mapper-input   Replay a derived bag's frozen mapper inputs only
  --record-canonical-mapper-input-bag DIR
                            Record direct ICP, filtered cloud, and mapping odometry
                            into DIR while this normal replay runs
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
    --dataset) require_value "$1" "${2:-}"; dataset="$2"; shift 2 ;;
    --pose-correction) require_value "$1" "${2:-}"; pose_correction="$2"; shift 2 ;;
    --local-window-icp) require_value "$1" "${2:-}"; local_window_icp="$2"; shift 2 ;;
    --local-submap-registration) require_value "$1" "${2:-}"; local_submap_registration="$2"; shift 2 ;;
    --local-pose-graph) require_value "$1" "${2:-}"; local_pose_graph="$2"; shift 2 ;;
    --local-line-pose-graph) require_value "$1" "${2:-}"; local_line_pose_graph="$2"; shift 2 ;;
    --local-pht-submap-pose-graph) require_value "$1" "${2:-}"; local_pht_submap_pose_graph="$2"; shift 2 ;;
    --pht-profile) require_value "$1" "${2:-}"; pht_profile="$2"; shift 2 ;;
    --pht-history-keyframes) require_value "$1" "${2:-}"; pht_history_keyframes="$2"; shift 2 ;;
    --icp-topic-queue-size) require_value "$1" "${2:-}"; icp_topic_queue_size="$2"; shift 2 ;;
    --run-id) require_value "$1" "${2:-}"; run_id="$2"; shift 2 ;;
    --domain-id) require_value "$1" "${2:-}"; domain_id="$2"; shift 2 ;;
    --rate) require_value "$1" "${2:-}"; rate="$2"; shift 2 ;;
    --loop) loop_playback="true"; shift ;;
    --no-rviz) no_rviz="true"; shift ;;
    --exit-after-playback) exit_after_playback="true"; shift ;;
    --canonical-mapper-input) canonical_mapper_input="true"; shift ;;
    --record-canonical-mapper-input-bag) require_value "$1" "${2:-}"; record_canonical_mapper_input_bag="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'error: unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -z "${bag_dir}" || -z "${dataset}" ]] || { printf 'error: select only one of --bag or --dataset\n' >&2; exit 2; }
if [[ -n "${dataset}" ]]; then
  case "${dataset}" in
    airoom) bag_dir="${REPOSITORY_DIR}/outputs/experiment_01_ground_plane/captures/airoom_20261008_231538_796047065/airoom_20261008_231538_796047065" ;;
    hallway) bag_dir="${REPOSITORY_DIR}/outputs/experiment_01_ground_plane/captures/dynamic_long_seq_hallway_20261011_213549/dynamic_long_seq_hallway_20261011_213549" ;;
    *) printf 'error: --dataset must be airoom or hallway\n' >&2; exit 2 ;;
  esac
fi
[[ -n "${bag_dir}" ]] || { printf 'error: --bag or --dataset is required\n' >&2; usage >&2; exit 2; }
bag_dir="$(readlink -f -- "${bag_dir}")"
[[ -r "${bag_dir}/metadata.yaml" ]] || {
  printf 'error: rosbag2 metadata.yaml not found in: %s\n' "${bag_dir}" >&2
  exit 1
}
[[ "${pose_correction}" == "off" || "${pose_correction}" == "on" || "${pose_correction}" == "fixed_height" ]] || {
  printf 'error: --pose-correction must be off, on, or fixed_height\n' >&2; exit 2;
}
[[ "${local_submap_registration}" == "off" || "${local_submap_registration}" == "on" ]] || {
  printf 'error: --local-submap-registration must be off or on\n' >&2; exit 2;
}
[[ "${local_window_icp}" == "off" || "${local_window_icp}" == "on" ]] || {
  printf 'error: --local-window-icp must be off or on\n' >&2; exit 2;
}
[[ "${local_pose_graph}" == "off" || "${local_pose_graph}" == "on" ]] || {
  printf 'error: --local-pose-graph must be off or on\n' >&2; exit 2;
}
[[ "${local_line_pose_graph}" == "off" || "${local_line_pose_graph}" == "on" ]] || { printf 'error: --local-line-pose-graph must be off or on\n' >&2; exit 2; }
[[ "${local_pht_submap_pose_graph}" == "off" || "${local_pht_submap_pose_graph}" == "visualization_only" || "${local_pht_submap_pose_graph}" == "single_line" || "${local_pht_submap_pose_graph}" == "full_se2" ]] || { printf 'error: invalid --local-pht-submap-pose-graph mode\n' >&2; exit 2; }
[[ "${pht_profile}" == "length_1_5" || "${pht_profile}" == "length_0_7" ]] || { printf 'error: --pht-profile must be length_1_5 or length_0_7\n' >&2; exit 2; }
[[ "${pht_history_keyframes}" =~ ^[1-9][0-9]*$ ]] || {
  printf 'error: --pht-history-keyframes must be a positive integer\n' >&2
  exit 2
}
[[ "${icp_topic_queue_size}" =~ ^[1-9][0-9]*$ ]] || {
  printf 'error: --icp-topic-queue-size must be a positive integer\n' >&2
  exit 2
}
[[ "${local_window_icp}" != "on" || "${local_submap_registration}" != "on" ]] &&
[[ "${local_window_icp}" != "on" || "${local_pose_graph}" != "on" ]] &&
[[ "${local_window_icp}" != "on" || "${local_line_pose_graph}" != "on" ]] &&
[[ "${local_submap_registration}" != "on" || "${local_pose_graph}" != "on" ]] &&
[[ "${local_submap_registration}" != "on" || "${local_line_pose_graph}" != "on" ]] &&
[[ "${local_pose_graph}" != "on" || "${local_line_pose_graph}" != "on" ]] || {
  printf 'error: only one local x/y/yaw correction mode may be on\n' >&2; exit 2;
}
[[ "${local_pht_submap_pose_graph}" == "off" || ("${local_window_icp}" == "off" && "${local_submap_registration}" == "off" && "${local_pose_graph}" == "off" && "${local_line_pose_graph}" == "off") ]] || {
  printf 'error: PHT submap graph is exclusive with other local x/y/yaw correction modes\n' >&2; exit 2;
}
pht_mode_for_launch="visualization_only"
if [[ "${local_pht_submap_pose_graph}" != "off" ]]; then
  pht_mode_for_launch="${local_pht_submap_pose_graph}"
fi
[[ "${local_window_icp}" != "on" || "${pose_correction}" == "fixed_height" ]] || {
  printf 'error: --local-window-icp on currently supports --pose-correction fixed_height only\n' >&2; exit 2;
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
[[ "${canonical_mapper_input}" != "true" || "${pose_correction}" == "fixed_height" ]] || {
  printf 'error: canonical mapper input replay requires --pose-correction fixed_height\n' >&2; exit 2;
}
[[ "${canonical_mapper_input}" != "true" || -z "${record_canonical_mapper_input_bag}" ]] || {
  printf 'error: canonical replay cannot record another canonical mapper-input bag\n' >&2; exit 2;
}
if [[ -n "${record_canonical_mapper_input_bag}" ]]; then
  record_canonical_mapper_input_bag="$(readlink -m -- "${record_canonical_mapper_input_bag}")"
  [[ ! -e "${record_canonical_mapper_input_bag}" ]] || {
    printf 'error: canonical mapper-input bag output exists: %s\n' "${record_canonical_mapper_input_bag}" >&2; exit 1;
  }
fi

if [[ "${pose_correction}" == "fixed_height" && "${local_pht_submap_pose_graph}" != "off" ]]; then
  if [[ "${pht_profile}" == "length_0_7" ]]; then
    readonly CONFIG_FILE="${REPOSITORY_DIR}/config/experiment/${EXPERIMENT_ID}/pht_submap_pose_graph_len_0_7.yaml"
  else
    readonly CONFIG_FILE="${REPOSITORY_DIR}/config/experiment/${EXPERIMENT_ID}/pht_submap_pose_graph.yaml"
  fi
elif [[ "${pose_correction}" == "fixed_height" && "${local_line_pose_graph}" == "on" ]]; then
  readonly CONFIG_FILE="${REPOSITORY_DIR}/config/experiment/${EXPERIMENT_ID}/local_line_pose_graph.yaml"
elif [[ "${pose_correction}" == "fixed_height" && "${local_pose_graph}" == "on" ]]; then
  readonly CONFIG_FILE="${REPOSITORY_DIR}/config/experiment/${EXPERIMENT_ID}/local_pose_graph.yaml"
elif [[ "${pose_correction}" == "fixed_height" && "${local_submap_registration}" == "on" ]]; then
  readonly CONFIG_FILE="${REPOSITORY_DIR}/config/experiment/${EXPERIMENT_ID}/local_submap_registration.yaml"
elif [[ "${pose_correction}" == "fixed_height" && "${local_window_icp}" == "on" ]]; then
  readonly CONFIG_FILE="${REPOSITORY_DIR}/config/experiment/${EXPERIMENT_ID}/local_window_icp_on.yaml"
elif [[ "${pose_correction}" == "fixed_height" ]]; then
  readonly CONFIG_FILE="${REPOSITORY_DIR}/config/experiment/${EXPERIMENT_ID}/fixed_height_imu_gravity.yaml"
else
  readonly CONFIG_FILE="${REPOSITORY_DIR}/config/experiment/${EXPERIMENT_ID}/pose_${pose_correction}.yaml"
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
local_window_icp_enabled: ${local_window_icp}
local_submap_registration_enabled: ${local_submap_registration}
local_pose_graph_enabled: ${local_pose_graph}
local_line_pose_graph_enabled: ${local_line_pose_graph}
local_pht_submap_pose_graph_mode: "${pht_mode_for_launch}"
pht_profile: "${pht_profile}"
pht_history_keyframes: ${pht_history_keyframes}
icp_topic_queue_size: ${icp_topic_queue_size}
height_filter_enabled: true
height_reference_mode: "${height_reference_mode}"
lidar_mount_height_above_floor_m: ${lidar_mount_height_m:-null}
fixed_base_link_height_above_floor_m: ${fixed_base_link_height_m:-null}
height_filter_min_m: ${height_filter_min_m}
height_filter_max_m: ${height_filter_max_m}
gravity_aligned_sensor_band_min_m: ${gravity_band_min_m:-null}
gravity_aligned_sensor_band_max_m: ${gravity_band_max_m:-null}
rear_sector_filter_enabled: true
rear_filter_input_topic: "/r0/livox/lidar"
rear_filter_output_topic: "/r0/mapping/lidar_rear_filtered"
icp_input_topic: "/r0/livox/lidar"
sensor_tf_roll_rad: 0.0
robot_namespace: "r0"
ros_domain_id: "${domain_id}"
playback_rate: ${rate}
loop_playback: ${loop_playback}
restart_mapping_each_loop: ${loop_playback}
config_snapshot: "config.yaml"
odometry_config_snapshot: "lidar_odometry.yaml"
rviz_config: "rviz/experiment/${EXPERIMENT_ID}_replay.rviz"
rviz_use_sim_time: true
repository_branch: "$(git -C "${REPOSITORY_DIR}" branch --show-current 2>/dev/null || true)"
repository_commit: "$(git -C "${REPOSITORY_DIR}" rev-parse HEAD 2>/dev/null || true)"
EOF

cat >"${RUN_DIR}/parameter_overrides.yaml" <<EOF
# Command-line override applied after config.yaml during this replay.
local_pht_submap_history_keyframes: ${pht_history_keyframes}
local_pht_submap_pose_graph_mode: "${local_pht_submap_pose_graph}"
icp_topic_queue_size: ${icp_topic_queue_size}
EOF

cd -- "${REPOSITORY_DIR}"
launch_pid=""
rviz_pid=""
topdown_exporter_pid=""
canonical_record_pid=""

stop_mapping() {
  if [[ -n "${launch_pid}" ]]; then
    kill -TERM -- "-${launch_pid}" 2>/dev/null || kill -TERM "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
    launch_pid=""
  fi
}

stop_topdown_exporter() {
  if [[ -n "${topdown_exporter_pid}" ]]; then
    kill -INT -- "-${topdown_exporter_pid}" 2>/dev/null || kill -INT "${topdown_exporter_pid}" 2>/dev/null || true
    wait "${topdown_exporter_pid}" 2>/dev/null || true
    topdown_exporter_pid=""
  fi
}

stop_canonical_recording() {
  if [[ -n "${canonical_record_pid}" ]]; then
    kill -INT -- "-${canonical_record_pid}" 2>/dev/null || kill -INT "${canonical_record_pid}" 2>/dev/null || true
    wait "${canonical_record_pid}" 2>/dev/null || true
    canonical_record_pid=""
  fi
}

cleanup() {
  trap - INT TERM EXIT
  if [[ -n "${rviz_pid}" ]] && kill -0 "${rviz_pid}" 2>/dev/null; then
    kill -TERM -- "-${rviz_pid}" 2>/dev/null || true
    wait "${rviz_pid}" 2>/dev/null || true
  fi
  stop_mapping
  stop_topdown_exporter
  stop_canonical_recording
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_mapping() {
  local cycle="$1"
  local cycle_label
  printf -v cycle_label '%04d' "${cycle}"
  printf 'Starting mapping for replay pass %s\n' "${cycle_label}"
  if [[ "${canonical_mapper_input}" == "true" ]]; then
    setsid ros2 launch "${REPOSITORY_DIR}/scripts/experiment/support/launch/canonical_mapper_input_mapping.launch.py" \
      use_sim_time:=true robot_id:=0 \
      scan_cloud_topic:=/r0/mapping/plane_height_filtered \
      odom_topic:=/r0/mapping/floor_based_odometry \
      local_frame_id:=r0/base_link global_frame_id:=odom \
      corrected_odometry_topic:=/r0/mapping/occupancy_mapper_odometry \
      local_pht_submap_diagnostics_csv_path:="${RUN_DIR}/pht_submap_pose_graph.csv" \
      local_pht_submap_visualization_dir:="${RUN_DIR}/pht_submap_pose_graph_visualization" \
      local_pht_submap_pose_graph_mode:="${pht_mode_for_launch}" \
      local_pht_submap_history_keyframes:="${pht_history_keyframes}" \
      occupancy_config_file:="${RUN_DIR}/config.yaml" \
      >"${RUN_DIR}/logs/launch_cycle_${cycle_label}.log" 2>&1 &
    launch_pid=$!
    sleep 3
    return
  fi
  setsid ros2 launch "${REPOSITORY_DIR}/scripts/experiment/support/launch/single_bag_mapping.launch.py" \
    use_bag:=false use_sim_time:=true publish_tf_odom:=true publish_sensor_static_tf:=false \
    enable_rear_lidar_filter:=true imu_input_is_filtered:=false expected_update_rate:=20.0 \
    icp_topic_queue_size:=${icp_topic_queue_size} \
    sensor_parent_frame:=r0/base_link sensor_child_frame:=r0/livox_frame local_frame_id:=r0/base_link \
    bag_lidar_topic:=/r0/livox/lidar scan_cloud_topic:=/r0/mapping/lidar_rear_filtered icp_scan_cloud_topic:=/r0/livox/lidar \
    imu_raw_topic:=/r0/livox/imu imu_filtered_topic:=/r0/livox/imu_filtered \
    plane_filtered_cloud_topic:=/r0/mapping/plane_height_filtered mapping_odometry_topic:=/r0/mapping/floor_based_odometry \
    corrected_odometry_topic:=/r0/mapping/occupancy_mapper_odometry global_frame_id:=odom \
    local_submap_diagnostics_csv_path:="${RUN_DIR}/local_submap_registration.csv" \
    local_pose_graph_diagnostics_csv_path:="${RUN_DIR}/local_pose_graph.csv" \
    local_pose_graph_visualization_dir:="${RUN_DIR}/pose_graph_visualization" \
    local_line_pose_graph_diagnostics_csv_path:="${RUN_DIR}/local_line_pose_graph.csv" \
    local_line_pose_graph_visualization_dir:="${RUN_DIR}/line_pose_graph_visualization" \
    local_pht_submap_diagnostics_csv_path:="${RUN_DIR}/pht_submap_pose_graph.csv" \
    local_pht_submap_visualization_dir:="${RUN_DIR}/pht_submap_pose_graph_visualization" \
    local_pht_submap_pose_graph_mode:="${pht_mode_for_launch}" \
    local_pht_submap_history_keyframes:="${pht_history_keyframes}" \
    odometry_config_file:="${RUN_DIR}/lidar_odometry.yaml" occupancy_config_file:="${RUN_DIR}/config.yaml" \
    >"${RUN_DIR}/logs/launch_cycle_${cycle_label}.log" 2>&1 &
  launch_pid=$!
  sleep 3
}

start_mapping 1
setsid python3 "${REPOSITORY_DIR}/scripts/experiment/02_occupancy_mapping/export_experiment_02_topdown.py" --ros-args \
  -p output_directory:="${RUN_DIR}/topdown" \
  >"${RUN_DIR}/logs/topdown_exporter.log" 2>&1 &
topdown_exporter_pid=$!
sleep 1
if [[ "${no_rviz}" != "true" ]]; then
  setsid rviz2 -d "${RVIZ_CONFIG}" --ros-args -p use_sim_time:=true \
    >"${RUN_DIR}/logs/rviz.log" 2>&1 &
  rviz_pid=$!
fi
if [[ -n "${record_canonical_mapper_input_bag}" ]]; then
  setsid ros2 bag record --storage sqlite3 --output "${record_canonical_mapper_input_bag}" \
    /r0/livox/lidar /r0/livox/imu /tf_static /r0/odom \
    /r0/mapping/plane_height_filtered /r0/mapping/floor_based_odometry \
    >"${RUN_DIR}/logs/canonical_mapper_input_record.log" 2>&1 &
  canonical_record_pid=$!
  sleep 1
fi
printf 'Experiment: %s\nRun: %s\nPose correction: %s\nLocal window ICP: %s\nBag: %s\nOutput: %s\n' \
  "${EXPERIMENT_ID}" "${run_id}" "${pose_correction}" "${local_window_icp}" "${bag_dir}" "${RUN_DIR}"

generate_pht_html() {
  [[ "${local_pht_submap_pose_graph}" != "off" ]] || return 0
  # Let synchronized callbacks drain after rosbag playback reaches EOF.
  sleep 2
  local html_path="${RUN_DIR}/pht_submap_pose_graph_visualization/pht_cumulative_submap_inspection.html"
  if python3 "${REPOSITORY_DIR}/scripts/experiment/02_occupancy_mapping/analyze_experiment_02_pht_submap_pose_graph.py" \
      --snapshot-dir "${RUN_DIR}/pht_submap_pose_graph_visualization" --output "${html_path}"; then
    printf 'PHT inspection HTML: %s\n' "${html_path}"
  else
    printf 'warning: PHT inspection HTML was not generated\n' >&2
  fi
}

cycle=1
while true; do
  printf '\nStarting bag replay pass %04d\n' "${cycle}" | tee -a "${RUN_DIR}/logs/bag_play.log"
  if [[ "${canonical_mapper_input}" == "true" ]]; then
    playback_topics=(/r0/mapping/plane_height_filtered /r0/mapping/floor_based_odometry /r0/odom /tf_static)
  else
    playback_topics=(/r0/livox/lidar /r0/livox/imu /tf_static)
  fi
  ros2 bag play --clock --rate "${rate}" "${bag_dir}" \
    --topics "${playback_topics[@]}" \
    2>&1 | tee -a "${RUN_DIR}/logs/bag_play.log"

  if [[ "${loop_playback}" != "true" ]]; then
    generate_pht_html
    stop_canonical_recording
    if [[ "${exit_after_playback}" == "true" ]]; then
      stop_mapping
      break
    fi
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
