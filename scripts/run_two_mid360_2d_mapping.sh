#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "${SCRIPT_PATH}")" && pwd)"
REPOSITORY_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

MID360_ENV_SOURCE="${MID360_ENV_FILE:-}"
if [[ -z "${MID360_ENV_SOURCE}" ]]; then
  for env_candidate in \
    "${SCRIPT_DIR}/mid360.env" \
    "${REPOSITORY_DIR}/../../bash/mid360.env"; do
    if [[ -r "${env_candidate}" ]]; then
      MID360_ENV_SOURCE="${env_candidate}"
      break
    fi
  done
fi
if [[ -n "${MID360_ENV_SOURCE}" ]]; then
  [[ -r "${MID360_ENV_SOURCE}" ]] || {
    printf 'error: MID-360 environment file is not readable: %s\n' \
      "${MID360_ENV_SOURCE}" >&2
    exit 1
  }
  set +u
  # shellcheck disable=SC1090
  source "${MID360_ENV_SOURCE}"
  set -u
fi

ROBOT_NUMBER="${ROBOT_NUMBER:-${ROBOT_ID:-}}"
RUN_LOCAL_MAPPING="${TWO_LIVE_LOCAL_MAPPING:-true}"
RUN_FUSION="${TWO_LIVE_MAPPING_HOST:-false}"
WAIT_FOR_INITIAL_ALIGNMENT="${TWO_LIVE_WAIT_FOR_INITIAL_ALIGNMENT:-true}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
LIVOX_WORKSPACE="${LIVOX_WORKSPACE:-}"
LIVOX_SETUP="${LIVOX_SETUP:-}"
DRIVER_CONFIG="${LIVOX_CONFIG:-}"
MAPPING_WORKSPACE="${MAPPING_WORKSPACE:-${REPOSITORY_DIR}}"
MAPPING_CONFIG="${MAPPING_CONFIG:-}"
RVIZ_CONFIG="${RVIZ_CONFIG:-}"
DOMAIN_ID="${ROS_DOMAIN_ID:-72}"
EXPECTED_UPDATE_RATE="${EXPECTED_UPDATE_RATE:-11.0}"
ROBOT0_LIDAR_TOPIC="/r0/livox/lidar"
ROBOT0_IMU_TOPIC="/r0/livox/imu"
ROBOT1_LIDAR_TOPIC="/r1/livox/lidar"
ROBOT1_IMU_TOPIC="/r1/livox/imu"
ALIGNMENT_TOPIC="${INITIAL_ALIGNMENT_TOPIC:-/toy/initial_xy_alignment}"
COMMON_FRAME_ID="${COMMON_FRAME_ID:-map}"
ALIGNMENT_STARTUP_DELAY_SEC="${INITIAL_ALIGNMENT_STARTUP_DELAY_SEC:-10.0}"
ALIGNMENT_TIMEOUT_SEC="${INITIAL_ALIGNMENT_TIMEOUT_SEC:-0}"
PUBLISH_SENSOR_STATIC_TF="true"
START_RVIZ=true
PROCESS_GROUPS=()
CHILD_PIDS=()
EXTRA_LAUNCH_ARGS=()
ALIGNMENT_FORWARD_ARGS=()
FUSION_FORWARD_ARGS=()

usage() {
  cat <<EOF
Usage: $(basename "$0") --robot-number {1|2} [options]

Run this same script on both robot laptops. The MID-360 drivers start first.
By default, RTAB-Map odometry and occupancy mapping stay stopped until the
mapping-host laptop finishes initial_xy_icp_alignment and publishes
${ALIGNMENT_TOPIC}. Add --mapping-host on exactly one laptop.

ROBOT_ID is loaded automatically from a nearby mid360.env when available.
Set MID360_ENV_FILE=/path/to/mid360.env to select one explicitly.

Roles:
  physical robot 1 -> r0 -> /r0/livox/lidar and /r0/livox/imu
  physical robot 2 -> r1 -> /r1/livox/lidar and /r1/livox/imu

Options:
  --robot-number NUMBER        Physical robot number: 1 or 2
  --mapping-host               Run initial alignment and post-alignment fusion here
  --local-mapping-only         Driver + this robot mapping; waits for remote alignment
  --driver-only                Run only this laptop's driver; no odom
  --no-initial-alignment-gate  Legacy/debug mode: start odom concurrently
  --alignment-topic TOPIC      Alignment topic (default: ${ALIGNMENT_TOPIC})
  --alignment-startup-delay SEC
                                Stationary settle time before ICP (default: ${ALIGNMENT_STARTUP_DELAY_SEC})
  --alignment-timeout SEC      Gate timeout; 0 waits indefinitely (default: ${ALIGNMENT_TIMEOUT_SEC})
  --domain-id ID               ROS domain shared by both laptops (default: ${DOMAIN_ID})
  --ros-setup FILE             ROS setup.bash (default: ${ROS_SETUP})
  --livox-workspace DIR        Built Livox workspace (default: auto-detect)
  --livox-setup FILE           Livox local_setup.bash
  --driver-config FILE         This laptop's MID360_config.json
  --mapping-workspace DIR      Built mapping workspace (default: ${MAPPING_WORKSPACE})
  --mapping-config FILE        Occupancy YAML; also configures initial ICP
  --expected-update-rate HZ    RTAB-Map input-rate ceiling (default: ${EXPECTED_UPDATE_RATE})
  --rviz-config FILE           Two-robot RViz config
  --robot0-lidar-topic TOPIC   r0 LiDAR input (default: ${ROBOT0_LIDAR_TOPIC})
  --robot0-imu-topic TOPIC     r0 IMU input (default: ${ROBOT0_IMU_TOPIC})
  --robot1-lidar-topic TOPIC   r1 LiDAR input (default: ${ROBOT1_LIDAR_TOPIC})
  --robot1-imu-topic TOPIC     r1 IMU input (default: ${ROBOT1_IMU_TOPIC})
  --publish-sensor-static-tf BOOL
                                Publish robot-specific sensor TF (default: true)
  --launch-arg NAME:=VALUE     Additional two_live_mapping launch argument;
                                may be repeated
  --no-rviz                    Mapping host runs without RViz
  -h, --help                   Show this help

Examples:
  # Laptop connected to physical robot 1: wait, then r0 odom/mapping
  $(basename "$0") --robot-number 1

  # Laptop connected to physical robot 2: run alignment, then r1 + fusion
  $(basename "$0") --robot-number 2 --mapping-host
EOF
}

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

require_value() {
  local option=$1
  local value=${2:-}
  [[ -n "${value}" ]] || die "${option} requires a value"
}

while (($# > 0)); do
  case "$1" in
    --robot-number)
      require_value "$1" "${2:-}"
      ROBOT_NUMBER=$2
      shift 2
      ;;
    --mapping-host)
      RUN_LOCAL_MAPPING=true
      RUN_FUSION=true
      shift
      ;;
    --local-mapping-only)
      RUN_LOCAL_MAPPING=true
      RUN_FUSION=false
      shift
      ;;
    --driver-only)
      RUN_LOCAL_MAPPING=false
      RUN_FUSION=false
      shift
      ;;
    --no-initial-alignment-gate)
      WAIT_FOR_INITIAL_ALIGNMENT=false
      shift
      ;;
    --alignment-topic)
      require_value "$1" "${2:-}"
      ALIGNMENT_TOPIC=$2
      shift 2
      ;;
    --alignment-startup-delay)
      require_value "$1" "${2:-}"
      ALIGNMENT_STARTUP_DELAY_SEC=$2
      shift 2
      ;;
    --alignment-timeout)
      require_value "$1" "${2:-}"
      ALIGNMENT_TIMEOUT_SEC=$2
      shift 2
      ;;
    --domain-id)
      require_value "$1" "${2:-}"
      DOMAIN_ID=$2
      shift 2
      ;;
    --ros-setup)
      require_value "$1" "${2:-}"
      ROS_SETUP=$2
      shift 2
      ;;
    --livox-workspace)
      require_value "$1" "${2:-}"
      LIVOX_WORKSPACE=$2
      shift 2
      ;;
    --livox-setup)
      require_value "$1" "${2:-}"
      LIVOX_SETUP=$2
      shift 2
      ;;
    --driver-config)
      require_value "$1" "${2:-}"
      DRIVER_CONFIG=$2
      shift 2
      ;;
    --mapping-workspace)
      require_value "$1" "${2:-}"
      MAPPING_WORKSPACE=$2
      shift 2
      ;;
    --mapping-config)
      require_value "$1" "${2:-}"
      MAPPING_CONFIG=$2
      shift 2
      ;;
    --expected-update-rate)
      require_value "$1" "${2:-}"
      EXPECTED_UPDATE_RATE=$2
      shift 2
      ;;
    --rviz-config)
      require_value "$1" "${2:-}"
      RVIZ_CONFIG=$2
      shift 2
      ;;
    --robot0-lidar-topic)
      require_value "$1" "${2:-}"
      ROBOT0_LIDAR_TOPIC=$2
      shift 2
      ;;
    --robot0-imu-topic)
      require_value "$1" "${2:-}"
      ROBOT0_IMU_TOPIC=$2
      shift 2
      ;;
    --robot1-lidar-topic)
      require_value "$1" "${2:-}"
      ROBOT1_LIDAR_TOPIC=$2
      shift 2
      ;;
    --robot1-imu-topic)
      require_value "$1" "${2:-}"
      ROBOT1_IMU_TOPIC=$2
      shift 2
      ;;
    --publish-sensor-static-tf)
      require_value "$1" "${2:-}"
      PUBLISH_SENSOR_STATIC_TF=$2
      shift 2
      ;;
    --launch-arg)
      require_value "$1" "${2:-}"
      [[ "$2" == *:=* ]] || die "--launch-arg must use NAME:=VALUE syntax"
      EXTRA_LAUNCH_ARGS+=("$2")
      shift 2
      ;;
    --no-rviz)
      START_RVIZ=false
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown option: $1"
      ;;
  esac
done

[[ "${ROBOT_NUMBER}" == "1" || "${ROBOT_NUMBER}" == "2" ]] ||
  die "--robot-number must be 1 or 2 (or set ROBOT_NUMBER/ROBOT_ID)"
[[ "${RUN_LOCAL_MAPPING}" == "true" || "${RUN_LOCAL_MAPPING}" == "false" ]] ||
  die "TWO_LIVE_LOCAL_MAPPING must be true or false"
[[ "${RUN_FUSION}" == "true" || "${RUN_FUSION}" == "false" ]] ||
  die "TWO_LIVE_MAPPING_HOST must be true or false"
[[ "${WAIT_FOR_INITIAL_ALIGNMENT}" == "true" || "${WAIT_FOR_INITIAL_ALIGNMENT}" == "false" ]] ||
  die "TWO_LIVE_WAIT_FOR_INITIAL_ALIGNMENT must be true or false"
if [[ "${RUN_FUSION}" == true && "${RUN_LOCAL_MAPPING}" != true ]]; then
  die "mapping host must also run its local mapping pipeline"
fi
[[ "${DOMAIN_ID}" =~ ^[0-9]+$ ]] || die "domain ID must be an integer"
((DOMAIN_ID >= 0 && DOMAIN_ID <= 232)) || die "domain ID must be between 0 and 232"
[[ "${EXPECTED_UPDATE_RATE}" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
  die "expected update rate must be a positive number"
[[ "${EXPECTED_UPDATE_RATE}" != "0" && "${EXPECTED_UPDATE_RATE}" != "0.0" ]] ||
  die "expected update rate must be greater than zero"
[[ "${ALIGNMENT_STARTUP_DELAY_SEC}" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
  die "alignment startup delay must be a non-negative number"
[[ "${ALIGNMENT_TIMEOUT_SEC}" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
  die "alignment timeout must be a non-negative number"
[[ "${ALIGNMENT_TOPIC}" == /* ]] || die "alignment topic must be absolute"
if [[ "${PUBLISH_SENSOR_STATIC_TF}" != "true" && "${PUBLISH_SENSOR_STATIC_TF}" != "false" ]]; then
  die "--publish-sensor-static-tf must be true or false"
fi

# Keep the two-stage auxiliary launches consistent with selected public launch
# arguments without forwarding unrelated arguments they do not declare.
for launch_arg in "${EXTRA_LAUNCH_ARGS[@]}"; do
  launch_name=${launch_arg%%:=*}
  launch_value=${launch_arg#*:=}
  case "${launch_name}" in
    alignment_topic)
      ALIGNMENT_TOPIC=${launch_value}
      ;;
    common_frame_id)
      COMMON_FRAME_ID=${launch_value}
      ;;
    sensor_tf_x_0|sensor_tf_y_0|sensor_tf_z_0|sensor_tf_yaw_0|sensor_tf_pitch_0|sensor_tf_roll_0|sensor_tf_x_1|sensor_tf_y_1|sensor_tf_z_1|sensor_tf_yaw_1|sensor_tf_pitch_1|sensor_tf_roll_1)
      ALIGNMENT_FORWARD_ARGS+=("${launch_arg}")
      ;;
    record_publish_period_ms|record_output_prefix|record_publish_merged_global)
      FUSION_FORWARD_ARGS+=("${launch_arg}")
      ;;
  esac
done

if [[ -z "${LIVOX_WORKSPACE}" ]]; then
  livox_workspace_candidates=(
    "${REPOSITORY_DIR}/../ws_livox"
    "${HOME}/aibot/livox_mid360/ws_livox"
    "${HOME}/ws_livox"
    "${HOME}/livox_ws"
  )
  for workspace_candidate in "${livox_workspace_candidates[@]}"; do
    if [[ -r "${workspace_candidate}/install/local_setup.bash" &&
          -r "${workspace_candidate}/src/livox_ros_driver2/config/MID360_config.json" ]]; then
      LIVOX_WORKSPACE="${workspace_candidate}"
      break
    fi
  done
fi
[[ -n "${LIVOX_WORKSPACE}" ]] ||
  die "Livox workspace was not found; pass --livox-workspace /path/to/ws_livox"

LIVOX_SETUP="${LIVOX_SETUP:-${LIVOX_WORKSPACE}/install/local_setup.bash}"
DRIVER_CONFIG="${DRIVER_CONFIG:-${LIVOX_WORKSPACE}/src/livox_ros_driver2/config/MID360_config.json}"
MAPPING_CONFIG="${MAPPING_CONFIG:-${MAPPING_WORKSPACE}/config/occupancy.yaml}"
RVIZ_CONFIG="${RVIZ_CONFIG:-${MAPPING_WORKSPACE}/rviz/two_robot_mapping.rviz}"
MAPPING_SETUP="${MAPPING_WORKSPACE}/install/local_setup.bash"
LOCAL_DRIVER_SCRIPT="${SCRIPT_DIR}/run_livox_robot.sh"

required_files=(
  "${ROS_SETUP}"
  "${LIVOX_SETUP}"
  "${DRIVER_CONFIG}"
  "${LOCAL_DRIVER_SCRIPT}"
)
if [[ "${RUN_LOCAL_MAPPING}" == true || "${RUN_FUSION}" == true ]]; then
  required_files+=("${MAPPING_SETUP}" "${MAPPING_CONFIG}")
  if [[ "${RUN_FUSION}" == true && "${START_RVIZ}" == true ]]; then
    required_files+=("${RVIZ_CONFIG}")
  fi
fi
for required_file in "${required_files[@]}"; do
  [[ -f "${required_file}" ]] || die "required file not found: ${required_file}"
done

command -v flock >/dev/null || die "flock is required"
command -v setsid >/dev/null || die "setsid is required"
if [[ "${WAIT_FOR_INITIAL_ALIGNMENT}" == true && "${ALIGNMENT_TIMEOUT_SEC}" != "0" && "${ALIGNMENT_TIMEOUT_SEC}" != "0.0" ]]; then
  command -v timeout >/dev/null || die "timeout is required for a finite alignment timeout"
fi

if [[ "${RUN_LOCAL_MAPPING}" == true || "${RUN_FUSION}" == true ]]; then
  sensor_topics=(
    "${ROBOT0_LIDAR_TOPIC}"
    "${ROBOT0_IMU_TOPIC}"
    "${ROBOT1_LIDAR_TOPIC}"
    "${ROBOT1_IMU_TOPIC}"
  )
  for topic in "${sensor_topics[@]}"; do
    [[ "${topic}" == /* ]] || die "sensor topics must be absolute: ${topic}"
  done
  for ((left=0; left<${#sensor_topics[@]}; left++)); do
    for ((right=left+1; right<${#sensor_topics[@]}; right++)); do
      [[ "${sensor_topics[left]}" != "${sensor_topics[right]}" ]] ||
        die "all four sensor topics must be different: ${sensor_topics[left]}"
    done
  done

  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  # shellcheck disable=SC1090
  source "${MAPPING_SETUP}"
  set -u

  command -v ros2 >/dev/null || die "ros2 is unavailable after sourcing ROS"
  ros2 pkg prefix co_3dto2d_mapping >/dev/null ||
    die "co_3dto2d_mapping is unavailable after sourcing ${MAPPING_SETUP}"
fi

export ROS_DOMAIN_ID="${DOMAIN_ID}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

LOCK_FILE="${XDG_RUNTIME_DIR:-/tmp}/run_two_mid360_robot${ROBOT_NUMBER}.lock"
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  die "physical robot ${ROBOT_NUMBER} live driver is already running"
fi

start_process_group() {
  local output_variable=$1
  shift

  setsid "$@" &
  local process_id=$!
  printf -v "${output_variable}" '%s' "${process_id}"
  PROCESS_GROUPS+=("${process_id}")
  CHILD_PIDS+=("${process_id}")
}

cleanup() {
  local status=$?
  local index
  local process_id

  trap - INT TERM HUP EXIT

  for ((index=${#PROCESS_GROUPS[@]} - 1; index >= 0; index--)); do
    process_id=${PROCESS_GROUPS[index]}
    kill -TERM -- "-${process_id}" 2>/dev/null || true
  done

  for _ in {1..30}; do
    local any_running=false
    for process_id in "${PROCESS_GROUPS[@]}"; do
      if kill -0 -- "-${process_id}" 2>/dev/null; then
        any_running=true
        break
      fi
    done
    if [[ "${any_running}" == false ]]; then
      break
    fi
    sleep 0.1
  done

  for process_id in "${PROCESS_GROUPS[@]}"; do
    if kill -0 -- "-${process_id}" 2>/dev/null; then
      kill -KILL -- "-${process_id}" 2>/dev/null || true
    fi
  done

  wait "${CHILD_PIDS[@]}" 2>/dev/null || true
  exit "${status}"
}
trap cleanup INT TERM HUP EXIT

if [[ "${ROBOT_NUMBER}" == "1" ]]; then
  ROBOT_NAMESPACE="r0"
else
  ROBOT_NAMESPACE="r1"
fi

if [[ "${RUN_FUSION}" == true ]]; then
  RVIZ_STATUS="${START_RVIZ}"
else
  RVIZ_STATUS="disabled"
fi

printf '%s\n' \
  "Starting physical robot ${ROBOT_NUMBER} as ${ROBOT_NAMESPACE}:" \
  "  ROS_DOMAIN_ID:       ${ROS_DOMAIN_ID}" \
  "  ROS_LOCALHOST_ONLY:  ${ROS_LOCALHOST_ONLY}" \
  "  environment file:   ${MID360_ENV_SOURCE:-not used}" \
  "  Livox workspace:     ${LIVOX_WORKSPACE}" \
  "  driver config:       ${DRIVER_CONFIG}" \
  "  local mapping:       ${RUN_LOCAL_MAPPING}" \
  "  fusion host:         ${RUN_FUSION}" \
  "  alignment gate:      ${WAIT_FOR_INITIAL_ALIGNMENT}" \
  "  alignment topic:     ${ALIGNMENT_TOPIC}" \
  "  expected rate:       ${EXPECTED_UPDATE_RATE} Hz" \
  "  RViz:                ${RVIZ_STATUS}"

start_process_group DRIVER_PID env \
  "ROS_SETUP=${ROS_SETUP}" \
  "LIVOX_SETUP=${LIVOX_SETUP}" \
  "LIVOX_CONFIG=${DRIVER_CONFIG}" \
  "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
  "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}" \
  bash "${LOCAL_DRIVER_SCRIPT}" "${ROBOT_NUMBER}"

if [[ "${RUN_LOCAL_MAPPING}" == true || "${RUN_FUSION}" == true ]]; then
  if [[ "${ROBOT_NUMBER}" == "1" && "${RUN_LOCAL_MAPPING}" == true ]]; then
    ENABLE_ROBOT0_PIPELINE="true"
  else
    ENABLE_ROBOT0_PIPELINE="false"
  fi
  if [[ "${ROBOT_NUMBER}" == "2" && "${RUN_LOCAL_MAPPING}" == true ]]; then
    ENABLE_ROBOT1_PIPELINE="true"
  else
    ENABLE_ROBOT1_PIPELINE="false"
  fi

  if [[ "${WAIT_FOR_INITIAL_ALIGNMENT}" == true ]]; then
    if [[ "${RUN_FUSION}" == true ]]; then
      alignment_command=(
        ros2 launch co_3dto2d_mapping initial_alignment_live.launch.py
        "mapping_config_file:=${MAPPING_CONFIG}"
        "robot0_cloud_topic:=${ROBOT0_LIDAR_TOPIC}"
        "robot1_cloud_topic:=${ROBOT1_LIDAR_TOPIC}"
        "alignment_topic:=${ALIGNMENT_TOPIC}"
        "common_frame_id:=${COMMON_FRAME_ID}"
        "startup_delay_sec:=${ALIGNMENT_STARTUP_DELAY_SEC}"
        "publish_sensor_static_tf:=${PUBLISH_SENSOR_STATIC_TF}"
      )
      alignment_command+=("${ALIGNMENT_FORWARD_ARGS[@]}")
      start_process_group INITIAL_ALIGNMENT_PID "${alignment_command[@]}"
      printf '%s\n' \
        "[STAGE 1/2] Initial alignment host is active." \
        "[STAGE 1/2] Keep both robots stationary and side-by-side."
    else
      printf '%s\n' \
        "[STAGE 1/2] Waiting for the mapping-host laptop to publish initial alignment." \
        "[STAGE 1/2] No RTAB-Map odometry or occupancy node has been started."
    fi

    alignment_wait_command=(
      ros2 topic echo
      --once
      --qos-reliability reliable
      --qos-durability transient_local
      "${ALIGNMENT_TOPIC}"
      geometry_msgs/msg/TransformStamped
    )
    printf '[STAGE 1/2] ODOM/MAPPING BLOCKED: waiting for %s ...\n' \
      "${ALIGNMENT_TOPIC}"
    if [[ "${ALIGNMENT_TIMEOUT_SEC}" == "0" || "${ALIGNMENT_TIMEOUT_SEC}" == "0.0" ]]; then
      "${alignment_wait_command[@]}" >/dev/null 2>&1 ||
        die "initial alignment wait failed"
    else
      timeout "${ALIGNMENT_TIMEOUT_SEC}s" \
        "${alignment_wait_command[@]}" >/dev/null 2>&1 ||
        die "initial alignment was not received within ${ALIGNMENT_TIMEOUT_SEC}s"
    fi
    printf '%s\n' \
      "[STAGE 1/2] INITIAL ALIGNMENT READY." \
      "[STAGE 2/2] Releasing RTAB-Map odometry and occupancy mapping now."

    # The alignment-only launch owns both sensor static TFs and the fixed
    # alignment publisher. The normal mapping launches must not duplicate them.
    MAPPING_ENABLE_FUSION=false
    MAPPING_PUBLISH_STATIC_TF=false
    MAPPING_STARTUP_DELAY_SEC=0.0
  else
    MAPPING_ENABLE_FUSION=${RUN_FUSION}
    MAPPING_PUBLISH_STATIC_TF=${PUBLISH_SENSOR_STATIC_TF}
    MAPPING_STARTUP_DELAY_SEC=10.0
  fi

  mapping_command=(
    ros2 launch co_3dto2d_mapping two_live_mapping.launch.py
    "enable_robot0_pipeline:=${ENABLE_ROBOT0_PIPELINE}"
    "enable_robot1_pipeline:=${ENABLE_ROBOT1_PIPELINE}"
    "enable_fusion:=${MAPPING_ENABLE_FUSION}"
    "robot0_lidar_topic:=${ROBOT0_LIDAR_TOPIC}"
    "robot0_imu_topic:=${ROBOT0_IMU_TOPIC}"
    "robot1_lidar_topic:=${ROBOT1_LIDAR_TOPIC}"
    "robot1_imu_topic:=${ROBOT1_IMU_TOPIC}"
    "expected_update_rate:=${EXPECTED_UPDATE_RATE}"
    "publish_sensor_static_tf:=${MAPPING_PUBLISH_STATIC_TF}"
    "mapping_startup_delay_sec:=${MAPPING_STARTUP_DELAY_SEC}"
    "alignment_topic:=${ALIGNMENT_TOPIC}"
    "common_frame_id:=${COMMON_FRAME_ID}"
    "occupancy_config_file:=${MAPPING_CONFIG}"
  )
  mapping_command+=("${EXTRA_LAUNCH_ARGS[@]}")
  start_process_group MAPPING_PID "${mapping_command[@]}"

  if [[ "${WAIT_FOR_INITIAL_ALIGNMENT}" == true && "${RUN_FUSION}" == true ]]; then
    fusion_command=(
      ros2 launch co_3dto2d_mapping post_alignment_fusion.launch.py
      "alignment_topic:=${ALIGNMENT_TOPIC}"
      "common_frame_id:=${COMMON_FRAME_ID}"
    )
    fusion_command+=("${FUSION_FORWARD_ARGS[@]}")
    start_process_group FUSION_PID "${fusion_command[@]}"
  fi

  if [[ "${RUN_FUSION}" == true && "${START_RVIZ}" == true ]]; then
    start_process_group RVIZ_PID rviz2 -d "${RVIZ_CONFIG}"
  fi
fi

printf '%s\n' \
  "Physical robot ${ROBOT_NUMBER} live process is running." \
  "Move only after the '[STAGE 2/2]' message has appeared on both laptops." \
  "Press Ctrl-C to stop every process started on this laptop."

set +e
wait -n "${CHILD_PIDS[@]}"
status=$?
set -e
exit "${status}"
