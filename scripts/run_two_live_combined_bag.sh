#!/usr/bin/env bash

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly DEFAULT_WORKSPACE="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly REPLAY_ROOT="/co_3dto2d_replay"

workspace="${DEFAULT_WORKSPACE}"
bag=""
playback_rate="0.5"
domain_id="${ROS_DOMAIN_ID:-72}"
startup_delay_seconds="5"
sensor_warmup_seconds="10"
alignment_warmup_seconds="3"
alignment_period_seconds="2"
imu_source="auto"
dry_run=false
check_environment=false
loop_playback=false
launch_rviz=false
extra_launch_args=()

usage() {
  printf '%s\n' \
    "Usage: $0 --bag PATH [options]" \
    "" \
    "Replay one bag containing both r0 and r1 streams into two-live mapping." \
    "Only four selected LiDAR/IMU topics are replayed; recorded odometry, maps," \
    "alignment, /tf, and /tf_static are deliberately excluded." \
    "" \
    "Options:" \
    "  --bag PATH             Combined r0/r1 bag directory (required)" \
    "  --workspace PATH       Built workspace root (default: repository root)" \
    "  --rate RATE            Playback rate (default: 0.5)" \
    "  --domain-id ID         Isolated ROS domain (default: ROS_DOMAIN_ID or 72)" \
    "  --startup-delay SEC    Launch head start before bag playback (default: 5)" \
    "  --sensor-warmup SEC    Recorded-time warm-up before odometry starts (default: 10)" \
    "  --alignment-warmup SEC Recorded-time map accumulation before ICP (default: 3)" \
    "  --alignment-period SEC Recorded-time interval between ICP attempts (default: 2)" \
    "  --imu-source MODE      auto, raw, or filtered (default: auto)" \
    "  --launch-arg NAME:=VALUE" \
    "                         Additional two_live_mapping launch argument; repeatable" \
    "  --loop                 Replay continuously until Ctrl-C" \
    "  --rviz                 Start the two-robot RViz configuration" \
    "  --dry-run              Validate metadata and print commands only" \
    "  --check-environment    Validate ROS/workspace setup without launching" \
    "  -h, --help             Show this help"
}

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

while (($# > 0)); do
  case "$1" in
    --bag)
      (($# >= 2)) || die "--bag requires a path"
      bag="$2"
      shift 2
      ;;
    --workspace)
      (($# >= 2)) || die "--workspace requires a path"
      workspace="$2"
      shift 2
      ;;
    --rate)
      (($# >= 2)) || die "--rate requires a value"
      playback_rate="$2"
      shift 2
      ;;
    --domain-id)
      (($# >= 2)) || die "--domain-id requires a value"
      domain_id="$2"
      shift 2
      ;;
    --startup-delay)
      (($# >= 2)) || die "--startup-delay requires a value"
      startup_delay_seconds="$2"
      shift 2
      ;;
    --sensor-warmup)
      (($# >= 2)) || die "--sensor-warmup requires a value"
      sensor_warmup_seconds="$2"
      shift 2
      ;;
    --alignment-warmup)
      (($# >= 2)) || die "--alignment-warmup requires a value"
      alignment_warmup_seconds="$2"
      shift 2
      ;;
    --alignment-period)
      (($# >= 2)) || die "--alignment-period requires a value"
      alignment_period_seconds="$2"
      shift 2
      ;;
    --imu-source)
      (($# >= 2)) || die "--imu-source requires auto, raw, or filtered"
      imu_source="$2"
      shift 2
      ;;
    --launch-arg)
      (($# >= 2)) || die "--launch-arg requires NAME:=VALUE"
      [[ "$2" == *:=* ]] || die "--launch-arg must use NAME:=VALUE syntax"
      extra_launch_args+=("$2")
      shift 2
      ;;
    --loop)
      loop_playback=true
      shift
      ;;
    --rviz)
      launch_rviz=true
      shift
      ;;
    --dry-run)
      dry_run=true
      shift
      ;;
    --check-environment)
      check_environment=true
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

[[ -n "${bag}" ]] || die "--bag is required"
[[ "${playback_rate}" =~ ^[0-9]+([.][0-9]+)?$ ]] || die "rate must be a positive number"
[[ "${playback_rate}" != "0" && "${playback_rate}" != "0.0" ]] || die "rate must be greater than zero"
[[ "${domain_id}" =~ ^[0-9]+$ ]] || die "domain ID must be an integer"
((domain_id >= 0 && domain_id <= 232)) || die "domain ID must be between 0 and 232"
[[ "${startup_delay_seconds}" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
  die "startup delay must be a non-negative number"
[[ "${sensor_warmup_seconds}" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
  die "sensor warm-up must be a non-negative number"
[[ "${alignment_warmup_seconds}" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
  die "alignment warm-up must be a non-negative number"
[[ "${alignment_period_seconds}" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
  die "alignment period must be a positive number"
[[ "${alignment_period_seconds}" != "0" && "${alignment_period_seconds}" != "0.0" ]] ||
  die "alignment period must be greater than zero"
case "${imu_source}" in
  auto|raw|filtered) ;;
  *) die "--imu-source must be auto, raw, or filtered" ;;
esac

metadata="${bag}/metadata.yaml"
[[ -r "${metadata}" ]] || die "bag metadada is not readable: ${metadata}"

topic_metadata() {
  local metadata_path="$1"
  local wanted_topic="$2"
  awk -v wanted_topic="${wanted_topic}" '
    $1 == "name:" { current_topic = $2 }
    $1 == "type:" && current_topic == wanted_topic { message_type = $2 }
    $1 == "message_count:" && current_topic == wanted_topic {
      print message_type, $2
      exit
    }
  ' "${metadata_path}"
}

topic_count_or_zero() {
  local topic="$1"
  local expected_type="$2"
  local details
  local actual_type
  local message_count

  details="$(topic_metadata "${metadata}" "${topic}")"
  if [[ -z "${details}" ]]; then
    printf '0\n'
    return
  fi
  read -r actual_type message_count <<<"${details}"
  [[ "${actual_type}" == "${expected_type}" ]] ||
    die "${bag}: ${topic} has type ${actual_type}, expected ${expected_type}"
  [[ "${message_count}" =~ ^[0-9]+$ ]] ||
    die "${bag}: invalid message count for ${topic}: ${message_count}"
  printf '%s\n' "${message_count}"
}

require_sensor_topic() {
  local topic="$1"
  local expected_type="$2"
  local message_count

  message_count="$(topic_count_or_zero "${topic}" "${expected_type}")"
  ((message_count > 0)) || die "${bag}: missing or empty topic ${topic}"
  printf '%s\n' "${message_count}"
}

select_imu_topic() {
  local robot_id="$1"
  local raw_topic="/r${robot_id}/livox/imu"
  local filtered_topic="/r${robot_id}/mapping/imu_filtered"
  local raw_count
  local filtered_count

  raw_count="$(topic_count_or_zero "${raw_topic}" "sensor_msgs/msg/Imu")"
  filtered_count="$(topic_count_or_zero "${filtered_topic}" "sensor_msgs/msg/Imu")"

  case "${imu_source}" in
    raw)
      ((raw_count > 0)) || die "${bag}: missing or empty topic ${raw_topic}"
      printf '%s %s raw\n' "${raw_topic}" "${raw_count}"
      ;;
    filtered)
      ((filtered_count > 0)) || die "${bag}: missing or empty topic ${filtered_topic}"
      printf '%s %s filtered\n' "${filtered_topic}" "${filtered_count}"
      ;;
    auto)
      ((raw_count > 0 || filtered_count > 0)) ||
        die "${bag}: no usable IMU topic for r${robot_id}"
      if ((filtered_count > raw_count)); then
        printf '%s %s filtered\n' "${filtered_topic}" "${filtered_count}"
      else
        printf '%s %s raw\n' "${raw_topic}" "${raw_count}"
      fi
      ;;
  esac
}

storage_identifier() {
  awk '$1 == "storage_identifier:" { print $2; exit }' "${metadata}"
}

readonly R0_LIDAR_SOURCE="/r0/livox/lidar"
readonly R1_LIDAR_SOURCE="/r1/livox/lidar"
r0_lidar_count="$(require_sensor_topic "${R0_LIDAR_SOURCE}" "sensor_msgs/msg/PointCloud2")"
r1_lidar_count="$(require_sensor_topic "${R1_LIDAR_SOURCE}" "sensor_msgs/msg/PointCloud2")"
read -r r0_imu_source r0_imu_count r0_imu_stage <<<"$(select_imu_topic 0)"
read -r r1_imu_source r1_imu_count r1_imu_stage <<<"$(select_imu_topic 1)"

r0_imu_input_is_filtered=false
r1_imu_input_is_filtered=false
[[ "${r0_imu_stage}" == "filtered" ]] && r0_imu_input_is_filtered=true
[[ "${r1_imu_stage}" == "filtered" ]] && r1_imu_input_is_filtered=true

storage="$(storage_identifier)"
[[ -n "${storage}" ]] || storage="sqlite3"

readonly R0_LIDAR_TOPIC="${REPLAY_ROOT}/r0/lidar"
readonly R0_IMU_TOPIC="${REPLAY_ROOT}/r0/imu"
readonly R1_LIDAR_TOPIC="${REPLAY_ROOT}/r1/lidar"
readonly R1_IMU_TOPIC="${REPLAY_ROOT}/r1/imu"

mapping_startup_delay_seconds="$(
  awk \
    -v launch_lead="${startup_delay_seconds}" \
    -v warmup="${sensor_warmup_seconds}" \
    -v rate="${playback_rate}" \
    'BEGIN { printf "%.3f", launch_lead + warmup / rate }'
)"
alignment_startup_delay_seconds="$(
  awk -v warmup="${alignment_warmup_seconds}" -v rate="${playback_rate}" \
    'BEGIN { printf "%.3f", warmup / rate }'
)"
alignment_recompute_period_seconds="$(
  awk -v period="${alignment_period_seconds}" -v rate="${playback_rate}" \
    'BEGIN { printf "%.3f", period / rate }'
)"

mapping_command=(
  ros2 launch co_3dto2d_mapping two_live_combined_bag_mapping.launch.py
  "robot0_lidar_topic:=${R0_LIDAR_TOPIC}"
  "robot0_imu_topic:=${R0_IMU_TOPIC}"
  "robot1_lidar_topic:=${R1_LIDAR_TOPIC}"
  "robot1_imu_topic:=${R1_IMU_TOPIC}"
  "robot0_imu_input_is_filtered:=${r0_imu_input_is_filtered}"
  "robot1_imu_input_is_filtered:=${r1_imu_input_is_filtered}"
  "wait_imu_to_init:=false"
  "expected_update_rate:=0.0"
  "mapping_startup_delay_sec:=${mapping_startup_delay_seconds}"
  "alignment_startup_delay_sec:=${alignment_startup_delay_seconds}"
  "alignment_recompute_period_sec:=${alignment_recompute_period_seconds}"
)
mapping_command+=("${extra_launch_args[@]}")

bag_command=(
  ros2 bag play "${bag}" --storage "${storage}" --rate "${playback_rate}"
  --topics
  "${R0_LIDAR_SOURCE}"
  "${r0_imu_source}"
  "${R1_LIDAR_SOURCE}"
  "${r1_imu_source}"
  --remap
  "${R0_LIDAR_SOURCE}:=${R0_LIDAR_TOPIC}"
  "${r0_imu_source}:=${R0_IMU_TOPIC}"
  "${R1_LIDAR_SOURCE}:=${R1_LIDAR_TOPIC}"
  "${r1_imu_source}:=${R1_IMU_TOPIC}"
)
if ${loop_playback}; then
  bag_command+=(--loop)
fi

print_command() {
  local label="$1"
  shift
  printf '%s=' "${label}"
  printf ' %q' "$@"
  printf '\n'
}

printf '%s\n' \
  "ROS_DOMAIN_ID=${domain_id}" \
  "BAG=${bag}" \
  "INPUT_R0_LIDAR=${R0_LIDAR_SOURCE} (${r0_lidar_count} messages)" \
  "INPUT_R0_IMU=${r0_imu_source} (${r0_imu_count} messages, ${r0_imu_stage})" \
  "INPUT_R1_LIDAR=${R1_LIDAR_SOURCE} (${r1_lidar_count} messages)" \
  "INPUT_R1_IMU=${r1_imu_source} (${r1_imu_count} messages, ${r1_imu_stage})" \
  "R0_FILTERED_IMU_BYPASS=${r0_imu_input_is_filtered}" \
  "R1_FILTERED_IMU_BYPASS=${r1_imu_input_is_filtered}" \
  "RECORDED_SENSOR_WARMUP=${sensor_warmup_seconds}s" \
  "MAPPING_STARTUP_DELAY_WALL=${mapping_startup_delay_seconds}s" \
  "RECORDED_ALIGNMENT_WARMUP=${alignment_warmup_seconds}s" \
  "ALIGNMENT_STARTUP_DELAY_WALL=${alignment_startup_delay_seconds}s" \
  "RECORDED_ALIGNMENT_PERIOD=${alignment_period_seconds}s" \
  "ALIGNMENT_RECOMPUTE_PERIOD_WALL=${alignment_recompute_period_seconds}s"
print_command MAPPING "${mapping_command[@]}"
print_command BAG_PLAYER "${bag_command[@]}"

if ${dry_run}; then
  exit 0
fi

# shellcheck source=lib/two_live_runtime.bash
source "${SCRIPT_DIR}/lib/two_live_runtime.bash"
