#!/usr/bin/env bash

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly DEFAULT_WORKSPACE="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly DEFAULT_BAG0="/mnt/ssd1/aibot/airoom_1"
readonly DEFAULT_BAG1="/mnt/ssd1/aibot/airoom_pureR"
readonly SOURCE_LIDAR_TOPIC="/r1/livox/lidar"
readonly SOURCE_IMU_TOPIC="/r1/livox/imu"
readonly REPLAY_ROOT="/co_3dto2d_replay"

workspace="${DEFAULT_WORKSPACE}"
bag0="${DEFAULT_BAG0}"
bag1="${DEFAULT_BAG1}"
playback_rate="0.5"
domain_id="${ROS_DOMAIN_ID:-72}"
startup_delay_seconds="5"
dry_run=false
check_environment=false
loop_playback=false
launch_rviz=false

usage() {
  printf '%s\n' \
    "Usage: $0 [options]" \
    "" \
    "Replay airoom_1 and airoom_pureR as isolated inputs to two-live mapping." \
    "Only the recorded raw LiDAR and IMU topics are replayed." \
    "" \
    "Options:" \
    "  --bag0 PATH          Robot 0 bag (default: ${DEFAULT_BAG0})" \
    "  --bag1 PATH          Robot 1 bag (default: ${DEFAULT_BAG1})" \
    "  --workspace PATH     Built workspace root (default: repository root)" \
    "  --rate RATE          Playback rate (default: 0.5)" \
    "  --domain-id ID       Isolated ROS domain (default: ROS_DOMAIN_ID or 72)" \
    "  --startup-delay SEC  Mapping discovery delay (default: 5)" \
    "  --loop               Replay both bags repeatedly until Ctrl-C" \
    "  --rviz               Start the two-robot RViz configuration" \
    "  --dry-run            Validate bags and print commands only" \
    "  --check-environment  Validate ROS/workspace setup without launching" \
    "  -h, --help           Show this help"
}

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

while (($# > 0)); do
  case "$1" in
    --bag0)
      (($# >= 2)) || die "--bag0 requires a path"
      bag0="$2"
      shift 2
      ;;
    --bag1)
      (($# >= 2)) || die "--bag1 requires a path"
      bag1="$2"
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

[[ "${playback_rate}" =~ ^[0-9]+([.][0-9]+)?$ ]] || die "rate must be a positive number"
[[ "${playback_rate}" != "0" && "${playback_rate}" != "0.0" ]] || die "rate must be greater than zero"
[[ "${domain_id}" =~ ^[0-9]+$ ]] || die "domain ID must be an integer"
((domain_id >= 0 && domain_id <= 232)) || die "domain ID must be between 0 and 232"
[[ "${startup_delay_seconds}" =~ ^[0-9]+([.][0-9]+)?$ ]] || die "startup delay must be a non-negative number"

topic_metadata() {
  local metadata="$1"
  local wanted_topic="$2"
  awk -v wanted_topic="${wanted_topic}" '
    $1 == "name:" { current_topic = $2 }
    $1 == "type:" && current_topic == wanted_topic { message_type = $2 }
    $1 == "message_count:" && current_topic == wanted_topic {
      print message_type, $2
      exit
    }
  ' "${metadata}"
}

require_sensor_topic() {
  local bag_path="$1"
  local topic="$2"
  local expected_type="$3"
  local metadata="${bag_path}/metadata.yaml"
  local details
  local actual_type
  local message_count

  [[ -r "${metadata}" ]] || die "bag metadata is not readable: ${metadata}"
  details="$(topic_metadata "${metadata}" "${topic}")"
  [[ -n "${details}" ]] || die "${bag_path}: missing topic ${topic}"
  read -r actual_type message_count <<<"${details}"
  [[ "${actual_type}" == "${expected_type}" ]] ||
    die "${bag_path}: ${topic} has type ${actual_type}, expected ${expected_type}"
  ((message_count > 0)) || die "${bag_path}: ${topic} has no messages"
}

storage_identifier() {
  awk '$1 == "storage_identifier:" { print $2; exit }' "$1/metadata.yaml"
}

require_sensor_topic "${bag0}" "${SOURCE_LIDAR_TOPIC}" "sensor_msgs/msg/PointCloud2"
require_sensor_topic "${bag0}" "${SOURCE_IMU_TOPIC}" "sensor_msgs/msg/Imu"
require_sensor_topic "${bag1}" "${SOURCE_LIDAR_TOPIC}" "sensor_msgs/msg/PointCloud2"
require_sensor_topic "${bag1}" "${SOURCE_IMU_TOPIC}" "sensor_msgs/msg/Imu"

storage0="$(storage_identifier "${bag0}")"
storage1="$(storage_identifier "${bag1}")"
[[ -n "${storage0}" ]] || storage0="sqlite3"
[[ -n "${storage1}" ]] || storage1="sqlite3"

readonly R0_LIDAR_TOPIC="${REPLAY_ROOT}/r0/lidar"
readonly R0_IMU_TOPIC="${REPLAY_ROOT}/r0/imu"
readonly R1_LIDAR_TOPIC="${REPLAY_ROOT}/r1/lidar"
readonly R1_IMU_TOPIC="${REPLAY_ROOT}/r1/imu"

mapping_command=(
  ros2 launch co_3dto2d_mapping two_live_mapping.launch.py
  "robot0_lidar_topic:=${R0_LIDAR_TOPIC}"
  "robot0_imu_topic:=${R0_IMU_TOPIC}"
  "robot1_lidar_topic:=${R1_LIDAR_TOPIC}"
  "robot1_imu_topic:=${R1_IMU_TOPIC}"
  "wait_imu_to_init:=false"
)
bag0_command=(
  ros2 bag play "${bag0}" --storage "${storage0}" --rate "${playback_rate}"
  --topics "${SOURCE_LIDAR_TOPIC}" "${SOURCE_IMU_TOPIC}"
  --remap "${SOURCE_LIDAR_TOPIC}:=${R0_LIDAR_TOPIC}" "${SOURCE_IMU_TOPIC}:=${R0_IMU_TOPIC}"
)
bag1_command=(
  ros2 bag play "${bag1}" --storage "${storage1}" --rate "${playback_rate}"
  --topics "${SOURCE_LIDAR_TOPIC}" "${SOURCE_IMU_TOPIC}"
  --remap "${SOURCE_LIDAR_TOPIC}:=${R1_LIDAR_TOPIC}" "${SOURCE_IMU_TOPIC}:=${R1_IMU_TOPIC}"
)
if ${loop_playback}; then
  bag0_command+=(--loop)
  bag1_command+=(--loop)
fi

print_command() {
  local label="$1"
  shift
  printf '%s=' "${label}"
  printf ' %q' "$@"
  printf '\n'
}

printf 'ROS_DOMAIN_ID=%s\n' "${domain_id}"
print_command MAPPING "${mapping_command[@]}"
print_command BAG_PLAYER_0 "${bag0_command[@]}"
print_command BAG_PLAYER_1 "${bag1_command[@]}"

if ${dry_run}; then
  exit 0
fi

# shellcheck source=lib/two_live_runtime.bash
source "${SCRIPT_DIR}/lib/two_live_runtime.bash"
