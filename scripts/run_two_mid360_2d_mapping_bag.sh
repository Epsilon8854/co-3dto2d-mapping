#!/usr/bin/env bash

set -Eeuo pipefail

readonly SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${SCRIPT_PATH}")" && pwd)"
readonly DEFAULT_BAG="/mnt/ssd1/aibot/airoom_0901/260901_r2/airoom_chair_r2"
readonly COMBINED_RUNNER="${SCRIPT_DIR}/run_two_live_combined_bag.sh"
readonly AIROOM_ALIGNMENT_PROFILE_NAME="airoom_chair_replay_place_recognition.yaml"

if [[ -r "${SCRIPT_DIR}/../config/${AIROOM_ALIGNMENT_PROFILE_NAME}" ]]; then
  readonly AIROOM_ALIGNMENT_PROFILE="$(readlink -f -- "${SCRIPT_DIR}/../config/${AIROOM_ALIGNMENT_PROFILE_NAME}")"
else
  readonly AIROOM_ALIGNMENT_PROFILE="$(readlink -f -- "${SCRIPT_DIR}/../../share/co_3dto2d_mapping/config/${AIROOM_ALIGNMENT_PROFILE_NAME}")"
fi

bag_path="${TWO_MID360_BAG_PATH:-${DEFAULT_BAG}}"
domain_id="${TWO_MID360_BAG_DOMAIN_ID:-${ROS_DOMAIN_ID:-173}}"
playback_rate="${TWO_MID360_BAG_RATE:-0.5}"
forwarded_arguments=()

usage() {
  printf '%s\n' \
    "Usage: $0 [options]" \
    "" \
    "Replay the default two-robot MID-360 bag through the combined-bag pipeline." \
    "The combined runner replays only r0/r1 LiDAR and IMU inputs, then rebuilds" \
    "odometry, alignment, occupancy, and the merged map from scratch." \
    "This shortcut uses an airoom_chair_r2 alignment profile for its static r1." \
    "" \
    "Wrapper options:" \
    "  --bag PATH          Bag directory (default: ${DEFAULT_BAG})" \
    "  --domain-id ID      Isolated ROS domain (default: ROS_DOMAIN_ID or 173)" \
    "  --rate RATE         Playback rate (default: 0.5)" \
    "  -h, --help          Show this help" \
    "" \
    "All other options are passed to run_two_live_combined_bag.sh, including" \
    "--dry-run, --rviz, --imu-source, --launch-arg, and --workspace."
}

require_value() {
  local option="$1"
  local value="${2:-}"
  [[ -n "${value}" ]] || {
    printf 'error: %s requires a value\n' "${option}" >&2
    exit 1
  }
}

while (($# > 0)); do
  case "$1" in
    --bag)
      require_value "$1" "${2:-}"
      bag_path="$2"
      shift 2
      ;;
    --domain-id)
      require_value "$1" "${2:-}"
      domain_id="$2"
      shift 2
      ;;
    --rate)
      require_value "$1" "${2:-}"
      playback_rate="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      forwarded_arguments+=("$1")
      shift
      ;;
  esac
done

[[ -x "${COMBINED_RUNNER}" ]] || {
  printf 'error: combined bag runner is not executable: %s\n' "${COMBINED_RUNNER}" >&2
  exit 1
}
[[ -r "${AIROOM_ALIGNMENT_PROFILE}" ]] || {
  printf 'error: airoom alignment profile is not readable: %s\n' \
    "${AIROOM_ALIGNMENT_PROFILE}" >&2
  exit 1
}

exec bash "${COMBINED_RUNNER}" \
  --bag "${bag_path}" \
  --domain-id "${domain_id}" \
  --rate "${playback_rate}" \
  --launch-arg "alignment_config_file:=${AIROOM_ALIGNMENT_PROFILE}" \
  "${forwarded_arguments[@]}"
