#!/usr/bin/env bash

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly DEFAULT_DATASET_ROOT="/home/user/cslam_merge/data"
readonly DEFAULT_MAPPING_CONFIG="$(cd -- "${SCRIPT_DIR}/.." && pwd)/config/s3e_sparse_strict.yaml"

dataset_root="${DEFAULT_DATASET_ROOT}"
mapping_config="${DEFAULT_MAPPING_CONFIG}"
sequence="S3E_Laboratory_1"
robot0="Alpha"
robot1="Bob"
forwarded_args=()

usage() {
  printf '%s\n' \
    "Usage: $0 [S3E options] [mapping options]" \
    "" \
    "Replay two selected robots from one S3Ev1 sequence into two-robot mapping." \
    "" \
    "S3E options:" \
    "  --dataset-root PATH   Dataset root (default: ${DEFAULT_DATASET_ROOT})" \
    "  --config PATH         Mapping profile (default: config/s3e_sparse_strict.yaml)" \
    "  --sequence NAME       Sequence directory (default: S3E_Laboratory_1)" \
    "  --robot0 NAME         Alpha, Bob, or Carol (default: Alpha)" \
    "  --robot1 NAME         Alpha, Bob, or Carol (default: Bob)" \
    "  -h, --help            Show this help and common mapping options" \
    "" \
    "All other options are forwarded to run_two_live_combined_bag.sh." \
    "Example:" \
    "  $0 --sequence S3E_Square_1 --robot0 Alpha --robot1 Carol --rate 0.5 --rviz"
}

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

require_value() {
  local option="$1"
  local value="${2:-}"
  [[ -n "${value}" ]] || die "${option} requires a value"
}

normalize_robot() {
  case "${1,,}" in
    alpha) printf 'Alpha\n' ;;
    bob) printf 'Bob\n' ;;
    carol) printf 'Carol\n' ;;
    *) die "robot must be Alpha, Bob, or Carol: $1" ;;
  esac
}

while (($# > 0)); do
  case "$1" in
    --dataset-root)
      require_value "$1" "${2:-}"
      dataset_root="$2"
      shift 2
      ;;
    --config)
      require_value "$1" "${2:-}"
      mapping_config="$2"
      shift 2
      ;;
    --sequence)
      require_value "$1" "${2:-}"
      sequence="$2"
      shift 2
      ;;
    --robot0)
      require_value "$1" "${2:-}"
      robot0="$(normalize_robot "$2")"
      shift 2
      ;;
    --robot1)
      require_value "$1" "${2:-}"
      robot1="$(normalize_robot "$2")"
      shift 2
      ;;
    -h|--help)
      usage
      printf '\nCommon mapping options:\n'
      bash "${SCRIPT_DIR}/run_two_live_combined_bag.sh" --help
      exit 0
      ;;
    *)
      forwarded_args+=("$1")
      shift
      ;;
  esac
done

robot0="$(normalize_robot "${robot0}")"
robot1="$(normalize_robot "${robot1}")"
[[ "${robot0}" != "${robot1}" ]] || die "robot0 and robot1 must be different"
[[ "${sequence}" =~ ^S3E_[A-Za-z0-9_]+$ ]] || die "invalid S3Ev1 sequence name: ${sequence}"

readonly bag="${dataset_root%/}/${sequence}"
[[ -r "${bag}/metadata.yaml" ]] || die "sequence is not a readable ROS bag: ${bag}"
[[ -r "${mapping_config}" ]] || die "mapping config is not readable: ${mapping_config}"

printf '%s\n' \
  "S3E_DATASET_ROOT=${dataset_root}" \
  "S3E_SEQUENCE=${sequence}" \
  "S3E_R0=${robot0}" \
  "S3E_R1=${robot1}" \
  "S3E_CONFIG=${mapping_config}"

exec bash "${SCRIPT_DIR}/run_two_live_combined_bag.sh" \
  --bag "${bag}" \
  --robot0-lidar-source "/${robot0}/velodyne_points" \
  --robot0-imu-source "/${robot0}/imu/data" \
  --robot1-lidar-source "/${robot1}/velodyne_points" \
  --robot1-imu-source "/${robot1}/imu/data" \
  --launch-arg "occupancy_config_file:=${mapping_config}" \
  --launch-arg "alignment_config_file:=${mapping_config}" \
  --launch-arg sensor_tf_roll_0:=0.0 \
  --launch-arg sensor_tf_roll_1:=0.0 \
  "${forwarded_args[@]}"
