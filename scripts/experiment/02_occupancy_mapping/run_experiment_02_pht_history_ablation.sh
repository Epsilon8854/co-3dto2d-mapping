#!/usr/bin/env bash

set -Eeuo pipefail

readonly SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
readonly REPOSITORY_DIR="$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
readonly EXPERIMENT_ID="experiment_02_occupancy_mapping"
readonly REPLAY_SCRIPT="${REPOSITORY_DIR}/scripts/experiment/02_occupancy_mapping/replay_experiment_02_occupancy_mapping.sh"

ablation_id="pht_history_hallway_$(date +%Y%m%d_%H%M%S)"
domain_id="${ROS_DOMAIN_ID:-72}"
canonical_mapper_input_bag=""

usage() {
  cat <<EOF
Usage: $(basename "$0") [--ablation-id ID] [--domain-id ID] [--canonical-mapper-input-bag DIR]

Replays the hallway bag four times. The only changed factor is the number of
prior keyframes accumulated as the PHT matching target: 1, 3, 5, and 10.
Every condition uses fixed-height pose correction, Full SE(2), and the 0.70 m
minimum PHT line-length profile. Each result includes a full-extent top-down
global map with raw (cyan) and corrected (orange) trajectories.
EOF
}

require_value() {
  [[ -n "${2:-}" ]] || { printf 'error: %s requires a value\n' "$1" >&2; exit 2; }
}

while (($# > 0)); do
  case "$1" in
    --ablation-id) require_value "$1" "${2:-}"; ablation_id="$2"; shift 2 ;;
    --domain-id) require_value "$1" "${2:-}"; domain_id="$2"; shift 2 ;;
    --canonical-mapper-input-bag) require_value "$1" "${2:-}"; canonical_mapper_input_bag="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'error: unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "${ablation_id}" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]] || { printf 'error: invalid ablation ID\n' >&2; exit 2; }
[[ "${domain_id}" =~ ^[0-9]+$ ]] || { printf 'error: ROS domain ID must be an integer\n' >&2; exit 2; }
if [[ -n "${canonical_mapper_input_bag}" ]]; then
  canonical_mapper_input_bag="$(readlink -f -- "${canonical_mapper_input_bag}")"
  [[ -r "${canonical_mapper_input_bag}/metadata.yaml" ]] || { printf 'error: canonical mapper-input bag is invalid\n' >&2; exit 1; }
fi

readonly ABLATION_DIR="${REPOSITORY_DIR}/outputs/${EXPERIMENT_ID}/ablations/${ablation_id}"
[[ ! -e "${ABLATION_DIR}" ]] || { printf 'error: ablation output already exists: %s\n' "${ABLATION_DIR}" >&2; exit 1; }
mkdir -p -- "${ABLATION_DIR}"

cat >"${ABLATION_DIR}/manifest.yaml" <<EOF
experiment_id: "${EXPERIMENT_ID}"
dataset: ${canonical_mapper_input_bag:-hallway}
pose_correction: fixed_height
pht_mode: full_se2
pht_profile: length_0_7
target_history_keyframes: [1, 3, 5, 10]
raw_trajectory_color: "#00dcff"
corrected_trajectory_color: "#ff9900"
EOF

for history in 1 3 5 10; do
  run_id="${ablation_id}_history_${history}"
  replay_input=(--dataset hallway)
  if [[ -n "${canonical_mapper_input_bag}" ]]; then
    replay_input=(--bag "${canonical_mapper_input_bag}" --canonical-mapper-input)
  fi
  "${REPLAY_SCRIPT}" \
    "${replay_input[@]}" \
    --pose-correction fixed_height \
    --local-pht-submap-pose-graph full_se2 \
    --pht-profile length_0_7 \
    --pht-history-keyframes "${history}" \
    --run-id "${run_id}" \
    --domain-id "${domain_id}" \
    --no-rviz \
    --exit-after-playback

  source_dir="${REPOSITORY_DIR}/outputs/${EXPERIMENT_ID}/runs/${run_id}"
  target_dir="${ABLATION_DIR}/history_${history}"
  mkdir -p -- "${target_dir}"
  cp -- "${source_dir}/topdown/${OUTPUT_FILENAME:-topdown_global_map_raw_cyan_corrected_orange.png}" "${target_dir}/"
  cp -- "${source_dir}/topdown/legend.yaml" "${source_dir}/run.yaml" "${source_dir}/pht_submap_pose_graph.csv" "${target_dir}/"
done

printf 'PHT target-history ablation complete: %s\n' "${ABLATION_DIR}"
