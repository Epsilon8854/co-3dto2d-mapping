#!/usr/bin/env bash

set -Eeuo pipefail

readonly SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
readonly REPOSITORY_DIR="$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
readonly EXPERIMENT_ID="experiment_02_occupancy_mapping"
readonly REPLAY_SCRIPT="${REPOSITORY_DIR}/scripts/experiment/02_occupancy_mapping/replay_experiment_02_occupancy_mapping.sh"

study_id="icp_reproducibility_hallway_$(date +%Y%m%d_%H%M%S)"
domain_id="${ROS_DOMAIN_ID:-72}"
icp_topic_queue_size="1"
replay_rate="1.0"

usage() {
  cat <<EOF
Usage: $(basename "$0") [--study-id ID] [--domain-id ID] [--icp-topic-queue-size COUNT] [--rate RATE]

Run the hallway bag twice with PHT disabled and the same RTAB-Map ICP scan
queue. The output compares direct ICP and ground-node synchronized raw
odometry at matching timestamps before any PHT target-history result is
interpreted.
EOF
}

require_value() { [[ -n "${2:-}" ]] || { printf 'error: %s requires a value\n' "$1" >&2; exit 2; }; }

while (($# > 0)); do
  case "$1" in
    --study-id) require_value "$1" "${2:-}"; study_id="$2"; shift 2 ;;
    --domain-id) require_value "$1" "${2:-}"; domain_id="$2"; shift 2 ;;
    --icp-topic-queue-size) require_value "$1" "${2:-}"; icp_topic_queue_size="$2"; shift 2 ;;
    --rate) require_value "$1" "${2:-}"; replay_rate="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'error: unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "${study_id}" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]] || { printf 'error: invalid study ID\n' >&2; exit 2; }
[[ "${domain_id}" =~ ^[0-9]+$ && "${icp_topic_queue_size}" =~ ^[1-9][0-9]*$ ]] || { printf 'error: invalid numeric option\n' >&2; exit 2; }
[[ "${replay_rate}" =~ ^[0-9]+([.][0-9]+)?$ ]] && awk "BEGIN { exit !(${replay_rate} > 0) }" || { printf 'error: --rate must be positive\n' >&2; exit 2; }

readonly STUDY_DIR="${REPOSITORY_DIR}/outputs/${EXPERIMENT_ID}/ablations/${study_id}"
[[ ! -e "${STUDY_DIR}" ]] || { printf 'error: output exists: %s\n' "${STUDY_DIR}" >&2; exit 1; }
mkdir -p -- "${STUDY_DIR}"

for repeat in 1 2; do
  run_id="${study_id}_repeat_${repeat}"
  "${REPLAY_SCRIPT}" \
    --dataset hallway \
    --pose-correction fixed_height \
    --local-pht-submap-pose-graph off \
    --icp-topic-queue-size "${icp_topic_queue_size}" \
    --rate "${replay_rate}" \
    --run-id "${run_id}" \
    --domain-id "${domain_id}" \
    --no-rviz \
    --exit-after-playback
  source_dir="${REPOSITORY_DIR}/outputs/${EXPERIMENT_ID}/runs/${run_id}/topdown"
  target_dir="${STUDY_DIR}/repeat_${repeat}"
  mkdir -p -- "${target_dir}"
  cp -- "${source_dir}/raw_odometry_xy.csv" "${source_dir}/icp_odometry_xy.csv" \
    "${source_dir}/corrected_odometry_xy.csv" \
    "${source_dir}/topdown_global_map_raw_cyan_corrected_orange.png" \
    "${source_dir}/legend.yaml" "${target_dir}/"
  cp -- "${REPOSITORY_DIR}/outputs/${EXPERIMENT_ID}/runs/${run_id}/logs/launch_cycle_0001.log" \
    "${target_dir}/launch_cycle_0001.log"
done

python3 - "${STUDY_DIR}" "${icp_topic_queue_size}" "${replay_rate}" <<'PY'
import csv
import math
import re
import sys
from pathlib import Path

study = Path(sys.argv[1])
queue_size = int(sys.argv[2])
replay_rate = float(sys.argv[3])

def read(repeat, filename):
    with (study / repeat / filename).open(newline="", encoding="utf-8") as stream:
        return {
            int(row["stamp_ns"]): (float(row["x_m"]), float(row["y_m"]))
            for row in csv.DictReader(stream)
        }

def reproducibility(filename):
    first, second = read("repeat_1", filename), read("repeat_2", filename)
    common = sorted(set(first) & set(second))
    distances = [
        math.hypot(first[stamp][0] - second[stamp][0], first[stamp][1] - second[stamp][1])
        for stamp in common
    ]
    return {
        "repeat_1_samples": len(first),
        "repeat_2_samples": len(second),
        "common_timestamps": len(common),
        "only_repeat_1": len(set(first) - set(second)),
        "only_repeat_2": len(set(second) - set(first)),
        "max_common_xy_delta_m": max(distances) if distances else 0.0,
        "mean_common_xy_delta_m": sum(distances) / len(distances) if distances else 0.0,
    }

timing_pattern = re.compile(r"update time=([0-9.]+)s delay=([0-9.]+)s")

def percentile(values, fraction):
    if not values:
        return 0.0
    sorted_values = sorted(values)
    return sorted_values[min(len(sorted_values) - 1, round((len(sorted_values) - 1) * fraction))]

def timing(repeat):
    values = [
        (float(match.group(1)), float(match.group(2)))
        for match in timing_pattern.finditer((study / repeat / "launch_cycle_0001.log").read_text(encoding="utf-8", errors="replace"))
    ]
    def describe(index):
        samples = [value[index] for value in values]
        return {
            "count": len(samples),
            "mean_s": sum(samples) / len(samples) if samples else 0.0,
            "p95_s": percentile(samples, 0.95),
            "max_s": max(samples) if samples else 0.0,
        }
    return {"update_time": describe(0), "delay": describe(1)}

def write_mapping(stream, data, indent=0):
    for key, value in data.items():
        if isinstance(value, dict):
            stream.write(" " * indent + f"{key}:\n")
            write_mapping(stream, value, indent + 2)
        elif isinstance(value, float):
            stream.write(" " * indent + f"{key}: {value:.9f}\n")
        else:
            stream.write(" " * indent + f"{key}: {value}\n")

summary = {
    "icp_topic_queue_size": queue_size,
    "playback_rate": replay_rate,
    "direct_icp_odometry": reproducibility("icp_odometry_xy.csv"),
    "ground_node_synchronized_raw_odometry": reproducibility("raw_odometry_xy.csv"),
    "rtabmap_reported_timing": {
        "repeat_1": timing("repeat_1"),
        "repeat_2": timing("repeat_2"),
    },
}
with (study / "raw_odometry_reproducibility.yaml").open("w", encoding="utf-8") as stream:
    write_mapping(stream, summary)
PY

printf 'ICP reproducibility study complete: %s\n' "${STUDY_DIR}"
