#!/usr/bin/env bash
# r1: owned driver + robot pipeline; no central fusion.
set -Eeuo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${script_dir}/run_experiment_04_multi_agent_mapping.sh" --role robot --robot-id 1 "$@"
