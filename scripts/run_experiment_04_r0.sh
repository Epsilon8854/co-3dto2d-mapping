#!/usr/bin/env bash
# r0: owned driver + robot pipeline + central fusion + RViz.
set -Eeuo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${script_dir}/run_experiment_04_multi_agent_mapping.sh" --role all --robot-id 0 "$@"
