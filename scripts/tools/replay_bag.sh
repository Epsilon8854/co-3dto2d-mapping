#!/usr/bin/env bash
set -Eeuo pipefail
repo_dir="$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
set +u
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "${MAPPING_WORKSPACE:-${repo_dir}}/install/local_setup.bash"
set -u
# Default: display recorded final results, without recomputing mapping.
if [[ "${1:-}" == "--reprocess" ]]; then
  shift
  exec python3 "${repo_dir}/scripts/lib/runtime.py" replay "$@"
fi
exec python3 "${repo_dir}/scripts/tools/replay_results.py" "$@"
