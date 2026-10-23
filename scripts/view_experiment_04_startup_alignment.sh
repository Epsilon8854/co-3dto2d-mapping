#!/usr/bin/env bash
# Attach to a running fusion directory without restarting mapping.
set -Eeuo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "${script_dir}/.." && pwd)"
set +u
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ -f "${repo_dir}/install/local_setup.bash" ]]; then
  source "${repo_dir}/install/local_setup.bash"
fi
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-72}"
export ROS_LOCALHOST_ONLY=0
exec python3 "${script_dir}/experiment_04_alignment_viewer.py" "$@"
