#!/usr/bin/env bash
set -Eeuo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "${script_dir}/.." && pwd)"
set +u
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "${MAPPING_WORKSPACE:-${repo_dir}}/install/local_setup.bash"
set -u
exec python3 "${script_dir}/experiment_04_runtime.py" replay "$@"
