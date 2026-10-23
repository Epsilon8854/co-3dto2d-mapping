#!/usr/bin/env bash
set -Eeuo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
set +u
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "${MAPPING_WORKSPACE:-${repo_dir}}/install/local_setup.bash"
set -u
exec python3 "${script_dir}/../../lib/runtime.py" record "$@"
