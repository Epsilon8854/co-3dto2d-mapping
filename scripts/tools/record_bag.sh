#!/usr/bin/env bash
# Run once on r0; never starts/stops either robot's mapping or LiDAR driver.
set -Eeuo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
set +u
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ -f "${repo_dir}/install/local_setup.bash" ]]; then
  source "${repo_dir}/install/local_setup.bash"
fi
set -u
exec python3 "${script_dir}/record_bag.py" "$@"
