#!/usr/bin/env bash
# Explicit opt-in build; historical implementations are excluded from colcon.
set -Eeuo pipefail
repo_dir="$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
set +u
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
set -u
build="${repo_dir}/outputs/experiment_support/legacy_mapper"
cmake -S "${repo_dir}/scripts/experiment/support/legacy_cpp" -B "${build}" -DCMAKE_BUILD_TYPE=Release
cmake --build "${build}" --parallel 2
printf 'Historical mapper: %s/occupancy_mapper\n' "${build}"
