#!/usr/bin/env bash
# Small source-only adapter for old experiment live wrappers.
set -Eeuo pipefail
repo_dir="$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
args=(use_bag:=false use_sim_time:=false robot_id:=0 "occupancy_config_file:=${MAPPING_CONFIG:?experiment profile required}")
while (($#)); do
  case "$1" in
    --domain-id) export ROS_DOMAIN_ID="$2";shift 2 ;;
    --driver-config) export LIVOX_CONFIG="$2";shift 2 ;;
    --livox-workspace) export LIVOX_SETUP="$2/install/local_setup.bash";shift 2 ;;
    --odometry-config) args+=("odometry_config_file:=$2");shift 2 ;;
    --launch-arg) args+=("${2/sensor_tf_roll_0/sensor_tf_roll}");shift 2 ;;
    *) printf 'Unknown historical adapter option: %s\n' "$1" >&2;exit 2 ;;
  esac
done
if [[ ! -x "${repo_dir}/outputs/experiment_support/legacy_mapper/occupancy_mapper" ]];then
  printf 'Build historical mapper first: bash scripts/experiment/support/build_legacy_mapper.sh\n' >&2;exit 1
fi
set +u
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "${MAPPING_WORKSPACE:-${repo_dir}}/install/local_setup.bash"
set -u
export PYTHONPATH="${repo_dir}${PYTHONPATH:+:${PYTHONPATH}}"
pids=()
cleanup(){
  trap - EXIT INT TERM
  for pid in "${pids[@]}";do kill -INT -- "-${pid}" 2>/dev/null || true;done
  for pid in "${pids[@]}";do wait "${pid}" 2>/dev/null || true;done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
setsid bash "${repo_dir}/scripts/lib/run_livox_driver.sh" 1 & pids+=("$!")
setsid ros2 launch "${repo_dir}/scripts/experiment/support/launch/single_bag_mapping.launch.py" "${args[@]}" & pids+=("$!")
if [[ -n "${OCCUPANCY_PNG_OUTPUT_DIR:-}" ]];then
 setsid ros2 run co_3dto2d_mapping single_agent_map_saver.py --ros-args -p "output_directory:=${OCCUPANCY_PNG_OUTPUT_DIR}" & pids+=("$!")
fi
wait -n "${pids[@]}"
