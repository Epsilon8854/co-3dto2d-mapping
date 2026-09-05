ros_setup="${ROS_SETUP:-}"
if [[ -z "${ros_setup}" && "${ROS_VERSION:-}" == "2" && -n "${ROS_DISTRO:-}" && -r "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  ros_setup="/opt/ros/${ROS_DISTRO}/setup.bash"
fi
if [[ -z "${ros_setup}" ]]; then
  for candidate in /opt/ros/humble/setup.bash /opt/ros/foxy/setup.bash; do
    if [[ -r "${candidate}" ]]; then
      ros_setup="${candidate}"
      break
    fi
  done
fi
[[ -r "${ros_setup}" ]] || die "ROS setup not found; set ROS_SETUP=/path/to/setup.bash"
[[ -r "${workspace}/install/local_setup.bash" ]] ||
  die "workspace is not built: ${workspace}/install/local_setup.bash is missing"

set +u
# shellcheck disable=SC1090
source "${ros_setup}"
# shellcheck disable=SC1091
source "${workspace}/install/local_setup.bash"
set -u
export PATH="/usr/bin:/bin:${PATH}"
hash -r
command -v ros2 >/dev/null || die "ros2 is unavailable after sourcing ${ros_setup}"
command -v setsid >/dev/null || die "setsid is required to manage the ROS process groups"
ros2 pkg prefix co_3dto2d_mapping >/dev/null || die "co_3dto2d_mapping is not installed"
python3 -c 'import rclpy' >/dev/null || die "system Python cannot import rclpy"

if ${check_environment}; then
  printf 'Environment check passed: ROS=%s workspace=%s Python=%s\n' \
    "${ros_setup}" "${workspace}" "$(command -v python3)"
  exit 0
fi

export ROS_DOMAIN_ID="${domain_id}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

log_directory="$(mktemp -d "${TMPDIR:-/tmp}/co3dto2d-two-live.XXXXXX")"
printf 'Logs: %s\n' "${log_directory}"

child_pids=()
cleanup() {
  local pid
  local attempt
  local groups_alive
  trap - EXIT INT TERM
  for pid in "${child_pids[@]}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || true
  done
  for attempt in {1..30}; do
    groups_alive=false
    for pid in "${child_pids[@]}"; do
      if kill -0 -- "-${pid}" 2>/dev/null; then
        groups_alive=true
      fi
    done
    if ! ${groups_alive}; then
      break
    fi
    sleep 0.1
  done
  for pid in "${child_pids[@]}"; do
    kill -KILL -- "-${pid}" 2>/dev/null || true
  done
  for pid in "${child_pids[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
}
trap cleanup EXIT
trap 'exit 130' INT TERM

setsid "${mapping_command[@]}" >"${log_directory}/mapping.log" 2>&1 &
mapping_pid=$!
child_pids+=("${mapping_pid}")

sleep "${startup_delay_seconds}"
if ! kill -0 "${mapping_pid}" 2>/dev/null; then
  printf 'Mapping launch exited during startup:\n' >&2
  tail -n 80 "${log_directory}/mapping.log" >&2
  exit 1
fi

if ${launch_rviz}; then
  setsid rviz2 -d "${workspace}/install/co_3dto2d_mapping/share/co_3dto2d_mapping/rviz/two_robot_mapping.rviz" \
    >"${log_directory}/rviz.log" 2>&1 &
  child_pids+=("$!")
fi

bag_pids=()
bag_logs=()
if declare -p bag_command >/dev/null 2>&1; then
  setsid "${bag_command[@]}" >"${log_directory}/bag.log" 2>&1 &
  bag_pids+=("$!")
  bag_logs+=("${log_directory}/bag.log")
else
  setsid "${bag0_command[@]}" >"${log_directory}/bag0.log" 2>&1 &
  bag_pids+=("$!")
  bag_logs+=("${log_directory}/bag0.log")
  setsid "${bag1_command[@]}" >"${log_directory}/bag1.log" 2>&1 &
  bag_pids+=("$!")
  bag_logs+=("${log_directory}/bag1.log")
fi
child_pids+=("${bag_pids[@]}")

printf '%s\n' \
  "Two-live replay is running on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}." \
  "Monitor: ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ros2 topic hz /r0/odom" \
  "Monitor: ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ros2 topic hz /r1/odom" \
  "Press Ctrl-C to stop."

bag_statuses=()
set +e
for bag_pid in "${bag_pids[@]}"; do
  wait "${bag_pid}"
  bag_statuses+=("$?")
done
set -e

if ${loop_playback}; then
  exit 0
fi

sleep 2
bag_failed=false
for bag_status in "${bag_statuses[@]}"; do
  if ((bag_status != 0)); then
    bag_failed=true
  fi
done
if ${bag_failed}; then
  printf 'Bag player failure; statuses:' >&2
  printf ' %s' "${bag_statuses[@]}" >&2
  printf '\n' >&2
  for bag_log in "${bag_logs[@]}"; do
    tail -n 40 "${bag_log}" >&2
  done
  exit 1
fi

if grep -Eq '\[ERROR\]|Traceback|process has died' "${log_directory}/mapping.log"; then
  printf 'Mapping reported runtime errors:\n' >&2
  grep -E '\[ERROR\]|Traceback|process has died' "${log_directory}/mapping.log" | tail -n 40 >&2
  exit 1
fi
if [[ "${require_place_recognition:-false}" == "true" ]]; then
  if ! grep -q 'Inter-robot occupancy alignment accepted' "${log_directory}/mapping.log"; then
    printf 'Mapping produced no accepted two-robot occupancy alignment. Logs: %s\n' \
      "${log_directory}" >&2
    exit 1
  fi
  printf 'Bag replay produced two-robot mapping and accepted occupancy alignment. Logs: %s\n' \
    "${log_directory}"
  exit 0
fi

if ! grep -q 'Startup alignment received:' "${log_directory}/mapping.log"; then
  printf 'Mapping produced no accepted startup ICP alignment. Logs: %s\n' \
    "${log_directory}" >&2
  exit 1
fi

printf 'Bag replay produced two-robot mapping with startup ICP alignment. Logs: %s\n' \
  "${log_directory}"
