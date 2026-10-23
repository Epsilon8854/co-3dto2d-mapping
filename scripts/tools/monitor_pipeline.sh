#!/usr/bin/env bash
# Default: observe r1 locally for 40 seconds; no mapping restart/build needed.
set -Eeuo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
role=local
robot_id=1
domain_id=72
duration=
output=
usage() {
  cat <<'USAGE'
Usage: bash scripts/tools/monitor_pipeline.sh [options]
  --role local|receiver   local on r1 PC; receiver on r0 PC (default: local)
  --robot-id 0|1          Robot whose pipeline is observed (default: 1)
  --domain-id ID          ROS domain (default: 72)
  --duration SECONDS      Default: local 40, receiver 60
  --output PATH          Default: timestamped JSON under outputs/multi_agent/diagnostics/
  -h, --help             Show this help
USAGE
}
while (($#)); do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --role|--robot-id|--domain-id|--duration|--output)
      if (($# < 2)) || [[ -z "$2" || "$2" == --* ]]; then
        echo "Missing value for $1" >&2; exit 2
      fi
      case "$1" in
        --role) role="$2" ;;
        --robot-id) robot_id="$2" ;;
        --domain-id) domain_id="$2" ;;
        --duration) duration="$2" ;;
        --output) output="$2" ;;
      esac
      shift 2 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done
if [[ "$role" != local && "$role" != receiver ]]; then echo 'Invalid role' >&2; exit 2; fi
if [[ "$robot_id" != 0 && "$robot_id" != 1 ]]; then echo 'Invalid robot ID' >&2; exit 2; fi
if [[ ! "$domain_id" =~ ^[0-9]+$ ]]; then echo 'Invalid domain ID' >&2; exit 2; fi
if [[ -z "$duration" ]]; then
  if [[ "$role" == local ]]; then duration=40; else duration=60; fi
fi
if [[ -z "$output" ]]; then
  run_id="$(date +%Y%m%d_%H%M%S_%N)"
  output="${repo_dir}/outputs/multi_agent/diagnostics/${run_id}/${role}_r${robot_id}.json"
fi
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "$ros_setup" ]]; then echo "ROS setup not found: $ros_setup" >&2; exit 1; fi
set +u
source "$ros_setup"
set -u
export ROS_DOMAIN_ID="$domain_id"
export ROS_LOCALHOST_ONLY=0
printf 'Pipeline monitor: role=%s robot=r%s domain=%s duration=%ss\nOutput: %s\n' \
  "$role" "$robot_id" "$domain_id" "$duration" "$output"
exec python3 "${script_dir}/monitor_pipeline.py" \
  --role "$role" --robot-id "$robot_id" --duration "$duration" --output "$output"
