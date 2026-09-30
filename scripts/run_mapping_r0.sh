#!/usr/bin/env bash
# Run physical robot 1 as r0: Livox driver and local mapping only.

set -Eeuo pipefail

script_path="$(readlink -f -- "${BASH_SOURCE[0]}")"
script_dir="$(cd -- "$(dirname -- "${script_path}")" && pwd)"
runner="${script_dir}/run_two_mid360_2d_mapping.sh"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<EOF
Usage: $(basename "$0") [runner options]

Starts physical robot 1 as r0 with its Livox driver and local mapping.
The two-robot fusion host and RViz run on PC 2 instead.

Example:
  bash scripts/$(basename "$0")
EOF
  exit 0
fi

[[ -x "${runner}" ]] || {
  printf 'error: runner not found or not executable: %s\n' "${runner}" >&2
  exit 1
}

exec bash "${runner}" --robot-number 1 --local-mapping-only "$@"
