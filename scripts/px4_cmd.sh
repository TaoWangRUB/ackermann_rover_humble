#!/usr/bin/env bash
# Run a PX4 NSH command via MAVLink SERIAL_CONTROL inside the Docker container.
#
# Usage (from host):
#   ./scripts/px4_cmd.sh "ver all"
#   ./scripts/px4_cmd.sh "commander status" 10
#   ./scripts/px4_cmd.sh "free"
#
# Args:
#   $1 — PX4 NSH command (required)
#   $2 — Timeout in seconds (optional, default: 8)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"
PX4_DEVICE="${PX4_DEVICE:-/dev/ttyACM0}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

CMD="${1:?Usage: $0 <nsh_command> [timeout]}"
TIMEOUT="${2:-8}"

if command -v lsof >/dev/null 2>&1; then
  serial_owners="$(lsof "${PX4_DEVICE}" 2>/dev/null | sed '1d' || true)"
  if [[ -n "${serial_owners}" ]]; then
    echo "ERROR: ${PX4_DEVICE} is already open by another host process." >&2
    echo "Close QGroundControl or any other serial client before running px4_cmd.sh." >&2
    echo >&2
    echo "Current owners:" >&2
    echo "${serial_owners}" >&2
    exit 1
  fi
fi

xdcomp exec ackermann_slam \
  python3 /workspace/scripts/px4_cmd.py "${CMD}" "${TIMEOUT}"
