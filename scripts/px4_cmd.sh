#!/usr/bin/env bash
# Run a PX4 NSH command via MAVLink SERIAL_CONTROL inside the Docker container.
#
# Usage (from host):
#   ./scripts/px4_cmd.sh "ver all"
#   ./scripts/px4_cmd.sh "commander status" 10
#   ./scripts/px4_cmd.sh "free"
#   PX4_USE_MAVPROXY=0 ./scripts/px4_cmd.sh "ver all" 20   # force direct USB mode
#
# Args:
#   $1 — PX4 NSH command (required)
#   $2 — Timeout in seconds (optional, default: 8)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"
PX4_DEVICE_FROM_ENV=false
if [[ -n "${PX4_DEVICE+x}" ]]; then
  PX4_DEVICE_FROM_ENV=true
fi
PX4_DEVICE="${PX4_DEVICE:-/dev/ttyACM0}"
PX4_BAUD="${PX4_BAUD:-57600}"
PX4_ASSERT_DTR="${PX4_ASSERT_DTR:-1}"
PX4_USE_MAVPROXY="${PX4_USE_MAVPROXY:-1}"
PX4_MAVPROXY_DEVICE="${PX4_MAVPROXY_DEVICE:-udpin:127.0.0.1:14550}"

resolve_px4_device() {
  local requested="$1"
  local prefer_by_id="$2"
  local by_id

  by_id=$(compgen -G '/dev/serial/by-id/*PX4*' | head -n 1 || true)

  if [[ "${prefer_by_id}" == true && -n "${by_id}" ]]; then
    printf '%s\n' "${by_id}"
    return 0
  fi

  if [[ -e "${requested}" ]]; then
    printf '%s\n' "${requested}"
    return 0
  fi

  if [[ -n "${by_id}" ]]; then
    printf '%s\n' "${by_id}"
    return 0
  fi

  printf '%s\n' "${requested}"
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

CMD="${1:?Usage: $0 <nsh_command> [timeout]}"
TIMEOUT="${2:-8}"

if [[ "${PX4_USE_MAVPROXY}" == "1" ]]; then
  "${SCRIPT_DIR}/start_px4_mavproxy_bridge.sh" >/dev/null
  PX4_DEVICE="${PX4_MAVPROXY_DEVICE}"
  PX4_ASSERT_DTR=0
else
  PX4_DEVICE="$(resolve_px4_device "${PX4_DEVICE}" "$([[ "${PX4_DEVICE_FROM_ENV}" == false ]] && echo true || echo false)")"
fi

if [[ "${PX4_DEVICE}" == /dev/* ]] && command -v lsof >/dev/null 2>&1; then
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
  bash -lc 'PX4_DEVICE="$1" PX4_BAUD="$2" PX4_ASSERT_DTR="$3" python3 /workspace/scripts/px4_cmd.py "$4" "$5"' \
  _ "${PX4_DEVICE}" "${PX4_BAUD}" "${PX4_ASSERT_DTR}" "${CMD}" "${TIMEOUT}"
