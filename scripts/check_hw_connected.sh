#!/usr/bin/env bash
# Poll for CubePilot Black serial device until it appears.
# Usage:
#   ./scripts/check_hw_connected.sh                  # default /dev/ttyUSB0, 60s timeout
#   ./scripts/check_hw_connected.sh /dev/ttyACM0 30  # custom device, 30s timeout
set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

DEVICE="${1:-/dev/ttyUSB0}"
TIMEOUT="${2:-60}"
INTERVAL=2
ELAPSED=0

echo "Waiting for CubePilot on ${DEVICE} (timeout: ${TIMEOUT}s)..."

while ! test -c "${DEVICE}"; do
    if [ "${ELAPSED}" -ge "${TIMEOUT}" ]; then
        echo "ERROR: ${DEVICE} not found after ${TIMEOUT}s"
        exit 1
    fi
    sleep "${INTERVAL}"
    ELAPSED=$((ELAPSED + INTERVAL))
    echo "  ... ${ELAPSED}s elapsed"
done

echo ""
echo "=== CubePilot detected on ${DEVICE} ==="
ls -l "${DEVICE}"
echo ""
udevadm info --name="${DEVICE}" 2>/dev/null | grep -E 'ID_VENDOR|ID_MODEL|ID_SERIAL' || true
echo ""
echo "Ready for DDS agent."
