#!/usr/bin/env bash
# Start the Micro-XRCE-DDS Agent inside the Docker container.
# This bridges PX4's built-in uxrce_dds_client to ROS 2 DDS topics.
#
# Usage (from host):
#   ./scripts/start_microxrce_agent.sh                    # UDP mode (SITL), port 8888
#   ./scripts/start_microxrce_agent.sh 8889               # UDP mode, custom port
#   ./scripts/start_microxrce_agent.sh --serial            # Serial mode (real HW), /dev/ttyUSB0 @ 921600
#   ./scripts/start_microxrce_agent.sh --serial /dev/ttyACM0 115200   # Serial, custom device & baud
#
# Prerequisites:
#   1. Docker container must be running (docker-compose up -d)
#   2. MicroXRCEAgent must be installed in the container (built by Dockerfile)
#   3. For serial mode: the serial device must be passed through to the container
#      (add "devices: ['/dev/ttyUSB0:/dev/ttyUSB0']" to docker-compose.yml)
#
# IMPORTANT: Start the agent BEFORE PX4 — PX4's uxrce_dds_client connects
# on startup and does not reliably reconnect if the agent starts later. If PX4
# is already booted, restart uxrce_dds_client or reboot PX4 after launching the
# agent.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

if [[ "${1:-}" == "--serial" ]]; then
    # ── Serial mode (real hardware) ──────────────────────────────────────
    SERIAL_DEV="${2:-/dev/ttyUSB0}"
    SERIAL_BAUD="${3:-921600}"

    echo "Starting Micro-XRCE-DDS Agent (serial mode) inside Docker container..."
    echo "  Device:    ${SERIAL_DEV}"
    echo "  Baud rate: ${SERIAL_BAUD}"
    echo ""

    xdcomp exec ackermann_slam bash -c "
      export LD_LIBRARY_PATH=/opt/microxrce/lib:\${LD_LIBRARY_PATH:-}
      /opt/microxrce/bin/MicroXRCEAgent serial --dev ${SERIAL_DEV} -b ${SERIAL_BAUD}
    "
else
    # ── UDP mode (SITL simulation) ───────────────────────────────────────
    AGENT_PORT="${1:-8888}"

    echo "Starting Micro-XRCE-DDS Agent (UDP mode) inside Docker container..."
    echo "  UDP port: ${AGENT_PORT}"
    echo ""

    xdcomp exec ackermann_slam bash -c "
      export LD_LIBRARY_PATH=/opt/microxrce/lib:\${LD_LIBRARY_PATH:-}
      /opt/microxrce/bin/MicroXRCEAgent udp4 -p ${AGENT_PORT}
    "
fi
