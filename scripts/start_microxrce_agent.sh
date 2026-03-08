#!/usr/bin/env bash
# Start the Micro-XRCE-DDS Agent inside the Docker container.
# This bridges PX4's built-in uxrce_dds_client to ROS 2 DDS topics.
#
# Usage (from host):
#   ./scripts/start_microxrce_agent.sh          # start agent on default port 8888
#   ./scripts/start_microxrce_agent.sh 8889     # start agent on custom port
#
# Prerequisites:
#   1. Docker container must be running (docker-compose up -d)
#   2. MicroXRCEAgent must be installed in the container (built by Dockerfile)
#
# IMPORTANT: Start the agent BEFORE PX4 SITL — PX4's uxrce_dds_client connects
# on startup and does not reliably reconnect if the agent starts later.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${PROJECT_DIR}/docker/docker-compose.yml"

AGENT_PORT="${1:-8888}"

echo "Starting Micro-XRCE-DDS Agent inside Docker container..."
echo "  UDP port: ${AGENT_PORT}"
echo ""

exec docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -c "
  export LD_LIBRARY_PATH=/opt/microxrce/lib:\${LD_LIBRARY_PATH:-}
  /opt/microxrce/bin/MicroXRCEAgent udp4 -p ${AGENT_PORT}
"
