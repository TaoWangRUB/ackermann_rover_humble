#!/usr/bin/env bash
# Stop all simulation processes (PX4 on host + ROS/Gazebo in Docker).
# Prevents stale processes, /clock conflicts, and TF time jumps.
#
# Usage:
#   ./scripts/stop_all.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
COMPOSE_FILE="${WORKSPACE_DIR}/docker/docker-compose.yml"

echo "Stopping all simulation processes in Docker (PX4 + ROS 2 + Gazebo)..."
docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam \
    bash -c "pkill -9 -f 'ros2|gz|ruby|px4'" 2>/dev/null || true

echo "All processes stopped."
