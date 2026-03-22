#!/usr/bin/env bash
# Stop all ROS 2 / Gazebo / PX4 processes inside the Docker container.
# Kills both the ros2 launch parents AND orphaned child nodes.
#
# Usage:
#   ./scripts/stop_all.sh
set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
COMPOSE_FILE="$(dirname "$SCRIPT_DIR")/docker/docker-compose.yml"

DC="docker-compose -f ${COMPOSE_FILE}"

# Check container is running
if ! ${DC} ps --status running 2>/dev/null | grep -q ackermann_slam; then
    echo "Container is not running."
    exit 0
fi

echo "Stopping all processes in container..."

# Pass 1: kill launch parents and well-known process names
${DC} exec -T ackermann_slam bash -c "
    pkill -9 -f 'ros2 launch' 2>/dev/null || true
    pkill -9 -f 'rviz2|gz sim|ruby|MicroXRCE' 2>/dev/null || true
    sleep 1
" 2>/dev/null || true

# Pass 2: kill any remaining ROS / workspace nodes (catches orphans)
${DC} exec -T ackermann_slam bash -c "
    ps aux --no-headers \
      | grep -E '/opt/ros|/workspace/install' \
      | grep -v grep \
      | awk '{print \$2}' \
      | xargs kill -9 2>/dev/null || true
" 2>/dev/null || true

# Verify
REMAINING=$(${DC} exec -T ackermann_slam bash -c "
    ps aux --no-headers \
      | grep -E '/opt/ros|/workspace/install' \
      | grep -v grep \
      | wc -l
" 2>/dev/null || echo "0")

if [[ "${REMAINING}" -eq 0 ]]; then
    echo "All processes stopped."
else
    echo "WARNING: ${REMAINING} process(es) still running. Retrying..."
    ${DC} exec -T ackermann_slam bash -c "
        ps aux --no-headers \
          | grep -E '/opt/ros|/workspace/install' \
          | grep -v grep \
          | awk '{print \$2}' \
          | xargs kill -9 2>/dev/null || true
    " 2>/dev/null || true
    echo "Done."
fi
