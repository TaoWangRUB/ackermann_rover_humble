#!/usr/bin/env bash
# Open an interactive shell inside the ackermann_slam Docker container.
# Starts the container if it is not already running.
#
# Usage:
#   ./scripts/start_docker.sh
set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/../docker/docker-compose.yml"

xhost +local:root

if ! docker-compose -f "${COMPOSE_FILE}" ps --services --filter status=running \
        | grep -q '^ackermann_slam$'; then
    echo "Container not running — starting ackermann_slam..."
    docker-compose -f "${COMPOSE_FILE}" up -d ackermann_slam
fi

exec docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -c \
    'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && exec bash'
