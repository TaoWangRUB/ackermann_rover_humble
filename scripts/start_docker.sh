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
source "${SCRIPT_DIR}/lib/dc.sh"

# Allow local X11 connections only when an X server is available.
if [[ -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null 2>&1; then
    xhost +local: || true
fi

if ! dcomp ps --services --filter status=running \
        | grep -q '^ackermann_slam$'; then
    echo "Container not running — starting ackermann_slam..."
    dcomp up -d ackermann_slam
fi

xdcomp exec ackermann_slam bash -c \
    'source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash && if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi && exec bash'
