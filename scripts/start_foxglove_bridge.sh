#!/usr/bin/env bash
# Start foxglove_bridge inside the Docker container so the Foxglove app on the
# host can show topics, TF, images and point clouds. This is the RViz
# replacement on hosts without an X server (macOS).
#
# Usage (from host):
#   ./scripts/start_foxglove_bridge.sh          # container port 8765
#   ./scripts/start_foxglove_bridge.sh 8770     # custom container port (Linux only)
#
# In Foxglove: Open connection -> Foxglove WebSocket -> ws://localhost:<port>
#   Linux: the container uses host networking, so <port> is the port above.
#   macOS: docker/docker-compose.mac.yml publishes container port 8765 as host
#          port FOXGLOVE_HOST_PORT (default 8766), so keep the default port
#          here and connect to ws://localhost:${FOXGLOVE_HOST_PORT:-8766}.
#
# Prerequisites:
#   1. Docker container running (./scripts/start_docker.sh or docker compose up -d)
#   2. ros-${ROS_DISTRO}-foxglove-bridge installed in the image
#      (docker/Dockerfile.mac does this; on Linux: sudo apt-get install ros-jazzy-foxglove-bridge)
set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"

PORT="${1:-8765}"

echo "Starting foxglove_bridge inside Docker container on port ${PORT}..."
if [[ "$(uname -s)" == "Darwin" ]]; then
    HOST_PORT="${FOXGLOVE_HOST_PORT:-8766}"
    if [[ "${PORT}" != "8765" ]]; then
        echo "  WARNING: on macOS only container port 8765 is published (as host port ${HOST_PORT});" >&2
        echo "           port ${PORT} will not be reachable from the Mac." >&2
    fi
    echo "  Foxglove: Open connection -> Foxglove WebSocket -> ws://localhost:${HOST_PORT}"
else
    echo "  Foxglove: Open connection -> Foxglove WebSocket -> ws://localhost:${PORT}"
fi
echo ""

CMD="source /opt/ros/\${ROS_DISTRO:-jazzy}/setup.bash && \
if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi && \
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=${PORT} address:=0.0.0.0"

if [ -t 0 ]; then
    xdcomp exec ackermann_slam bash -c "${CMD}"
else
    xdcomp exec -T ackermann_slam bash -c "${CMD}"
fi
