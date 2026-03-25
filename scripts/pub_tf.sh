#!/usr/bin/env bash
# Publish static transforms to satisfy the px4_vision_odom bridge:
#   odom -> ackermann/base_link           (identity)
#   ackermann/base_link -> cubepilot_link (URDF offset: x=0.087 y=0 z=0.10)
set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${PROJECT_DIR}/docker/docker-compose.yml"
ENV_FILE="${PROJECT_DIR}/.env"

echo "Publishing static TFs inside Docker container..."
echo "  odom -> ackermann/base_link (identity)"
echo "  ackermann/base_link -> cubepilot_link (x=0.087 z=0.10)"

exec docker-compose --env-file "${ENV_FILE}" -f "${COMPOSE_FILE}" exec ackermann_slam bash -c "
  source /opt/ros/jazzy/setup.bash
  source /workspace/install/setup.bash
  # Args: x y z yaw pitch roll frame_id child_frame_id
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom ackermann/base_link &
  ros2 run tf2_ros static_transform_publisher 0.087 0 0.10 0 0 0 ackermann/base_link cubepilot_link &
  wait
"
