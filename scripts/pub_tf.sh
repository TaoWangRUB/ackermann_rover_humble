#!/usr/bin/env bash
# Publish a static transform from odom to ackermann/base_link to satisfy the px4_vision_odom bridge
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${PROJECT_DIR}/docker/docker-compose.yml"

echo "Publishing static TF: odom -> ackermann/base_link (0 offset) inside Docker container..."

exec docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -c "
  source /opt/ros/jazzy/setup.bash
  source /workspace/install/setup.bash
  # Args: x y z yaw pitch roll frame_id child_frame_id
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom ackermann/base_link
"
