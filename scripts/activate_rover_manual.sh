#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/../docker/docker-compose.yml"
MODE_ID="${1:-23}"

echo "Requesting registered external mode ${MODE_ID} and arming PX4 (container: ackermann_slam)"

docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -lc \
  "source /opt/ros/\$ROS_DISTRO/setup.bash && source /workspace/install/setup.bash && \
   ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand '{command: 100001, param1: ${MODE_ID}.0, target_system: 1, target_component: 1, source_system: 255, source_component: 0, from_external: true}' && \
   sleep 0.5 && /px4/build/px4_sitl_default/bin/px4-commander arm"

printf 'Command published and arm request sent. Check vehicle status with:\n  pxh> listener vehicle_status\n'
