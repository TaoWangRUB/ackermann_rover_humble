#!/usr/bin/env bash
# Activate a PX4 registered external mode and arm the rover.
#
# Usage:
#   ./scripts/activate_rover_manual.sh          # default mode ID 23
#   ./scripts/activate_rover_manual.sh 24       # custom mode ID
#
# Publishes VehicleCommand to switch PX4 into the specified external mode,
# then sends an arm request via ROS 2.
# Works with both PX4 SITL (Gazebo) and real hardware (Cube Black).
set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/../docker/docker-compose.yml"
MODE_ID="${1:-23}"

SOURCE_CMD="source /opt/ros/\$ROS_DISTRO/setup.bash && source /workspace/install/setup.bash"

echo "Requesting registered external mode ${MODE_ID} and arming PX4 (container: ackermann_slam)"

# 1. Switch to the registered external mode
docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -lc \
  "${SOURCE_CMD} && \
   ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
     '{command: 100001, param1: ${MODE_ID}.0, target_system: 1, target_component: 1, source_system: 255, source_component: 0, from_external: true}'"

sleep 0.5

# 2. Arm via VehicleCommand (command 400 = MAV_CMD_COMPONENT_ARM_DISARM, param1=1 = arm)
docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -lc \
  "${SOURCE_CMD} && \
   ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
     '{command: 400, param1: 1.0, target_system: 1, target_component: 1, source_system: 255, source_component: 0, from_external: true}'"

printf 'Mode %s activated and arm request sent. Check vehicle status with:\n  ros2 topic echo /fmu/out/vehicle_status --once\n' "${MODE_ID}"
