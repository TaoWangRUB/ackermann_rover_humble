#!/usr/bin/env bash
# Activate a PX4 registered external mode and/or arm/disarm the rover.
#
# Usage:
#   ./scripts/activate_rover_manual.sh              # activate mode 23 + arm (default)
#   ./scripts/activate_rover_manual.sh 24           # activate mode 24 + arm
#   ./scripts/activate_rover_manual.sh --arm        # arm only (skip mode activation)
#   ./scripts/activate_rover_manual.sh --disarm     # disarm only
#   ./scripts/activate_rover_manual.sh --activate   # activate mode 23 + arm (explicit)
#   ./scripts/activate_rover_manual.sh 24 --activate # activate mode 24 + arm
#
# Publishes VehicleCommand to switch PX4 into the specified external mode,
# then sends an arm/disarm request via ROS 2.
# Works with both PX4 SITL (Gazebo) and real hardware (Cube Black).
set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/../docker/docker-compose.yml"
MODE_ID="23"
ACTION="activate"  # default: activate mode + arm

for arg in "$@"; do
    case "${arg}" in
        --arm)       ACTION="arm" ;;
        --disarm)    ACTION="disarm" ;;
        --activate)  ACTION="activate" ;;
        [0-9]*)      MODE_ID="${arg}" ;;
    esac
done

SOURCE_CMD="source /opt/ros/\$ROS_DISTRO/setup.bash && source /workspace/install/setup.bash"

PUB_CMD="ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand"

if [[ "${ACTION}" == "disarm" ]]; then
    echo "Disarming PX4..."
    docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -lc \
      "${SOURCE_CMD} && ${PUB_CMD} \
         '{command: 400, param1: 0.0, target_system: 1, target_component: 1, source_system: 255, source_component: 0, from_external: true}'"
    echo "Disarm request sent."

elif [[ "${ACTION}" == "arm" ]]; then
    echo "Arming PX4 (no mode switch)..."
    docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -lc \
      "${SOURCE_CMD} && ${PUB_CMD} \
         '{command: 400, param1: 1.0, target_system: 1, target_component: 1, source_system: 255, source_component: 0, from_external: true}'"
    echo "Arm request sent."

else
    # activate: switch mode + arm
    echo "Activating external mode ${MODE_ID} + arming PX4..."

    # 1. Switch to the registered external mode
    docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -lc \
      "${SOURCE_CMD} && ${PUB_CMD} \
         '{command: 100001, param1: ${MODE_ID}.0, target_system: 1, target_component: 1, source_system: 255, source_component: 0, from_external: true}'"

    sleep 0.5

    # 2. Arm
    docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -lc \
      "${SOURCE_CMD} && ${PUB_CMD} \
         '{command: 400, param1: 1.0, target_system: 1, target_component: 1, source_system: 255, source_component: 0, from_external: true}'"

    printf 'Mode %s activated and arm request sent.\n' "${MODE_ID}"
fi
