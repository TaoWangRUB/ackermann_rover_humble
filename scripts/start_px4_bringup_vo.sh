#!/usr/bin/env bash
# Launch px4_bringup inside the Docker container.
#
# Usage (from host):
#   ./scripts/start_px4_bringup_vo.sh [OPTIONS]
#
# Options:
#   --bridge                Enable PX4 mode bridge (default: disabled)
#   --mode-type TYPE        Mode type (default: manual)
#   --vo-bridge             Enable VO bridge: launches px4_vehicle_odometry + px4_vision_odom (default: disabled)
#   --odom-topic TOPIC      Odometry topic for vision odom node (default: /odometry/filtered)
#   --reversible-drive      Bidirectional ESC: throttle [-1,1] and allow reverse in Nav2 (default: false)
#
# Examples:
#   ./scripts/start_px4_bringup_vo.sh --bridge
#   ./scripts/start_px4_bringup_vo.sh --bridge --mode-type speed_steering
#   ./scripts/start_px4_bringup_vo.sh --vo-bridge
#   ./scripts/start_px4_bringup_vo.sh --bridge --vo-bridge
#   ./scripts/start_px4_bringup_vo.sh --bridge --mode-type manual --vo-bridge --odom-topic /rtabmap/odom
#   ./scripts/start_px4_bringup_vo.sh --bridge --reversible-drive
set -euo pipefail

MODE_TYPE="manual"
BRIDGE="false"
VO_BRIDGE="false"
ODOM_TOPIC="/odometry/filtered"
REVERSIBLE_DRIVE="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode-type)        MODE_TYPE="$2"; shift 2 ;;
    --bridge)           BRIDGE="true"; shift ;;
    --vo-bridge)        VO_BRIDGE="true"; shift ;;
    --odom-topic)       ODOM_TOPIC="$2"; shift 2 ;;
    --reversible-drive) REVERSIBLE_DRIVE="true"; shift ;;
    -h|--help)
      sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
      exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
done

case "${MODE_TYPE}" in
  manual|trajectory|speed_steering|speed_attitude)
    # Valid mode
    ;;
  *)
    echo "ERROR: Invalid mode-type '${MODE_TYPE}'" >&2
    echo "Valid options: manual, trajectory, speed_steering, speed_attitude" >&2
    exit 1
    ;;
esac

if [[ "${BRIDGE}" == "false" && "${VO_BRIDGE}" == "false" ]]; then
    echo "Nothing to launch! Both --bridge and --vo-bridge are disabled."
    exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${PROJECT_DIR}/docker/docker-compose.yml"
ENV_FILE="${PROJECT_DIR}/.env"

echo "Launching px4_bringup (mode=${MODE_TYPE}, bridge=${BRIDGE}, vo_bridge=${VO_BRIDGE}, odom_topic=${ODOM_TOPIC}, reversible_drive=${REVERSIBLE_DRIVE})..."

# Build the launch command
LAUNCH_CMD="source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash &&"
LAUNCH_CMD+=" ros2 launch px4_bringup px4_bringup.launch.py"
LAUNCH_CMD+=" enable_mode_node:=${BRIDGE}"
LAUNCH_CMD+=" mode_type:=${MODE_TYPE}"
LAUNCH_CMD+=" enable_vo_bridge:=${VO_BRIDGE}"
LAUNCH_CMD+=" odom_topic:=${ODOM_TOPIC}"
LAUNCH_CMD+=" enable_vehicle_odometry:=${VO_BRIDGE}"
LAUNCH_CMD+=" reversible_drive:=${REVERSIBLE_DRIVE}"

exec docker-compose --env-file "${ENV_FILE}" -f "${COMPOSE_FILE}" exec ackermann_slam bash -c "${LAUNCH_CMD}"
