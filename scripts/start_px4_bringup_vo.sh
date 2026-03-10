#!/usr/bin/env bash
# Launch px4_bringup inside the Docker container.
#
# Usage (from host):
#   ./scripts/start_px4_bringup_vo.sh [OPTIONS]
#
# Options:
#   --mode-type TYPE        Mode type (default: speed_steering)
#   --vo-bridge             Enable VO bridge (default: disabled)
#   --odom-topic TOPIC      Odometry topic (default: /odom)
#
# Examples:
#   ./scripts/start_px4_bringup_vo.sh
#   ./scripts/start_px4_bringup_vo.sh --vo-bridge
#   ./scripts/start_px4_bringup_vo.sh --mode-type trajectory --vo-bridge --odom-topic /rtabmap/odom
set -euo pipefail

MODE_TYPE="speed_steering"
VO_BRIDGE="false"
ODOM_TOPIC="/odom"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode-type)   MODE_TYPE="$2"; shift 2 ;;
    --vo-bridge)   VO_BRIDGE="true"; shift ;;
    --odom-topic)  ODOM_TOPIC="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
      exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${PROJECT_DIR}/docker/docker-compose.yml"

echo "Launching px4_bringup (mode=${MODE_TYPE}, vo_bridge=${VO_BRIDGE}, odom=${ODOM_TOPIC})..."

exec docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -c "
  source /opt/ros/jazzy/setup.bash
  source /workspace/install/setup.bash
  ros2 launch px4_bringup px4_bringup.launch.py \
    mode_type:=${MODE_TYPE} enable_vo_bridge:=${VO_BRIDGE} odom_topic:=${ODOM_TOPIC}
"
