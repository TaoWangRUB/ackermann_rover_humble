#!/usr/bin/env bash
# Launch px4_bringup inside the Docker container.
#
# Usage (from host):
#   ./scripts/start_px4_bringup_vo.sh [OPTIONS]
#
# Options:
#   --bridge                Enable PX4 mode bridge (default: disabled)
#   --mode-type TYPE        Mode type (default: manual)
#   --vo-bridge             Enable VO bridge (default: disabled)
#   --odom-topic TOPIC      Odometry topic (default: /odometry/filtered)
#   --odom-transport TYPE   Odometry transport: 'xrce' (default) or 'mavlink'
#   --mav-device DEVICE     MAVLink device (default: /dev/ttyACM0)
#   --mav-baud BAUD         MAVLink baud (default: 57600)
#   --mav-rate RATE         MAVLink publish rate (default: 20)
#
# Examples:
#   ./scripts/start_px4_bringup_vo.sh --bridge
#   ./scripts/start_px4_bringup_vo.sh --vo-bridge
#   ./scripts/start_px4_bringup_vo.sh --bridge --mode-type manual --vo-bridge --odom-topic /odometry/filtered
#   # Use MAVLink transport for VO (uses default device/baud/rate in the bridge):
#   ./scripts/start_px4_bringup_vo.sh --bridge --vo-bridge --odom-transport mavlink
set -euo pipefail

MODE_TYPE="manual"
BRIDGE="false"
VO_BRIDGE="false"
ODOM_TOPIC="/odometry/filtered"
ODOM_TRANSPORT="xrce"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode-type)   MODE_TYPE="$2"; shift 2 ;;
    --bridge)      BRIDGE="true"; shift ;;
    --vo-bridge)   VO_BRIDGE="true"; shift ;;
    --odom-topic)  ODOM_TOPIC="$2"; shift 2 ;;
    --odom-transport) ODOM_TRANSPORT="$2"; shift 2 ;;
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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${PROJECT_DIR}/docker/docker-compose.yml"

echo "Launching px4_bringup (mode=${MODE_TYPE}, bridge=${BRIDGE}, vo_bridge=${VO_BRIDGE}, odom=${ODOM_TOPIC}, odom_transport=${ODOM_TRANSPORT})..."

# Build the launch command
LAUNCH_CMD="source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && "

if [[ "${BRIDGE}" == "true" ]]; then
  # Launch mode node + optionally VO bridge via px4_bringup.launch.py
  LAUNCH_CMD+=" ros2 launch px4_bringup px4_bringup.launch.py mode_type:=${MODE_TYPE} enable_vo_bridge:=${VO_BRIDGE} odom_topic:=${ODOM_TOPIC} odometry_transport:=${ODOM_TRANSPORT} &"
  LAUNCH_CMD+=" PIDS=\"\$!\";"
elif [[ "${VO_BRIDGE}" == "true" ]]; then
  # VO bridge only — launch either XRCE node or MAVLink bridge directly
  if [[ "${ODOM_TRANSPORT}" == "xrce" ]]; then
    LAUNCH_CMD+=" ros2 run px4_bringup px4_vision_odom.py --ros-args -p odom_topic:=${ODOM_TOPIC} -p odom_frame:=odom -p base_frame:=ackermann/base_link &"
    LAUNCH_CMD+=" PIDS=\"\$!\";"
  else
    # Launch the MAVLink bridge script (uses /workspace/install setup inside container)
    LAUNCH_CMD+=" /workspace/scripts/px4_mavlink_vpe.sh --topic ${ODOM_TOPIC} &"
    LAUNCH_CMD+=" PIDS=\"\$!\";"
  fi
else
    echo "Nothing to launch! Both bridge and vo_bridge are disabled."
    exit 0
fi

LAUNCH_CMD+=" trap 'kill \$PIDS 2>/dev/null; wait' EXIT INT TERM;"
LAUNCH_CMD+=" wait"

exec docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -c "${LAUNCH_CMD}"
