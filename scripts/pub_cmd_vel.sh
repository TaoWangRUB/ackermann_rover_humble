#!/usr/bin/env bash
# Publish TwistStamped on /cmd_vel inside Docker.
#
# Usage:
#   ./scripts/pub_cmd_vel.sh                  # forward 1.0 m/s, no turn
#   ./scripts/pub_cmd_vel.sh 0.5 0.3          # forward 0.5 m/s, left turn 0.3 rad/s
#   ./scripts/pub_cmd_vel.sh 0 0              # stop
set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

LINEAR_X="${1:-1.0}"
ANGULAR_Z="${2:-0.0}"
RATE="${3:-10}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/../docker/docker-compose.yml"

echo "Publishing /cmd_vel: linear.x=${LINEAR_X}, angular.z=${ANGULAR_Z} @ ${RATE} Hz"
exec docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -c "
  source /opt/ros/\$ROS_DISTRO/setup.bash && source /workspace/install/setup.bash &&
  ros2 topic pub -r ${RATE} /cmd_vel geometry_msgs/msg/TwistStamped \
    \"{header: {frame_id: 'ackermann/base_link'}, twist: {linear: {x: ${LINEAR_X}}, angular: {z: ${ANGULAR_Z}}}}\"
"
