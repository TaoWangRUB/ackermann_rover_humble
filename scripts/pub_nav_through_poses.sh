#!/usr/bin/env bash
# Send a fixed Nav2 NavigateThroughPoses goal through two map-frame waypoints:
#   (3.0, -2.0) -> (4.0, 3.0)
#
# The waypoint orientations are intentionally fixed to a constrained
# drive-through-heading test:
#   pose 1: northeast
#   pose 2: south
#
# Usage (from host):
#   ./scripts/pub_nav_through_poses.sh
#   ./scripts/pub_nav_through_poses.sh --frame=map
#   ./scripts/pub_nav_through_poses.sh --action=/navigate_through_poses
#   ./scripts/pub_nav_through_poses.sh --wait-timeout=90
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"

ACTION_NAME="/navigate_through_poses"
FRAME_ID="map"
WAIT_TIMEOUT_SEC=180

for arg in "$@"; do
    case "${arg}" in
        --action=*) ACTION_NAME="${arg#--action=}" ;;
        --frame=*)  FRAME_ID="${arg#--frame=}" ;;
        --wait-timeout=*) WAIT_TIMEOUT_SEC="${arg#--wait-timeout=}" ;;
        -h|--help)
            sed -n '2,/^set /{ /^# \{0,1\}/s/^# \{0,1\}//p }' "$0"
            exit 0
            ;;
        *)
            echo "Unknown argument: ${arg}" >&2
            exit 1
            ;;
    esac
done

GOAL_PAYLOAD=$(cat <<EOF
{
  poses: [
    {
      header: {frame_id: "${FRAME_ID}"},
      pose: {
        position: {x: 3.0, y: -2.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.3429, w: 0.9394}
      }
    },
    {
      header: {frame_id: "${FRAME_ID}"},
      pose: {
        position: {x: 4.0, y: 3.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: -0.7071, w: 0.7071}
      }
    }
  ]
}
EOF
)

if ! dcomp ps --services --filter status=running 2>/dev/null | grep -q '^ackermann_slam$'; then
    echo "Container not running — starting ackermann_slam..."
    dcomp up -d ackermann_slam
fi

ROS_CMD="source /opt/ros/\$ROS_DISTRO/setup.bash && "
ROS_CMD+="source /workspace/install/setup.bash && "
ROS_CMD+="wait_for_active() { "
ROS_CMD+="  local node=\"\$1\"; local deadline=\$((SECONDS + ${WAIT_TIMEOUT_SEC})); "
ROS_CMD+="  while true; do "
ROS_CMD+="    if ros2 lifecycle get \"\${node}\" 2>/dev/null | grep -q '^active \\[3\\]'; then "
ROS_CMD+="      break; "
ROS_CMD+="    fi; "
ROS_CMD+="    if (( SECONDS >= deadline )); then "
ROS_CMD+="      echo \"Timed out waiting for \${node} to become active\" >&2; "
ROS_CMD+="      return 1; "
ROS_CMD+="    fi; "
ROS_CMD+="    sleep 1; "
ROS_CMD+="  done; "
ROS_CMD+="}; "
ROS_CMD+="wait_for_active /bt_navigator && "
ROS_CMD+="wait_for_active /controller_server && "
ROS_CMD+="wait_for_active /planner_server && "
ROS_CMD+="ros2 action send_goal --feedback "
ROS_CMD+="'${ACTION_NAME}' nav2_msgs/action/NavigateThroughPoses "
ROS_CMD+="'${GOAL_PAYLOAD}'"

if [ -t 0 ]; then
    xdcomp exec ackermann_slam bash -lc "${ROS_CMD}"
else
    xdcomp exec -T ackermann_slam bash -lc "${ROS_CMD}"
fi
