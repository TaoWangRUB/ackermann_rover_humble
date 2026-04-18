#!/usr/bin/env bash
# Launch the Nav2 stack inside the ackermann_slam Docker container on the host
# laptop (offloaded from the compute-starved Jetson).
#
# Mirrors the Nav2 IncludeLaunchDescription in
# src/robot_bringup/launch/robot_bringup.launch.py, so costmaps/BT/params stay
# identical to the on-rover launch path — only the host changes.
#
# Prerequisites:
#   1. Docker container running:   docker compose -f docker/docker-compose.yml up -d ackermann_slam
#   2. Workspace built in-container (at least ackermann_nav2_bringup):
#        ./scripts/start_nav2.sh --build
#   3. Same ROS_DOMAIN_ID / RMW as the Jetson, and DDS discovery reachable
#      (same LAN, or a discovery server). The container inherits network=host
#      via docker-compose.yml, so this works out of the box on a shared LAN.
#   4. Jetson is already publishing /tf, /tf_static, /map, /odometry/filtered
#      (or the remapped /odom), /scan, and subscribing to /cmd_vel.
#
# Flags:
#   --controller=TYPE       Nav2 path controller: mppi (default) or rpp
#   --params-file=PATH      Override params YAML (container path or host path
#                           under the workspace — it's mounted at /workspace)
#   --bt-xml=PATH           Override NavigateToPose BT XML
#   --through-poses-bt=PATH Override NavigateThroughPoses BT XML
#   --reversible-drive      Bidirectional ESC (matches start_jetson_session.sh default)
#   --use-sim-time          Only when replaying bags on the host
#   --build                 colcon build ackermann_nav2_bringup (up-to) before launch
#   --build-only            Build and exit
#
# Usage:
#   ./scripts/start_nav2.sh
#   ./scripts/start_nav2.sh --reversible-drive
#   ./scripts/start_nav2.sh --controller=rpp --reversible-drive
#   ./scripts/start_nav2.sh --build
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"

# Defaults mirror DeclareLaunchArgument values in robot_bringup.launch.py.
CONTROLLER="mppi"
PARAMS_FILE=""
BT_XML=""
THROUGH_BT=""
REVERSIBLE_DRIVE="false"
USE_SIM_TIME="false"
BUILD="false"
BUILD_ONLY="false"

for arg in "$@"; do
    case "${arg}" in
        --controller=*)        CONTROLLER="${arg#--controller=}" ;;
        --params-file=*)       PARAMS_FILE="${arg#--params-file=}" ;;
        --bt-xml=*)            BT_XML="${arg#--bt-xml=}" ;;
        --through-poses-bt=*)  THROUGH_BT="${arg#--through-poses-bt=}" ;;
        --reversible-drive)    REVERSIBLE_DRIVE="true" ;;
        --use-sim-time)        USE_SIM_TIME="true" ;;
        --build)               BUILD="true" ;;
        --build-only)          BUILD="true"; BUILD_ONLY="true" ;;
        -h|--help)
            sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
            exit 0 ;;
        *)
            echo "Unknown argument: ${arg}" >&2
            exit 1 ;;
    esac
done

# ── Ensure Docker container is running ──────────────────────────────────
if ! dcomp ps --services --filter status=running 2>/dev/null \
        | grep -q '^ackermann_slam$'; then
    echo "Container not running — starting ackermann_slam..."
    dcomp up -d ackermann_slam
fi

# ── Optional build inside the container ────────────────────────────────
if [[ "${BUILD}" == "true" ]]; then
    echo "Building ackermann_nav2_bringup (and deps) inside container..."
    dcomp exec ackermann_slam bash -c \
        "source /opt/ros/\$ROS_DISTRO/setup.bash && cd /workspace && colcon build --symlink-install --packages-up-to ackermann_nav2_bringup"
    if [[ "${BUILD_ONLY}" == "true" ]]; then
        echo "Build complete. Skipping launch (--build-only)."
        exit 0
    fi
fi

echo "Launching Nav2 inside Docker container..."
echo "  controller       = ${CONTROLLER}"
echo "  params_file      = ${PARAMS_FILE:-<package default>}"
echo "  bt_xml           = ${BT_XML:-<nav2_bt_navigator default>}"
echo "  through_poses_bt = ${THROUGH_BT:-<nav2_bt_navigator default>}"
echo "  reversible_drive = ${REVERSIBLE_DRIVE}"
echo "  use_sim_time     = ${USE_SIM_TIME}"
echo ""

LAUNCH_CMD="source /opt/ros/\$ROS_DISTRO/setup.bash && source /workspace/install/setup.bash && "
LAUNCH_CMD+="ros2 launch ackermann_nav2_bringup nav2_bringup.launch.py"
LAUNCH_CMD+=" controller:=${CONTROLLER}"
LAUNCH_CMD+=" reversible_drive:=${REVERSIBLE_DRIVE}"
LAUNCH_CMD+=" use_sim_time:=${USE_SIM_TIME}"
[[ -n "${PARAMS_FILE}" ]] && LAUNCH_CMD+=" params_file:=${PARAMS_FILE}"
[[ -n "${BT_XML}" ]]      && LAUNCH_CMD+=" bt_xml:=${BT_XML}"
[[ -n "${THROUGH_BT}" ]]  && LAUNCH_CMD+=" navigate_through_poses_bt:=${THROUGH_BT}"

if [ -t 0 ]; then
    xdcomp exec ackermann_slam bash -c "${LAUNCH_CMD}"
else
    xdcomp exec -T ackermann_slam bash -c "${LAUNCH_CMD}"
fi
