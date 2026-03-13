#!/usr/bin/env bash
# Launch ROS 2 nodes inside the Docker container.
# Wraps robot_bringup and optionally px4_bridge so you don't need to
# source workspaces or type long ros2 launch commands manually.
#
# Flags:
#   --px4              Enable PX4 SITL (disables ros2_control, implies --bridge --vo-bridge)
#   --rtabmap          Launch RTAB-Map SLAM
#   --nav2             Launch Nav2 navigation stack
#   --bridge[=MODE]    Launch PX4 mode node (default: speed_steering; options: trajectory, speed_attitude, manual)
#   --vo-bridge        Launch VO bridge: px4_vision_odom + px4_vehicle_odometry
#   --odom-topic=TOPIC Odometry source for vision odom node (default: /odometry/filtered)
#   --no-rviz          Disable RViz2
#   --build[=PKG]      Build workspace (or specific pkg) before launching
#   --build-only[=PKG] Build only, do not launch
#
# Flag implications:
#   --px4        → auto-enables --bridge + --vo-bridge (PX4 SITL needs both)
#   --vo-bridge  alone launches only px4_vision_odom + px4_vehicle_odometry (no mode node)
#   --bridge     alone launches only the mode node (no VO bridge)
#   --bridge     without --px4 keeps enable_px4_sitl=false (ros2_control stays active)
#
# Usage (from host):
#
#   ── Gazebo only (ros2_control) ──
#   ./scripts/start_ros2_nodes.sh
#
#   ── Gazebo + RTAB-Map ──
#   ./scripts/start_ros2_nodes.sh --rtabmap
#
#   ── Gazebo + RTAB-Map + Nav2 ──
#   ./scripts/start_ros2_nodes.sh --rtabmap --nav2
#
#   ── Gazebo + RTAB-Map + VO bridge (ros2_control active, no PX4 mode node) ──
#   ./scripts/start_ros2_nodes.sh --rtabmap --vo-bridge
#
#   ── Gazebo + RTAB-Map + VO bridge using rtabmap/odom directly (bypass EKF) ──
#   ./scripts/start_ros2_nodes.sh --rtabmap --vo-bridge --odom-topic=/rtabmap/odom
#
#   ── Gazebo + RTAB-Map + Nav2 + VO bridge (ros2_control active) ──
#   ./scripts/start_ros2_nodes.sh --rtabmap --nav2 --vo-bridge
#
#   ── Gazebo + RTAB-Map + Nav2 + PX4 mode + VO bridge (ros2_control active) ──
#   ./scripts/start_ros2_nodes.sh --rtabmap --nav2 --bridge=manual --vo-bridge
#
#   ── PX4 SITL (no ros2_control, auto mode + VO) ──
#   ./scripts/start_ros2_nodes.sh --px4
#
#   ── PX4 SITL + RTAB-Map + Nav2 ──
#   ./scripts/start_ros2_nodes.sh --px4 --rtabmap --nav2
#
#   ── PX4 SITL + trajectory mode ──
#   ./scripts/start_ros2_nodes.sh --px4 --bridge=trajectory
#
#   ── Build all, then launch Gazebo ──
#   ./scripts/start_ros2_nodes.sh --build
#
#   ── Build one package, then launch ──
#   ./scripts/start_ros2_nodes.sh --build=description_robot
#
#   ── Build only (no launch) ──
#   ./scripts/start_ros2_nodes.sh --build-only
#   ./scripts/start_ros2_nodes.sh --build-only=pkg1,pkg2
#
#   ── Any combination + disable RViz ──
#   ./scripts/start_ros2_nodes.sh --rtabmap --nav2 --vo-bridge --no-rviz
#
# Prerequisites:
#   1. Docker container must be running (docker-compose up -d)
#   2. For --bridge / --vo-bridge / --px4: MicroXRCEAgent and PX4 SITL must
#      already be running (./scripts/start_microxrce_agent.sh + PX4 SITL)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${PROJECT_DIR}/docker/docker-compose.yml"

# Defaults
PX4="false"
RTABMAP="false"
NAV2="false"
RVIZ="true"
BRIDGE="false"
BRIDGE_MODE="speed_steering"
VO_BRIDGE="false"
ODOM_TOPIC="/odometry/filtered"
BUILD="false"
BUILD_ONLY="false"
BUILD_PKGS=""

# Parse arguments
for arg in "$@"; do
    case "$arg" in
        --px4)          PX4="true" ;;
        --rtabmap)      RTABMAP="true" ;;
        --nav2)         NAV2="true" ;;
        --no-rviz)      RVIZ="false" ;;
        --bridge)       BRIDGE="true" ;;
        --bridge=*)     BRIDGE="true"; BRIDGE_MODE="${arg#--bridge=}" ;;
        --vo-bridge)    VO_BRIDGE="true" ;;
        --odom-topic=*) ODOM_TOPIC="${arg#--odom-topic=}" ;;
        --build)        BUILD="true" ;;
        --build=*)      BUILD="true"; BUILD_PKGS="${arg#--build=}" ;;
        --build-only)   BUILD="true"; BUILD_ONLY="true" ;;
        --build-only=*) BUILD="true"; BUILD_ONLY="true"; BUILD_PKGS="${arg#--build-only=}" ;;
        -h|--help)
            sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
            exit 0 ;;
        *)
            echo "Unknown argument: $arg"
            echo "Usage: $0 [--px4] [--rtabmap] [--nav2] [--no-rviz] [--bridge[=mode]] [--vo-bridge] [--odom-topic=TOPIC] [--build[=pkg]] [--build-only[=pkg,pkg]]"
            exit 1
            ;;
    esac
done

# ── Flag implications ──
# --px4 (PX4 SITL) always needs both mode node and VO bridge
if [[ "${PX4}" == "true" ]]; then
    BRIDGE="true"
    VO_BRIDGE="true"
fi

# ── Build step (if requested) ──
if [[ "${BUILD}" == "true" ]]; then
    BUILD_CMD="source /opt/ros/\$ROS_DISTRO/setup.bash && cd /workspace && colcon build --symlink-install"
    if [[ -n "${BUILD_PKGS}" ]]; then
        # Replace commas with spaces for --packages-select
        BUILD_CMD+=" --packages-select ${BUILD_PKGS//,/ }"
        echo "Building packages: ${BUILD_PKGS}..."
    else
        echo "Building all packages..."
    fi
    docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -c "${BUILD_CMD}"
    echo ""
    if [[ "${BUILD_ONLY}" == "true" ]]; then
        echo "Build complete. Skipping launch (--build-only)."
        exit 0
    fi
fi

echo "Launching ROS 2 nodes inside Docker container..."
echo "  PX4 SITL:     ${PX4}"
echo "  RTAB-Map:     ${RTABMAP}"
echo "  Nav2:         ${NAV2}"
echo "  RViz:         ${RVIZ}"
if [[ "${BRIDGE}" == "true" ]]; then
    echo "  px4_bridge:   ${BRIDGE_MODE}"
fi
if [[ "${VO_BRIDGE}" == "true" ]]; then
    echo "  VO bridge:    true"
    echo "  Odom topic:   ${ODOM_TOPIC}"
fi
echo ""

# Build the launch command
LAUNCH_CMD="source /opt/ros/\$ROS_DISTRO/setup.bash && source /workspace/install/setup.bash && "
LAUNCH_CMD+="ros2 launch robot_bringup robot_bringup.launch.py"
LAUNCH_CMD+=" enable_px4_sitl:=${PX4}"
LAUNCH_CMD+=" rtabmap:=${RTABMAP}"
LAUNCH_CMD+=" nav2:=${NAV2}"
LAUNCH_CMD+=" rviz:=${RVIZ}"

# Chain px4_bringup after robot_bringup when --bridge and/or --vo-bridge
if [[ "${BRIDGE}" == "true" || "${VO_BRIDGE}" == "true" ]]; then
    LAUNCH_CMD+=" &"
    LAUNCH_CMD+=" BRINGUP_PID=\$!;"
    LAUNCH_CMD+=" PIDS=\$BRINGUP_PID;"
    LAUNCH_CMD+=" echo 'Waiting 5s for Gazebo to start before launching px4 nodes...';"
    LAUNCH_CMD+=" sleep 5;"
    LAUNCH_CMD+=" ros2 launch px4_bringup px4_bringup.launch.py"
    LAUNCH_CMD+=" enable_mode_node:=${BRIDGE}"
    LAUNCH_CMD+=" mode_type:=${BRIDGE_MODE}"
    LAUNCH_CMD+=" enable_vo_bridge:=${VO_BRIDGE}"
    LAUNCH_CMD+=" odom_topic:=${ODOM_TOPIC}"
    LAUNCH_CMD+=" enable_vehicle_odometry:=${VO_BRIDGE} &"
    LAUNCH_CMD+=" PIDS=\"\$PIDS \$!\";"
    LAUNCH_CMD+=" trap 'kill \$PIDS 2>/dev/null; wait' EXIT INT TERM;"
    LAUNCH_CMD+=" wait"
fi

exec docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -c "${LAUNCH_CMD}"
