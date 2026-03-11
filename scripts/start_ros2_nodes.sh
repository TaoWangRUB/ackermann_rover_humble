#!/usr/bin/env bash
# Launch ROS 2 nodes inside the Docker container.
# Wraps robot_bringup and optionally px4_bridge so you don't need to
# source workspaces or type long ros2 launch commands manually.
#
# Usage (from host):
#   ./scripts/start_ros2_nodes.sh                          # Gazebo only (no PX4, no SLAM, no Nav2)
#   ./scripts/start_ros2_nodes.sh --px4                    # Gazebo + PX4 sensors (no bridge yet)
#   ./scripts/start_ros2_nodes.sh --px4 --bridge            # Gazebo + px4_bridge (speed_steering)
#   ./scripts/start_ros2_nodes.sh --px4 --bridge=trajectory # Gazebo + px4_bridge (trajectory mode)
#   ./scripts/start_ros2_nodes.sh --rtabmap                # Gazebo + RTAB-Map
#   ./scripts/start_ros2_nodes.sh --rtabmap --nav2          # Gazebo + RTAB-Map + Nav2
#   ./scripts/start_ros2_nodes.sh --px4 --rtabmap --nav2   # Full stack (PX4 + SLAM + Nav2)
#   ./scripts/start_ros2_nodes.sh --no-rviz                # Disable RViz
#   ./scripts/start_ros2_nodes.sh --build                   # Build all pkgs, then launch
#   ./scripts/start_ros2_nodes.sh --build=description_robot # Build one pkg, then launch
#   ./scripts/start_ros2_nodes.sh --build-only              # Build all pkgs, no launch
#   ./scripts/start_ros2_nodes.sh --build-only=pkg1,pkg2    # Build selected pkgs, no launch
#
# Prerequisites:
#   1. Docker container must be running (docker-compose up -d)
#   2. For --bridge: MicroXRCEAgent and PX4 SITL must already be running
#      (./scripts/start_microxrce_agent.sh + ./scripts/start_px4_sitl.sh)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${PROJECT_DIR}/docker/docker-compose.yml"

# Defaults
PX4="false"
RTABMAP="false"
NAV2="false"
RVIZ="true"
BRIDGE=""
BRIDGE_MODE="speed_steering"
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
        --build)        BUILD="true" ;;
        --build=*)      BUILD="true"; BUILD_PKGS="${arg#--build=}" ;;
        --build-only)   BUILD="true"; BUILD_ONLY="true" ;;
        --build-only=*) BUILD="true"; BUILD_ONLY="true"; BUILD_PKGS="${arg#--build-only=}" ;;
        *)
            echo "Unknown argument: $arg"
            echo "Usage: $0 [--px4] [--rtabmap] [--nav2] [--no-rviz] [--bridge[=mode]] [--build[=pkg]] [--build-only[=pkg,pkg]]"
            exit 1
            ;;
    esac
done

# If --bridge is set without --px4, enable PX4 implicitly
if [[ "${BRIDGE}" == "true" && "${PX4}" == "false" ]]; then
    PX4="true"
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
echo ""

# Build the launch command
LAUNCH_CMD="source /opt/ros/\$ROS_DISTRO/setup.bash && source /workspace/install/setup.bash && "
LAUNCH_CMD+="ros2 launch robot_bringup robot_bringup.launch.py"
LAUNCH_CMD+=" enable_px4_sitl:=${PX4}"
LAUNCH_CMD+=" rtabmap:=${RTABMAP}"
LAUNCH_CMD+=" nav2:=${NAV2}"
LAUNCH_CMD+=" rviz:=${RVIZ}"

# If --bridge is requested, chain px4_bridge after bringup in the same shell
# using a background process + wait pattern
if [[ "${BRIDGE}" == "true" ]]; then
    LAUNCH_CMD+=" &"
    LAUNCH_CMD+=" BRINGUP_PID=\$!;"
    LAUNCH_CMD+=" echo 'Waiting 5s for Gazebo to start before launching px4_bridge...';"
    LAUNCH_CMD+=" sleep 5;"
    LAUNCH_CMD+=" ros2 launch px4_bringup px4_bringup.launch.py mode_type:=${BRIDGE_MODE} &"
    LAUNCH_CMD+=" BRIDGE_PID=\$!;"
    LAUNCH_CMD+=" trap 'kill \$BRINGUP_PID \$BRIDGE_PID 2>/dev/null; wait' EXIT INT TERM;"
    LAUNCH_CMD+=" wait"
fi

exec docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -c "${LAUNCH_CMD}"
