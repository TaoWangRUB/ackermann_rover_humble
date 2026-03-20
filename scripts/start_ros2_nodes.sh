#!/usr/bin/env bash
# Launch ROS 2 nodes inside the Docker container.
# Wraps robot_bringup and optionally px4_bridge so you don't need to
# source workspaces or type long ros2 launch commands manually.
#
# Flags:
#   --hw               Hardware mode: real cameras instead of Gazebo (use_gazebo:=false).
#                      Starts robot_state_publisher + realsense_camera_bringup.
#                      Default cameras in HW mode: L515 only.
#   --d435i            [HW mode] Enable D435i depth camera
#   --l515             [HW mode] Enable L515 depth camera (on by default in HW mode)
#   --t265             [HW mode] Enable T265 tracking camera + odom_tf_relay
#   --px4              Enable PX4 SITL (disables ros2_control, implies --bridge --vo-bridge)
#   --rtabmap          Launch RTAB-Map SLAM
#   --nav2             Launch Nav2 navigation stack
#   --bridge[=MODE]    Launch PX4 mode node (default: manual; options: speed_steering, trajectory, speed_attitude)
#   --vo-bridge        Launch VO bridge: px4_vision_odom + px4_vehicle_odometry
#   --odom-topic=TOPIC Odometry source for vision odom node (default: /odometry/filtered)
#   --reversible-drive Bidirectional ESC: throttle [-1,1] and allow reverse in Nav2 (default: false)
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
#   ── Hardware mode (real cameras, L515 default) ──
#   ./scripts/start_ros2_nodes.sh --hw
#
#   ── Hardware mode + RTAB-Map ──
#   ./scripts/start_ros2_nodes.sh --hw --rtabmap
#
#   ── Hardware mode + RTAB-Map + Nav2 ──
#   ./scripts/start_ros2_nodes.sh --hw --rtabmap --nav2
#
#   ── Hardware mode + L515 + T265 + RTAB-Map ──
#   ./scripts/start_ros2_nodes.sh --hw --l515 --t265 --rtabmap
#
#   ── Hardware mode + VO bridge to real PX4 ──
#   ./scripts/start_ros2_nodes.sh --hw --rtabmap --vo-bridge --bridge=speed_steering
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
#   ── Unidirectional ESC (default) ──
#   ./scripts/start_ros2_nodes.sh --rtabmap --nav2 --px4
#
#   ── Bidirectional ESC ──
#   ./scripts/start_ros2_nodes.sh --rtabmap --nav2 --px4 --reversible-drive
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
#   2. For --bridge / --vo-bridge / --px4: MicroXRCEAgent must already be running
#      (./scripts/start_microxrce_agent.sh); for --px4 also PX4 SITL
#   3. For --hw: RealSense cameras must be connected and accessible via USB
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${PROJECT_DIR}/docker/docker-compose.yml"

# Defaults
HW="false"
HW_D435I="false"
HW_L515="false"   # set to true below if --hw without explicit camera flags
HW_T265="false"
HW_CAMERA_EXPLICIT="false"  # tracks whether user passed any camera flag
PX4="false"
RTABMAP="false"
NAV2="false"
RVIZ="true"
BRIDGE="false"
BRIDGE_MODE="manual"
VO_BRIDGE="false"
ODOM_TOPIC="/odometry/filtered"
REVERSIBLE_DRIVE="false"
BUILD="false"
BUILD_ONLY="false"
BUILD_PKGS=""

# Parse arguments
for arg in "$@"; do
    case "$arg" in
        --hw)           HW="true" ;;
        --d435i)        HW_D435I="true"; HW_CAMERA_EXPLICIT="true" ;;
        --l515)         HW_L515="true";  HW_CAMERA_EXPLICIT="true" ;;
        --t265)         HW_T265="true";  HW_CAMERA_EXPLICIT="true" ;;
        --px4)          PX4="true" ;;
        --rtabmap)      RTABMAP="true" ;;
        --nav2)         NAV2="true" ;;
        --no-rviz)      RVIZ="false" ;;
        --bridge)       BRIDGE="true" ;;
        --bridge=*)     BRIDGE="true"; BRIDGE_MODE="${arg#--bridge=}" ;;
        --vo-bridge)         VO_BRIDGE="true" ;;
        --odom-topic=*)      ODOM_TOPIC="${arg#--odom-topic=}" ;;
        --reversible-drive)  REVERSIBLE_DRIVE="true" ;;
        --build)             BUILD="true" ;;
        --build=*)      BUILD="true"; BUILD_PKGS="${arg#--build=}" ;;
        --build-only)   BUILD="true"; BUILD_ONLY="true" ;;
        --build-only=*) BUILD="true"; BUILD_ONLY="true"; BUILD_PKGS="${arg#--build-only=}" ;;
        -h|--help)
            sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
            exit 0 ;;
        *)
            echo "Unknown argument: $arg"
            echo "Usage: $0 [--hw] [--d435i] [--l515] [--t265] [--px4] [--rtabmap] [--nav2] [--no-rviz] [--bridge[=mode]] [--vo-bridge] [--odom-topic=TOPIC] [--build[=pkg]] [--build-only[=pkg,pkg]]"
            exit 1
            ;;
    esac
done

# ── Flag implications ──
# --hw without explicit camera flags → default to L515
if [[ "${HW}" == "true" && "${HW_CAMERA_EXPLICIT}" == "false" ]]; then
    HW_L515="true"
fi

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
if [[ "${HW}" == "true" ]]; then
    echo "  Mode:         hardware (real cameras)"
    echo "  D435i:        ${HW_D435I}"
    echo "  L515:         ${HW_L515}"
    echo "  T265:         ${HW_T265}"
else
    echo "  Mode:         simulation (Gazebo)"
    echo "  PX4 SITL:     ${PX4}"
fi
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
if [[ "${HW}" == "true" ]]; then
    LAUNCH_CMD+=" use_gazebo:=false"
    LAUNCH_CMD+=" use_sim_time:=false"
    LAUNCH_CMD+=" hw_enable_d435i:=${HW_D435I}"
    LAUNCH_CMD+=" hw_enable_l515:=${HW_L515}"
    LAUNCH_CMD+=" hw_enable_t265:=${HW_T265}"
else
    LAUNCH_CMD+=" enable_px4_sitl:=${PX4}"
fi
LAUNCH_CMD+=" rtabmap:=${RTABMAP}"
LAUNCH_CMD+=" nav2:=${NAV2}"
LAUNCH_CMD+=" rviz:=${RVIZ}"
LAUNCH_CMD+=" reversible_drive:=${REVERSIBLE_DRIVE}"

# Chain px4_bringup after robot_bringup when --bridge and/or --vo-bridge
if [[ "${BRIDGE}" == "true" || "${VO_BRIDGE}" == "true" ]]; then
    LAUNCH_CMD+=" &"
    LAUNCH_CMD+=" BRINGUP_PID=\$!;"
    LAUNCH_CMD+=" PIDS=\$BRINGUP_PID;"
    LAUNCH_CMD+=" echo 'Waiting 5s for Gazebo to start before launching px4 nodes...';"
    LAUNCH_CMD+=" sleep 5;"
    LAUNCH_CMD+=" source /opt/ros/\$ROS_DISTRO/setup.bash;"
    LAUNCH_CMD+=" source /workspace/install/setup.bash;"
    LAUNCH_CMD+=" ros2 launch px4_bringup px4_bringup.launch.py"
    LAUNCH_CMD+=" enable_mode_node:=${BRIDGE}"
    LAUNCH_CMD+=" mode_type:=${BRIDGE_MODE}"
    LAUNCH_CMD+=" enable_vo_bridge:=${VO_BRIDGE}"
    LAUNCH_CMD+=" odom_topic:=${ODOM_TOPIC}"
    LAUNCH_CMD+=" enable_vehicle_odometry:=${VO_BRIDGE}"
    LAUNCH_CMD+=" reversible_drive:=${REVERSIBLE_DRIVE} &"
    LAUNCH_CMD+=" PIDS=\"\$PIDS \$!\";"
    LAUNCH_CMD+=" trap 'kill \$PIDS 2>/dev/null; wait' EXIT INT TERM;"
    LAUNCH_CMD+=" wait"
fi

exec docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -c "${LAUNCH_CMD}"
