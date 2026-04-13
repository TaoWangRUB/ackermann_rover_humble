#!/usr/bin/env bash
# Launch ROS 2 nodes inside the Docker container.
# Wraps robot_bringup and optionally px4_bridge so you don't need to
# source workspaces or type long ros2 launch commands manually.
#
# Flags:
#   --hw                   Hardware mode: real cameras instead of Gazebo (use_gazebo:=false).
#                          Starts robot_state_publisher + realsense_camera_bringup.
#   --depth-camera=NAME    Depth camera for RTAB-Map: l515 (default), d435i, or none.
#                          Selects which camera node starts (HW), which Gazebo bridge topics
#                          are exposed (sim), and which RTAB-Map topics are subscribed.
#                          Auto-set to 'none' when --vins-odom/--cuvslam-odom without --rtabmap.
#   --t265                 [HW mode] Enable T265 tracking camera + odom_tf_relay
#   --t265-odom            [HW mode] Use T265 as odometry source for EKF/RTAB-Map
#                          while keeping RTAB-Map VO on /vo_odom for debugging
#   --vins-odom            Use VINS-Fusion as the odometry source. In hardware
#                          mode this automatically relies on the T265 fisheye
#                          streams; in simulation it auto-enables the simulated
#                          T265 path via robot_bringup.
#   --cuvslam-odom         Use cuVSLAM as the odometry source. In hardware
#                          mode this automatically relies on the T265 fisheye
#                          streams; in simulation it auto-enables the simulated
#                          T265 path via robot_bringup.
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
#   ── Hardware mode + D435i + T265 + RTAB-Map ──
#   ./scripts/start_ros2_nodes.sh --hw --depth-camera=d435i --t265 --rtabmap
#
#   ── Hardware mode + D435i + T265 odom (skip VO) + RTAB-Map ──
#   ./scripts/start_ros2_nodes.sh --hw --depth-camera=d435i --t265-odom --rtabmap
#
#   ── Hardware mode + D435i + VINS odom + RTAB-Map ──
#   ./scripts/start_ros2_nodes.sh --hw --depth-camera=d435i --vins-odom --rtabmap
#
#   ── Hardware mode + D435i + cuVSLAM odom + RTAB-Map ──
#   ./scripts/start_ros2_nodes.sh --hw --depth-camera=d435i --cuvslam-odom --rtabmap
#
#   ── Hardware mode + VINS odom only (T265 only, no depth camera) ──
#   ./scripts/start_ros2_nodes.sh --hw --vins-odom
#
#   ── Hardware mode + cuVSLAM odom only (T265 only, no depth camera) ──
#   ./scripts/start_ros2_nodes.sh --hw --cuvslam-odom
#
#   ── Gazebo + VINS odom + RTAB-Map ──
#   ./scripts/start_ros2_nodes.sh --vins-odom --rtabmap
#
#   ── Gazebo + cuVSLAM odom + RTAB-Map ──
#   ./scripts/start_ros2_nodes.sh --cuvslam-odom --rtabmap
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
source "${SCRIPT_DIR}/lib/dc.sh"

# Defaults
HW="false"
DEPTH_CAMERA=""
DEPTH_CAMERA_EXPLICIT="false"
HW_T265="false"
HW_T265_ODOM="false"
VINS_ODOM="false"
CUVSLAM_ODOM="false"
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
        --hw)              HW="true" ;;
        --depth-camera=*)  DEPTH_CAMERA="${arg#--depth-camera=}"; DEPTH_CAMERA_EXPLICIT="true" ;;
        --t265)            HW_T265="true" ;;
        --t265-odom)       HW_T265="true"; HW_T265_ODOM="true" ;;
        --vins-odom)       VINS_ODOM="true" ;;
        --cuvslam-odom)    CUVSLAM_ODOM="true" ;;
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
            echo "Usage: $0 [--hw] [--depth-camera=NAME] [--t265] [--t265-odom] [--vins-odom] [--cuvslam-odom] [--px4] [--rtabmap] [--nav2] [--no-rviz] [--bridge[=mode]] [--vo-bridge] [--odom-topic=TOPIC] [--build[=pkg]] [--build-only[=pkg,pkg]]"
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

# Auto-resolve depth camera: default to 'none' when only T265/VINS/cuVSLAM is needed,
# otherwise fall back to 'l515'.
if [[ -z "${DEPTH_CAMERA}" ]]; then
    if [[ "${DEPTH_CAMERA_EXPLICIT}" == "false" && "${RTABMAP}" == "false" \
          && ( "${VINS_ODOM}" == "true" || "${CUVSLAM_ODOM}" == "true" || "${HW_T265}" == "true" || "${HW_T265_ODOM}" == "true" ) ]]; then
        DEPTH_CAMERA="none"
    else
        DEPTH_CAMERA="l515"
    fi
fi

# ── Build step (if requested) ──
if [[ "${BUILD}" == "true" ]]; then
    if [[ "${CUVSLAM_ODOM}" == "true" && -z "${BUILD_PKGS}" ]]; then
        echo "Building cuVSLAM and related bringup packages..."
        dcomp exec ackermann_slam bash -lc "source /opt/ros/\$ROS_DISTRO/setup.bash && if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi && bash /workspace/scripts/build_cuvslam.sh"
    else
        BUILD_CMD="source /opt/ros/\$ROS_DISTRO/setup.bash && cd /workspace && colcon build --symlink-install"
        if [[ -n "${BUILD_PKGS}" ]]; then
        # Replace commas with spaces for --packages-select
            BUILD_CMD+=" --packages-select ${BUILD_PKGS//,/ }"
            echo "Building packages: ${BUILD_PKGS}..."
        else
            BUILD_CMD+=" --packages-ignore-regex '^example_.*' --packages-ignore px4_ros2_py"
            echo "Building all workspace packages (skipping PX4 example packages and px4_ros2_py)..."
        fi
        dcomp exec ackermann_slam bash -c "${BUILD_CMD}"
    fi
    echo ""
    if [[ "${BUILD_ONLY}" == "true" ]]; then
        echo "Build complete. Skipping launch (--build-only)."
        exit 0
    fi
fi

# ── Ensure Docker container is running ────────────────────────────────
if ! dcomp ps --services --filter status=running 2>/dev/null \
        | grep -q '^ackermann_slam$'; then
    echo "Container not running — starting ackermann_slam..."
    dcomp up -d ackermann_slam
fi

# ── Kill old ROS nodes before starting new ones ─────────────────────
# Kill previous robot_bringup / SLAM nodes but NOT rover_monitor
# (which may be running from a parallel tmux pane).
# NOTE: With pid:host the container sees host PIDs, so exclude
# docker/docker-compose processes to avoid killing ourselves.
echo "Stopping old ROS nodes..."
dcomp exec -T ackermann_slam bash -c \
    'pgrep -f "robot_bringup|rtabmap|cuvslam|realsense|rgbd_odometry|ekf_filter|point_cloud" 2>/dev/null \
     | while read pid; do
         cmdline=$(tr "\0" " " < /proc/$pid/cmdline 2>/dev/null)
         case "$cmdline" in *docker*) continue ;; esac
         kill -9 "$pid" 2>/dev/null
       done; sleep 1' 2>/dev/null || true
echo "Clean."

echo "Launching ROS 2 nodes inside Docker container..."
if [[ "${HW}" == "true" ]]; then
    echo "  Mode:         hardware (real cameras)"
    if [[ "${DEPTH_CAMERA}" != "none" ]]; then
        echo "  Depth camera: ${DEPTH_CAMERA}"
    fi
    echo "  T265:         ${HW_T265}"
    echo "  T265 odom:    ${HW_T265_ODOM}"
else
    echo "  Mode:         simulation (Gazebo)"
    echo "  Depth camera: ${DEPTH_CAMERA}"
    echo "  PX4 SITL:     ${PX4}"
fi
echo "  VINS odom:    ${VINS_ODOM}"
echo "  cuVSLAM odom: ${CUVSLAM_ODOM}"
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
LAUNCH_CMD+=" depth_camera:=${DEPTH_CAMERA}"
if [[ "${HW}" == "true" ]]; then
    LAUNCH_CMD+=" use_gazebo:=false"
    LAUNCH_CMD+=" use_sim_time:=false"
    LAUNCH_CMD+=" hw_enable_t265:=${HW_T265}"
    LAUNCH_CMD+=" use_t265_odom:=${HW_T265_ODOM}"
else
    LAUNCH_CMD+=" enable_px4_sitl:=${PX4}"
fi
LAUNCH_CMD+=" use_vins_odom:=${VINS_ODOM}"
LAUNCH_CMD+=" use_cuvslam_odom:=${CUVSLAM_ODOM}"
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

xdcomp exec ackermann_slam bash -c "${LAUNCH_CMD}"
