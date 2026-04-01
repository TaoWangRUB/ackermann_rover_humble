#!/usr/bin/env bash
# Launch one or more RealSense cameras inside the Docker container.
# All cameras run from a single launch file with IfCondition flags.
#
# Usage (from host):
#   ./scripts/start_cameras.sh [OPTIONS]
#
# Camera flags (default: --d435i if none specified):
#   --d435i                Launch D435i
#   --l515                 Launch L515
#   --t265                 Launch T265
#
# Per-camera options:
#   --serial-d435i=SN      USB serial number for D435i
#   --serial-l515=SN       USB serial number for L515
#   --serial-t265=SN       USB serial number for T265
#   --imu / --no-imu       Enable/disable D435i IMU (default: enabled)
#   --align-depth          Align depth to color (default: enabled)
#   --no-align-depth       Disable depth alignment
#   --infra                Enable infrared streams 1 & 2
#   --exposure-rgb=VAL     RGB exposure in us (disables auto-exposure)
#   --exposure-depth=VAL   Depth exposure in us (disables auto-exposure)
#   --gain-rgb=VAL         RGB gain
#   --gain-depth=VAL       Depth gain
#   --color-profile=WxHxF  Color profile (e.g. 640x480x30)
#   --depth-profile=WxHxF  Depth profile (e.g. 640x480x30)
#
# Build options:
#   --build[=PKG]          Build workspace (or specific pkg) before launching
#   --build-only[=PKG]     Build only, do not launch
#
# Examples:
#   ./scripts/start_cameras.sh                           # D435i only (default)
#   ./scripts/start_cameras.sh --d435i --l515            # D435i + L515
#   ./scripts/start_cameras.sh --d435i --l515 --t265     # all three
#   ./scripts/start_cameras.sh --d435i --exposure-rgb=100 --infra
#   ./scripts/start_cameras.sh --build=realsense_camera_bringup --d435i
#
# Prerequisites:
#   Docker container must be running (docker-compose up -d)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"

# --- Defaults ---
LAUNCH_D435I="false"
LAUNCH_L515="false"
LAUNCH_T265="false"
SERIAL_D435I=""
SERIAL_L515=""
SERIAL_T265=""
ENABLE_IMU="true"
ALIGN_DEPTH="true"
ENABLE_INFRA="false"
EXPOSURE_RGB=""
EXPOSURE_DEPTH=""
GAIN_RGB=""
GAIN_DEPTH=""
COLOR_PROFILE=""
DEPTH_PROFILE=""
BUILD="false"
BUILD_ONLY="false"
BUILD_PKGS=""

# --- Parse arguments ---
for arg in "$@"; do
    case "${arg}" in
        --d435i)             LAUNCH_D435I="true" ;;
        --l515)              LAUNCH_L515="true" ;;
        --t265)              LAUNCH_T265="true" ;;
        --serial-d435i=*)    SERIAL_D435I="${arg#--serial-d435i=}" ;;
        --serial-l515=*)     SERIAL_L515="${arg#--serial-l515=}" ;;
        --serial-t265=*)     SERIAL_T265="${arg#--serial-t265=}" ;;
        --imu)               ENABLE_IMU="true" ;;
        --no-imu)            ENABLE_IMU="false" ;;
        --align-depth)       ALIGN_DEPTH="true" ;;
        --no-align-depth)    ALIGN_DEPTH="false" ;;
        --infra)             ENABLE_INFRA="true" ;;
        --exposure-rgb=*)    EXPOSURE_RGB="${arg#--exposure-rgb=}" ;;
        --exposure-depth=*)  EXPOSURE_DEPTH="${arg#--exposure-depth=}" ;;
        --gain-rgb=*)        GAIN_RGB="${arg#--gain-rgb=}" ;;
        --gain-depth=*)      GAIN_DEPTH="${arg#--gain-depth=}" ;;
        --color-profile=*)   COLOR_PROFILE="${arg#--color-profile=}" ;;
        --depth-profile=*)   DEPTH_PROFILE="${arg#--depth-profile=}" ;;
        --build)             BUILD="true" ;;
        --build=*)           BUILD="true"; BUILD_PKGS="${arg#--build=}" ;;
        --build-only)        BUILD="true"; BUILD_ONLY="true" ;;
        --build-only=*)      BUILD="true"; BUILD_ONLY="true"; BUILD_PKGS="${arg#--build-only=}" ;;
        -h|--help)
            sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
            exit 0 ;;
        *)
            echo "Unknown argument: ${arg}" >&2
            echo "Run '$0 --help' for usage." >&2
            exit 1 ;;
    esac
done

# Default to D435i if no camera selected
if [[ "${LAUNCH_D435I}" == "false" && "${LAUNCH_L515}" == "false" && "${LAUNCH_T265}" == "false" ]]; then
    LAUNCH_D435I="true"
fi

# --- Build step ---
if [[ "${BUILD}" == "true" ]]; then
    BUILD_CMD="source /opt/ros/\$ROS_DISTRO/setup.bash && cd /workspace && colcon build --symlink-install"
    if [[ -n "${BUILD_PKGS}" ]]; then
        BUILD_CMD+=" --packages-select ${BUILD_PKGS//,/ }"
        echo "Building packages: ${BUILD_PKGS}..."
    else
        BUILD_CMD+=" --packages-ignore-regex '^example_.*' --packages-ignore px4_ros2_py"
        echo "Building all workspace packages (skipping PX4 example packages and px4_ros2_py)..."
    fi
    dcomp exec ackermann_slam bash -c "${BUILD_CMD}"
    echo ""
    if [[ "${BUILD_ONLY}" == "true" ]]; then
        echo "Build complete. Skipping launch (--build-only)."
        exit 0
    fi
fi

# --- Summary ---
echo "Launching RealSense cameras inside Docker container..."
[[ "${LAUNCH_D435I}" == "true" ]] && echo "  D435i:  serial=${SERIAL_D435I:-auto}  imu=${ENABLE_IMU}  align=${ALIGN_DEPTH}  infra=${ENABLE_INFRA}"
[[ "${LAUNCH_L515}"  == "true" ]] && echo "  L515:   serial=${SERIAL_L515:-auto}  align=${ALIGN_DEPTH}  infra=${ENABLE_INFRA}"
[[ "${LAUNCH_T265}"  == "true" ]] && echo "  T265:   serial=${SERIAL_T265:-auto}  (odom only)"
echo ""

# --- Build launch arguments ---
# Helper: append shared depth-camera overrides for a given prefix
_add_depth_overrides() {
    local pfx="$1"
    LAUNCH_ARGS+=" ${pfx}_align_depth:=${ALIGN_DEPTH}"
    LAUNCH_ARGS+=" ${pfx}_enable_infra1:=${ENABLE_INFRA} ${pfx}_enable_infra2:=${ENABLE_INFRA}"
    if [[ -n "${EXPOSURE_RGB}" ]]; then
        LAUNCH_ARGS+=" ${pfx}_rgb_exposure:=${EXPOSURE_RGB} ${pfx}_rgb_auto_exposure:=false"
    fi
    if [[ -n "${EXPOSURE_DEPTH}" ]]; then
        LAUNCH_ARGS+=" ${pfx}_depth_exposure:=${EXPOSURE_DEPTH} ${pfx}_depth_auto_exposure:=false"
    fi
    if [[ -n "${GAIN_RGB}" ]]; then LAUNCH_ARGS+=" ${pfx}_rgb_gain:=${GAIN_RGB}"; fi
    if [[ -n "${GAIN_DEPTH}" ]]; then LAUNCH_ARGS+=" ${pfx}_depth_gain:=${GAIN_DEPTH}"; fi
    if [[ -n "${COLOR_PROFILE}" ]]; then LAUNCH_ARGS+=" ${pfx}_color_profile:=${COLOR_PROFILE}"; fi
    if [[ -n "${DEPTH_PROFILE}" ]]; then LAUNCH_ARGS+=" ${pfx}_depth_profile:=${DEPTH_PROFILE}"; fi
}

SOURCE="source /opt/ros/\$ROS_DISTRO/setup.bash && source /workspace/install/setup.bash && _a='realsense'; _b='_camera_node'; pkill -f \"\${_a}\${_b}\" 2>/dev/null; sleep 1; ros2 daemon stop 2>/dev/null; ros2 daemon start 2>/dev/null"

LAUNCH_ARGS=""
LAUNCH_ARGS+=" enable_d435i:=${LAUNCH_D435I}"
LAUNCH_ARGS+=" enable_l515:=${LAUNCH_L515}"
LAUNCH_ARGS+=" enable_t265:=${LAUNCH_T265}"

# D435i
if [[ -n "${SERIAL_D435I}" ]]; then LAUNCH_ARGS+=" d435i_serial_no:=${SERIAL_D435I}"; fi
LAUNCH_ARGS+=" d435i_enable_imu:=${ENABLE_IMU}"
_add_depth_overrides "d435i"
# When T265 is on the same USB hub, delay D435i/L515 so T265 resets and
# starts streaming first (T265 reset takes ~8s; 12s gives a safe margin).
if [[ "${LAUNCH_T265}" == "true" ]]; then
    LAUNCH_ARGS+=" d435i_startup_delay_s:=12.0"
fi
# L515
if [[ -n "${SERIAL_L515}" ]]; then LAUNCH_ARGS+=" l515_serial_no:=${SERIAL_L515}"; fi
_add_depth_overrides "l515"
if [[ "${LAUNCH_T265}" == "true" ]]; then
    LAUNCH_ARGS+=" l515_startup_delay_s:=12.0"
fi

# T265
if [[ -n "${SERIAL_T265}" ]]; then LAUNCH_ARGS+=" t265_serial_no:=${SERIAL_T265}"; fi

LAUNCH_CMD="${SOURCE} && ros2 launch realsense_camera_bringup realsense_camera.launch.py${LAUNCH_ARGS}"

xdcomp exec ackermann_slam bash -c "${LAUNCH_CMD}"
