#!/usr/bin/env bash
# Launch one or more RealSense cameras inside the Docker container.
# Each requested camera runs as its own realsense_camera_node instance.
#
# Usage (from host):
#   ./scripts/start_cameras.sh [OPTIONS]
#
# Camera flags (default: --l515 if none specified):
#   --d435i                Launch D435i (camera_name=d435i)
#   --l515                 Launch L515  (camera_name=l515)
#   --t265                 Launch T265  (camera_name=t265)
#
# Per-camera options:
#   --serial-d435i=SN      USB serial number for D435i (default: auto)
#   --serial-l515=SN       USB serial number for L515  (default: auto)
#   --serial-t265=SN       USB serial number for T265  (default: auto)
#   --imu                  Enable IMU for D435i and L515 (default: disabled)
#   --align-depth          Align depth to color for D435i / L515
#
# Build options:
#   --build[=PKG]          Build workspace (or specific pkg) before launching
#   --build-only[=PKG]     Build only, do not launch
#
# Examples:
#   # Default: launch L515 only
#   ./scripts/start_cameras.sh
#
#   # Launch D435i only
#   ./scripts/start_cameras.sh --d435i
#
#   # Launch all three cameras
#   ./scripts/start_cameras.sh --d435i --l515 --t265
#
#   # D435i with IMU + T265 tracking
#   ./scripts/start_cameras.sh --d435i --t265 --imu
#
#   # Specific serial numbers (when multiple of same model connected)
#   ./scripts/start_cameras.sh --d435i --serial-d435i=123456789 --l515 --serial-l515=987654321
#
#   # Build package then launch
#   ./scripts/start_cameras.sh --d435i --l515 --build=realsense_camera_bingup
#
# Prerequisites:
#   Docker container must be running (docker-compose up -d)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${PROJECT_DIR}/docker/docker-compose.yml"

# --- Defaults ---
LAUNCH_D435I="false"
LAUNCH_L515="false"
LAUNCH_T265="false"
SERIAL_D435I=""
SERIAL_L515=""
SERIAL_T265=""
ENABLE_IMU="false"
ALIGN_DEPTH="false"
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
        --align-depth)       ALIGN_DEPTH="true" ;;
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

# Default to L515 if no camera selected
if [[ "${LAUNCH_D435I}" == "false" && "${LAUNCH_L515}" == "false" && "${LAUNCH_T265}" == "false" ]]; then
    LAUNCH_L515="true"
fi

# --- Build step ---
if [[ "${BUILD}" == "true" ]]; then
    BUILD_CMD="source /opt/ros/\$ROS_DISTRO/setup.bash && cd /workspace && colcon build --symlink-install"
    if [[ -n "${BUILD_PKGS}" ]]; then
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

# --- Summary ---
echo "Launching RealSense cameras inside Docker container..."
[[ "${LAUNCH_D435I}" == "true" ]] && echo "  D435i:  serial=${SERIAL_D435I:-auto}  imu=${ENABLE_IMU}  align_depth=${ALIGN_DEPTH}"
[[ "${LAUNCH_L515}"  == "true" ]] && echo "  L515:   serial=${SERIAL_L515:-auto}  imu=${ENABLE_IMU}  align_depth=${ALIGN_DEPTH}"
[[ "${LAUNCH_T265}"  == "true" ]] && echo "  T265:   serial=${SERIAL_T265:-auto}"
echo ""

# --- Build inner launch command ---
SOURCE="source /opt/ros/\$ROS_DISTRO/setup.bash && source /workspace/install/setup.bash && _a='realsense'; _b='_camera_node'; pkill -f \"\${_a}\${_b}\" 2>/dev/null; sleep 1; ros2 daemon stop 2>/dev/null; ros2 daemon start 2>/dev/null"
LAUNCH_CMD="${SOURCE} && PIDS='';"

_add_camera() {
    local model="$1"   # d435i | l515 | t265
    local serial="$2"
    local extra="$3"   # additional ros2 launch args

    LAUNCH_CMD+=" ros2 launch realsense_camera_bingup realsense_camera.launch.py"
    LAUNCH_CMD+=" camera_model:=${model}"
    LAUNCH_CMD+=" camera_name:=${model}"
    [[ -n "${serial}" ]] && LAUNCH_CMD+=" serial_no:=${serial}"
    LAUNCH_CMD+=" ${extra} &"
    LAUNCH_CMD+=" PIDS=\"\$PIDS \$!\";"
}

DEPTH_EXTRA="enable_imu:=${ENABLE_IMU} align_depth_to_color:=${ALIGN_DEPTH}"

[[ "${LAUNCH_D435I}" == "true" ]] && _add_camera "d435i" "${SERIAL_D435I}" "${DEPTH_EXTRA}"
[[ "${LAUNCH_L515}"  == "true" ]] && _add_camera "l515"  "${SERIAL_L515}"  "${DEPTH_EXTRA}"
[[ "${LAUNCH_T265}"  == "true" ]] && _add_camera "t265"  "${SERIAL_T265}"  ""

LAUNCH_CMD+=" trap 'kill -INT \$PIDS 2>/dev/null; sleep 2; kill \$PIDS 2>/dev/null; wait' EXIT INT TERM;"
LAUNCH_CMD+=" wait"

exec docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -c "${LAUNCH_CMD}"
