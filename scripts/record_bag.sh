#!/usr/bin/env bash
# Record a rosbag2 (MCAP) of camera + odometry topics for offline RTAB-Map
# tuning. Topic set mirrors the odom/camera flags used by
# scripts/start_jetson_session.sh so the same invocation works here.
#
# Output:
#   <repo>/bags/run_YYYYMMDD_HHMM/   (MCAP, zstd-compressed)
#
# Usage:
#   ./scripts/record_bag.sh                          # D435i + T265 odom
#   ./scripts/record_bag.sh --cuvslam-odom           # D435i + cuVSLAM odom
#   ./scripts/record_bag.sh --vins-odom              # D435i + VINS odom
#   ./scripts/record_bag.sh --rgbd-odom              # D435i + cuVSLAM RGBD
#   ./scripts/record_bag.sh --depth-camera=l515      # swap depth camera
#   ./scripts/record_bag.sh --name=kitchen_loop      # custom output name
#   ./scripts/record_bag.sh --extra=/foo,/bar        # append topics
#   ./scripts/record_bag.sh --wait-seconds=15        # extra settle after topics appear
#   ./scripts/record_bag.sh --dry-run                # print, don't run
#
# Stop with Ctrl-C; the bag finalizes cleanly on SIGINT.
#
# Replay later:
#   docker compose -f docker/docker-compose.yml exec ackermann_slam bash -c \
#     "source install/setup.bash && ros2 bag play /workspace/bags/run_<ts> --clock"
set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"

# ── args (flag set matches start_jetson_session.sh) ────────────────────
NAME="run_$(date +%Y%m%d_%H%M)"
DEPTH_CAMERA="d435i"
T265_ODOM=false
CUVSLAM_ODOM=false
RGBD_ODOM=false
VINS_ODOM=false
EXTRA_TOPICS=""
WAIT_SECONDS="10"
DRY_RUN=false

for arg in "$@"; do
    case "${arg}" in
        --name=*)          NAME="${arg#--name=}" ;;
        --depth-camera=*)  DEPTH_CAMERA="${arg#--depth-camera=}" ;;
        --t265-odom)       T265_ODOM=true ;;
        --cuvslam-odom)    CUVSLAM_ODOM=true ;;
        --rgbd-odom)       RGBD_ODOM=true ;;
        --vins-odom)       VINS_ODOM=true ;;
        --extra=*)         EXTRA_TOPICS="${arg#--extra=}" ;;
        --wait-seconds=*)  WAIT_SECONDS="${arg#--wait-seconds=}" ;;
        --dry-run)         DRY_RUN=true ;;
        *) echo "Unknown arg: ${arg}" >&2; exit 2 ;;
    esac
done

# Default odom source: T265 (matches the original Jetson invocation).
if [[ "${CUVSLAM_ODOM}" == false && "${RGBD_ODOM}" == false \
   && "${VINS_ODOM}" == false && "${T265_ODOM}" == false ]]; then
    T265_ODOM=true
fi

# ── topic set ──────────────────────────────────────────────────────────
TOPICS=()

# Depth camera — topic names mirror src/robot_bringup/launch/robot_bringup.launch.py (HW mode)
case "${DEPTH_CAMERA}" in
    d435i|l515)
        TOPICS+=(
            "/${DEPTH_CAMERA}/color/image_raw"
            "/${DEPTH_CAMERA}/color/camera_info"
            "/${DEPTH_CAMERA}/aligned_depth_to_color/image_raw"
            "/${DEPTH_CAMERA}/aligned_depth_to_color/camera_info"
            "/${DEPTH_CAMERA}/imu"
        )
        ;;
    none)
        ;;
    *)
        echo "Unknown --depth-camera: ${DEPTH_CAMERA} (expected d435i|l515|none)" >&2
        exit 2
        ;;
esac

# Odometry source — priority mirrors rtabmap_slam.launch.py:
#   cuVSLAM > cuVSLAM RGBD > VINS > T265. Only the selected source is recorded
#   so replay reconstructs the exact topology the launch file subscribes to.
if [[ "${CUVSLAM_ODOM}" == true ]]; then
    TOPICS+=(
        /t265/fisheye1/image_raw
        /t265/fisheye1/camera_info
        /t265/fisheye2/image_raw
        /t265/fisheye2/camera_info
        /t265/imu
    )
    TOPICS+=(/cuvslam_odom)
    ODOM_LABEL="cuvslam_odom"
elif [[ "${RGBD_ODOM}" == true ]]; then
    TOPICS+=(/cuvslam_rgbd_odom)
    ODOM_LABEL="cuvslam_rgbd_odom"
elif [[ "${VINS_ODOM}" == true ]]; then
    TOPICS+=(
        /t265/fisheye1/image_raw
        /t265/fisheye1/camera_info
        /t265/fisheye2/image_raw
        /t265/fisheye2/camera_info
        /t265/imu
    )
    TOPICS+=(/vins_odom)
    ODOM_LABEL="vins_odom"
else  # T265
    TOPICS+=(/t265/odom /t265/odom_base)
    ODOM_LABEL="t265"
fi

# Common topics — always recorded
TOPICS+=(
    /odometry/filtered    # EKF output (reference for replay comparison)
    /cmd_vel              # commands issued during the run
    /tf /tf_static        # tf_static is latched — MUST be captured for replay
)

# User-supplied extras (comma-separated)
if [[ -n "${EXTRA_TOPICS}" ]]; then
    IFS=',' read -r -a _EXTRA <<< "${EXTRA_TOPICS}"
    TOPICS+=("${_EXTRA[@]}")
fi

OUT_DIR="/workspace/bags/${NAME}"
TOPIC_ARGS="${TOPICS[*]}"
READY_TOPICS=()
MESSAGE_TOPICS=()

if [[ "${DEPTH_CAMERA}" != "none" ]]; then
    READY_TOPICS+=(
        "/${DEPTH_CAMERA}/color/image_raw"
        "/${DEPTH_CAMERA}/aligned_depth_to_color/image_raw"
        "/${DEPTH_CAMERA}/imu"
    )
    MESSAGE_TOPICS+=(
        "/${DEPTH_CAMERA}/color/image_raw"
        "/${DEPTH_CAMERA}/aligned_depth_to_color/image_raw"
        "/${DEPTH_CAMERA}/imu"
    )
fi

if [[ "${CUVSLAM_ODOM}" == true ]]; then
    READY_TOPICS+=(
        /t265/fisheye1/image_raw
        /t265/fisheye1/camera_info
        /t265/fisheye2/image_raw
        /t265/fisheye2/camera_info
        /t265/imu
    )
    MESSAGE_TOPICS+=(
        /t265/fisheye1/image_raw
        /t265/fisheye2/image_raw
        /t265/imu
    )
    READY_TOPICS+=(/cuvslam_odom)
    MESSAGE_TOPICS+=(/cuvslam_odom)
elif [[ "${RGBD_ODOM}" == true ]]; then
    READY_TOPICS+=(/cuvslam_rgbd_odom)
    MESSAGE_TOPICS+=(/cuvslam_rgbd_odom)
elif [[ "${VINS_ODOM}" == true ]]; then
    READY_TOPICS+=(
        /t265/fisheye1/image_raw
        /t265/fisheye1/camera_info
        /t265/fisheye2/image_raw
        /t265/fisheye2/camera_info
        /t265/imu
    )
    MESSAGE_TOPICS+=(
        /t265/fisheye1/image_raw
        /t265/fisheye2/image_raw
        /t265/imu
    )
    READY_TOPICS+=(/vins_odom)
    MESSAGE_TOPICS+=(/vins_odom)
else
    READY_TOPICS+=(/t265/odom /t265/odom_base)
    MESSAGE_TOPICS+=(/t265/odom /t265/odom_base)
fi

READY_TOPICS+=(
    /odometry/filtered
    /tf
    /tf_static
)
MESSAGE_TOPICS+=(
    /odometry/filtered
    /tf
    /tf_static
)

echo "─────────────────────────────────────────────────────────────"
echo "  rosbag2 record"
echo "  output:       ${OUT_DIR}"
echo "  storage:      mcap (zstd_fast)"
echo "  depth camera: ${DEPTH_CAMERA}"
echo "  odom source:  ${ODOM_LABEL}"
echo "  wait extra:   ${WAIT_SECONDS}s"
echo "  topics (${#TOPICS[@]}):"
for t in "${TOPICS[@]}"; do echo "    ${t}"; done
echo "─────────────────────────────────────────────────────────────"

if [[ "${DRY_RUN}" == true ]]; then
    echo "[dry-run] not executing"
    exit 0
fi

# Host-visible bags/ dir (container mount maps /workspace → repo root).
mkdir -p "${PROJECT_DIR}/bags"

xdcomp exec ackermann_slam bash -c "
  source /opt/ros/jazzy/setup.bash && \
  source /workspace/install/setup.bash && \
  echo 'Waiting for required topics before recording...' && \
  timeout 180 bash -lc '
    for topic in ${READY_TOPICS[*]}; do
      echo \"  waiting for \${topic}\"
      until ros2 topic list 2>/dev/null | grep -qx \${topic}; do
        sleep 1
      done
    done
    for topic in ${MESSAGE_TOPICS[*]}; do
      echo \"  waiting for first message on \${topic}\"
      if ! timeout 30 ros2 topic echo --once \${topic} >/dev/null 2>&1; then
        echo \"  WARNING: no message on \${topic} within 30s (proceeding anyway)\" >&2
      fi
    done
  ' && \
  echo 'Topics are ready. Settling for ${WAIT_SECONDS}s...' && \
  sleep ${WAIT_SECONDS} && \
  echo 'Starting rosbag2 record...' && \
  ros2 bag record \
    -o ${OUT_DIR} \
    -s mcap \
    --storage-preset-profile zstd_fast \
    ${TOPIC_ARGS}
"
