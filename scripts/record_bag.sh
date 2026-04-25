#!/usr/bin/env bash
# Record a rosbag2 (MCAP) of camera + odometry topics for offline RTAB-Map
# tuning. Topic set mirrors the odom/camera flags used by
# scripts/start_jetson_session.sh so the same invocation works here.
#
# Output:
#   <repo>/bags/run_YYYYMMDD_HHMM/   (MCAP, zstd-compressed)
#
# Interactive controls (TTY):
#   r   start a NEW bag segment (spawns fresh ros2 bag record)
#   s   stop & finalize current segment, keep the script alive and ready
#       for another r (each r → its own bag dir: ${NAME}_seg1, _seg2, ...)
#   Ctrl-C   finalize current segment (if any) and exit
# Without a TTY (piped / non-interactive), a single segment auto-starts
# and runs until SIGINT.
#
# Usage:
#   ./scripts/record_bag.sh                          # D435i + T265 odom
#   ./scripts/record_bag.sh --sim                    # Gazebo sim RTAB-Map VO topics
#   ./scripts/record_bag.sh --sim --depth-camera=d435i --t265-odom
#   ./scripts/record_bag.sh --sim --depth-camera=d435i --cuvslam-odom
#   ./scripts/record_bag.sh --cuvslam-odom           # D435i + cuVSLAM odom
#   ./scripts/record_bag.sh --vins-odom              # D435i + VINS odom
#   ./scripts/record_bag.sh --rgbd-odom              # D435i + cuVSLAM RGBD
#   ./scripts/record_bag.sh --depth-camera=l515      # swap depth camera
#   ./scripts/record_bag.sh --name=kitchen_loop      # custom output name
#   ./scripts/record_bag.sh --extra=/foo,/bar        # append topics
#   ./scripts/record_bag.sh --wait-seconds=15        # extra settle after topics appear
#   ./scripts/record_bag.sh --no-compress-fisheye    # record raw T265 fisheye
#   ./scripts/record_bag.sh --dry-run                # print, don't run
#
# RTAB-Map readiness gate:
#   Always blocks until the /rtabmap node is up AND has published on one of
#   its output topics (/mapData, /mapGraph, /rtabmap/*) before starting the
#   recorder, so no frames are dropped.
#
# Fisheye compression:
#   For --cuvslam-odom / --vins-odom (HW only), the T265 fisheye streams are
#   republished through image_transport (PNG lossless) and the compressed
#   topics are recorded instead of raw. Cuts fisheye bag volume ~3.3×.
#   Replay via start_rosbag_session.sh decompresses transparently.
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
SIM=false
T265_ODOM=false
CUVSLAM_ODOM=false
RGBD_ODOM=false
VINS_ODOM=false
EXTRA_TOPICS=""
WAIT_SECONDS="10"
DRY_RUN=false
COMPRESS_FISHEYE=true

for arg in "$@"; do
    case "${arg}" in
        --name=*)              NAME="${arg#--name=}" ;;
        --depth-camera=*)      DEPTH_CAMERA="${arg#--depth-camera=}" ;;
        --sim)                 SIM=true ;;
        --t265-odom)           T265_ODOM=true ;;
        --cuvslam-odom)        CUVSLAM_ODOM=true ;;
        --rgbd-odom)           RGBD_ODOM=true ;;
        --vins-odom)           VINS_ODOM=true ;;
        --extra=*)             EXTRA_TOPICS="${arg#--extra=}" ;;
        --wait-seconds=*)      WAIT_SECONDS="${arg#--wait-seconds=}" ;;
        --no-compress-fisheye) COMPRESS_FISHEYE=false ;;
        --compress-fisheye)    COMPRESS_FISHEYE=true ;;
        --dry-run)             DRY_RUN=true ;;
        *) echo "Unknown arg: ${arg}" >&2; exit 2 ;;
    esac
done

# Default odom source: T265 (matches the original Jetson invocation).
# In simulation, no explicit odom flag means "record the default Gazebo +
# RTAB-Map VO path" rather than forcing simulated T265 odom.
if [[ "${CUVSLAM_ODOM}" == false && "${RGBD_ODOM}" == false \
   && "${VINS_ODOM}" == false && "${T265_ODOM}" == false ]]; then
    if [[ "${SIM}" == false ]]; then
        T265_ODOM=true
    fi
fi

# ── topic set ──────────────────────────────────────────────────────────
TOPICS=()

if [[ "${SIM}" == true ]]; then
    # Simulation topics — names come from the Gazebo ros_gz_bridge
    # (see src/description_robot/launch/gazebo_bringup.launch.py).
    case "${DEPTH_CAMERA}" in
        d435i|l515)
            TOPICS+=(
                "/${DEPTH_CAMERA}/image"
                "/${DEPTH_CAMERA}/camera_info"
                "/${DEPTH_CAMERA}/depth_image"
                "/${DEPTH_CAMERA}/imu/raw"
            )
            ;;
        none) ;;
        *) echo "Unknown --depth-camera: ${DEPTH_CAMERA}" >&2; exit 2 ;;
    esac

    # Simulation odometry source — priority mirrors rtabmap_slam.launch.py:
    #   cuVSLAM > cuVSLAM RGBD > VINS > T265 > RTAB-Map VO/default sim topics.
    if [[ "${CUVSLAM_ODOM}" == true ]]; then
        TOPICS+=(
            /t265/fisheye1/image_raw
            /t265/fisheye1/camera_info
            /t265/fisheye2/image_raw
            /t265/fisheye2/camera_info
            /t265/imu
            /cuvslam_odom
        )
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
            /vins_odom
        )
        ODOM_LABEL="vins_odom"
    elif [[ "${T265_ODOM}" == true ]]; then
        TOPICS+=(/t265/odom_base)
        ODOM_LABEL="t265"
    else
        # Default sim recording keeps both controller odom and RTAB-Map VO so
        # replay/debugging can compare wheel and visual odometry.
        TOPICS+=(/ackermann_steering_controller/odometry /vo_odom)
        ODOM_LABEL="sim_controller_vo"
    fi
else
    # Hardware topics — names come from realsense_camera_bringup
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
        none) ;;
        *)
            echo "Unknown --depth-camera: ${DEPTH_CAMERA} (expected d435i|l515|none)" >&2
            exit 2
            ;;
    esac

    # T265 native odometry is recorded unconditionally on hardware: the T265
    # is always plugged in regardless of which downstream odom source feeds
    # RTAB-Map, and keeping it in every bag lets offline replay compare any
    # other odometry path (cuVSLAM / VINS / RGBD) against the T265 baseline.
    TOPICS+=(/t265/odom /t265/odom_base)

    # Selected odometry source — priority mirrors rtabmap_slam.launch.py:
    #   cuVSLAM > cuVSLAM RGBD > VINS > T265. Adds the source-specific topics
    #   so replay reconstructs the exact topology the launch file subscribes to.
    # When --compress-fisheye (default) is on for cuvslam/vins modes, the
    # fisheye image topic swapped to the compressed variant; the raw stream is
    # republished through image_transport before ros2 bag record starts.
    if [[ "${CUVSLAM_ODOM}" == true || "${VINS_ODOM}" == true ]]; then
        if [[ "${COMPRESS_FISHEYE}" == true ]]; then
            FISHEYE_TOPIC_1="/t265/fisheye1/image_raw/compressed"
            FISHEYE_TOPIC_2="/t265/fisheye2/image_raw/compressed"
        else
            FISHEYE_TOPIC_1="/t265/fisheye1/image_raw"
            FISHEYE_TOPIC_2="/t265/fisheye2/image_raw"
        fi
    fi

    if [[ "${CUVSLAM_ODOM}" == true ]]; then
        TOPICS+=(
            "${FISHEYE_TOPIC_1}"
            /t265/fisheye1/camera_info
            "${FISHEYE_TOPIC_2}"
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
            "${FISHEYE_TOPIC_1}"
            /t265/fisheye1/camera_info
            "${FISHEYE_TOPIC_2}"
            /t265/fisheye2/camera_info
            /t265/imu
        )
        TOPICS+=(/vins_odom)
        ODOM_LABEL="vins_odom"
    else  # T265
        ODOM_LABEL="t265"
    fi
fi

# Common topics — always recorded
TOPICS+=(
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

if [[ "${SIM}" == true ]]; then
    if [[ "${DEPTH_CAMERA}" != "none" ]]; then
        READY_TOPICS+=("/${DEPTH_CAMERA}/image" "/${DEPTH_CAMERA}/depth_image")
        MESSAGE_TOPICS+=("/${DEPTH_CAMERA}/image" "/${DEPTH_CAMERA}/depth_image")
    fi
    if [[ "${CUVSLAM_ODOM}" == true ]]; then
        READY_TOPICS+=(
            /t265/fisheye1/image_raw
            /t265/fisheye1/camera_info
            /t265/fisheye2/image_raw
            /t265/fisheye2/camera_info
            /t265/imu
            /cuvslam_odom
        )
        MESSAGE_TOPICS+=(
            /t265/fisheye1/image_raw
            /t265/fisheye2/image_raw
            /t265/imu
            /cuvslam_odom
        )
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
            /vins_odom
        )
        MESSAGE_TOPICS+=(
            /t265/fisheye1/image_raw
            /t265/fisheye2/image_raw
            /t265/imu
            /vins_odom
        )
    elif [[ "${T265_ODOM}" == true ]]; then
        READY_TOPICS+=(/t265/odom_base)
        MESSAGE_TOPICS+=(/t265/odom_base)
    else
        READY_TOPICS+=(/ackermann_steering_controller/odometry /vo_odom)
        MESSAGE_TOPICS+=(/ackermann_steering_controller/odometry /vo_odom)
    fi
else
    if [[ "${DEPTH_CAMERA}" != "none" ]]; then
        READY_TOPICS+=(
            "/${DEPTH_CAMERA}/color/image_raw"
            "/${DEPTH_CAMERA}/aligned_depth_to_color/image_raw"
            "/${DEPTH_CAMERA}/imu"
        )
        MESSAGE_TOPICS+=(
            "/${DEPTH_CAMERA}/color/image_raw"
            "/${DEPTH_CAMERA}/aligned_depth_to_color/image_raw"
        )
    fi

    # T265 native odometry is recorded unconditionally on hardware (see the
    # TOPICS section above). Gate startup on it for every odom mode so we
    # don't begin recording before the T265 streams are flowing.
    READY_TOPICS+=(/t265/odom /t265/odom_base)
    MESSAGE_TOPICS+=(/t265/odom /t265/odom_base)

    if [[ "${CUVSLAM_ODOM}" == true ]]; then
        READY_TOPICS+=(
            "${FISHEYE_TOPIC_1}"
            /t265/fisheye1/camera_info
            "${FISHEYE_TOPIC_2}"
            /t265/fisheye2/camera_info
            /t265/imu
        )
        MESSAGE_TOPICS+=(
            "${FISHEYE_TOPIC_1}"
            "${FISHEYE_TOPIC_2}"
            /t265/imu
        )
        READY_TOPICS+=(/cuvslam_odom)
        MESSAGE_TOPICS+=(/cuvslam_odom)
    elif [[ "${RGBD_ODOM}" == true ]]; then
        READY_TOPICS+=(/cuvslam_rgbd_odom)
        MESSAGE_TOPICS+=(/cuvslam_rgbd_odom)
    elif [[ "${VINS_ODOM}" == true ]]; then
        READY_TOPICS+=(
            "${FISHEYE_TOPIC_1}"
            /t265/fisheye1/camera_info
            "${FISHEYE_TOPIC_2}"
            /t265/fisheye2/camera_info
            /t265/imu
        )
        MESSAGE_TOPICS+=(
            "${FISHEYE_TOPIC_1}"
            "${FISHEYE_TOPIC_2}"
            /t265/imu
        )
        READY_TOPICS+=(/vins_odom)
        MESSAGE_TOPICS+=(/vins_odom)
    fi
fi

READY_TOPICS+=(
    /tf
    /tf_static
)
MESSAGE_TOPICS+=(
    /tf
    /tf_static
)

# Fisheye compression is only meaningful on hardware cuvslam/vins modes
# (those are the only modes that record fisheye streams).
FISHEYE_COMPRESS_ACTIVE=false
if [[ "${SIM}" == false && "${COMPRESS_FISHEYE}" == true \
   && ( "${CUVSLAM_ODOM}" == true || "${VINS_ODOM}" == true ) ]]; then
    FISHEYE_COMPRESS_ACTIVE=true
fi

echo "─────────────────────────────────────────────────────────────"
echo "  rosbag2 record"
echo "  output:       ${OUT_DIR}"
echo "  storage:      mcap (zstd_fast)"
echo "  depth camera: ${DEPTH_CAMERA}"
echo "  odom source:  ${ODOM_LABEL}"
echo "  wait extra:   ${WAIT_SECONDS}s"
if [[ "${SIM}" == false && ( "${CUVSLAM_ODOM}" == true || "${VINS_ODOM}" == true ) ]]; then
    echo "  fisheye:      $( [[ "${FISHEYE_COMPRESS_ACTIVE}" == true ]] && echo 'compressed (PNG)' || echo 'raw' )"
fi
echo "  topics (${#TOPICS[@]}):"
for t in "${TOPICS[@]}"; do echo "    ${t}"; done
echo "─────────────────────────────────────────────────────────────"

if [[ "${DRY_RUN}" == true ]]; then
    echo "[dry-run] not executing"
    exit 0
fi

# Host-visible bags/ dir (container mount maps /workspace → repo root).
mkdir -p "${PROJECT_DIR}/bags"

# Build republisher launch prelude (compressed mode only). The republishers
# subscribe to the RAW fisheye topics and publish CompressedImage on the
# /compressed child topics that rosbag2 records. The unified cleanup()
# function below stops them when the exec block exits.
REPUBLISH_START=""
if [[ "${FISHEYE_COMPRESS_ACTIVE}" == true ]]; then
    REPUBLISH_START="
  echo 'Starting image_transport republishers (raw -> PNG)...' && \
  ros2 run image_transport republish raw compressed \
      --ros-args \
      -r in:=/t265/fisheye1/image_raw \
      -r out/compressed:=/t265/fisheye1/image_raw/compressed \
      -p format:=png -p png_level:=3 \
      >/tmp/republish_fisheye1.log 2>&1 & \
  echo \"  fisheye1 republisher pid=\$!\" && \
  ros2 run image_transport republish raw compressed \
      --ros-args \
      -r in:=/t265/fisheye2/image_raw \
      -r out/compressed:=/t265/fisheye2/image_raw/compressed \
      -p format:=png -p png_level:=3 \
      >/tmp/republish_fisheye2.log 2>&1 & \
  echo \"  fisheye2 republisher pid=\$!\" && \
  sleep 2 && \\"
fi

# Block on the /rtabmap node being up AND publishing a message. Waits up to
# 60s for the node and then up to 30s for the first message on any of the
# candidate output topics (topic names depend on launch: with namespace they
# live under /rtabmap/, without they're at root). Prints a WARNING and
# proceeds if neither signal arrives in time.
RTABMAP_WAIT_CMD="
  echo 'Waiting for /rtabmap node...' && \
  timeout 60 bash -lc 'until ros2 node list 2>/dev/null | grep -qx /rtabmap; do sleep 1; done' && \
  echo 'Waiting for RTAB-Map to publish...' && \
  bash -lc 'found=false; for t in /mapData /mapGraph /rtabmap/mapData /rtabmap/mapGraph /rtabmap/info /info; do if ros2 topic info \$t >/dev/null 2>&1 && timeout 30 ros2 topic echo --once \$t >/dev/null 2>&1; then echo \"  got message on \$t\"; found=true; break; fi; done; if [ \$found = false ]; then echo \"  WARNING: /rtabmap up but no messages within 30s (proceeding anyway)\" >&2; fi' && \\"

xdcomp exec ackermann_slam bash -c "
  source /opt/ros/jazzy/setup.bash
  source /workspace/install/setup.bash

  SEGMENT=0
  REC_PID=''

  # SIGTERM the current recorder and wait up to 30s for it to finalize the
  # MCAP + metadata.yaml. Escalates to SIGKILL as a last resort. No-op if
  # no recorder is running.
  stop_segment() {
    if [ -n \"\$REC_PID\" ] && kill -0 \$REC_PID 2>/dev/null; then
      kill -TERM \$REC_PID 2>/dev/null || true
      for _ in \$(seq 1 60); do
        kill -0 \$REC_PID 2>/dev/null || break
        sleep 0.5
      done
      if kill -0 \$REC_PID 2>/dev/null; then
        echo 'Recorder still alive after 30s, sending SIGKILL' >&2
        kill -KILL \$REC_PID 2>/dev/null || true
      fi
      wait \$REC_PID 2>/dev/null || true
    fi
    REC_PID=''
  }

  # Spawn a new ros2 bag record into its own segment directory. If the
  # segN path already exists (e.g. from a prior interrupted run), fall
  # back to a timestamped suffix so rosbag2 doesn't refuse to start.
  start_segment() {
    SEGMENT=\$((SEGMENT+1))
    SEG_DIR=\"${OUT_DIR}_seg\${SEGMENT}\"
    if [ -d \"\$SEG_DIR\" ]; then
      SEG_DIR=\"\${SEG_DIR}_\$(date +%H%M%S)\"
    fi
    ros2 bag record \
      -o \"\$SEG_DIR\" \
      -s mcap \
      --storage-preset-profile zstd_fast \
      ${TOPIC_ARGS} \
      > /tmp/rosbag2_recorder_seg\${SEGMENT}.log 2>&1 &
    REC_PID=\$!
    echo \"[recording -> \$(basename \$SEG_DIR)]\"
  }

  cleanup() {
    stty sane 2>/dev/null || true
    printf '\n'
    if [ -n \"\$REC_PID\" ] && kill -0 \$REC_PID 2>/dev/null; then
      echo 'Finalizing current segment (SIGTERM -> recorder)...'
      # ros2 bag record's CLI wrapper ignores SIGINT from kill(2) (only
      # reacts to Ctrl-C from a controlling TTY). SIGTERM is forwarded
      # and cleanly finalizes the MCAP + metadata.yaml.
      stop_segment
    fi
    pkill -INT -f 'image_transport/republish raw compressed' 2>/dev/null || true
    sleep 1
    pkill -f 'image_transport/republish raw compressed' 2>/dev/null || true
  }
  trap cleanup EXIT INT TERM

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
  ' && ${RTABMAP_WAIT_CMD} ${REPUBLISH_START}
  echo 'Topics are ready. Settling for ${WAIT_SECONDS}s...'
  sleep ${WAIT_SECONDS}

  if [ -t 0 ]; then
    echo
    echo '==================================================================='
    echo '  r = START new bag   s = STOP & finalize current   Ctrl-C = exit'
    echo '==================================================================='
    echo '[idle]'
    stty -icanon -echo min 1 time 0
    while IFS= read -r -n 1 key; do
      case \"\$key\" in
        r|R)
          if [ -z \"\$REC_PID\" ] || ! kill -0 \$REC_PID 2>/dev/null; then
            start_segment
          else
            echo 'Already recording — press s first to stop the current segment.'
          fi
          ;;
        s|S)
          if [ -n \"\$REC_PID\" ] && kill -0 \$REC_PID 2>/dev/null; then
            echo 'Stopping current segment...'
            stop_segment
            echo '[idle]'
          fi
          ;;
      esac
    done
  else
    echo 'No TTY detected — recording a single segment (send SIGINT to finalize)'
    start_segment
    wait \$REC_PID
  fi
"
