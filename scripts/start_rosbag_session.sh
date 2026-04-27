#!/usr/bin/env bash
# Start a tmux session for rosbag recording (--record) or offline RTAB-Map
# tuning from a recorded bag (--replay, default).
#
# ── --replay (default) ──────────────────────────────────────────────
# Launches (in dependency order):
#   1. RTAB-Map SLAM (with rtabmap_viz)     — immediate, use_sim_time:=true
#   2. Bag playback                          — after RTAB-Map is ready
#   3. Inspection shell                      — immediate
#
# Layout (3 panes):
#   ┌─────────────────────────┬──────────────────────┐
#   │  RTAB-Map + viz         │                      │
#   │  (use_sim_time:=true)   │  Inspection shell    │
#   ├─────────────────────────┤  (container bash)    │
#   │  Bag playback           │                      │
#   │  (ros2 bag play --clock)│                      │
#   └─────────────────────────┴──────────────────────┘
#
# ── --record ────────────────────────────────────────────────────────
# Launches:
#   1. ros2 bag record (via scripts/record_bag.sh, waits idle for r)
#   2. Inspection shell
#
# Layout (2 panes):
#   ┌─────────────────────────┬──────────────────────┐
#   │  ros2 bag record        │  Inspection shell    │
#   │  (record_bag.sh)        │  (container bash)    │
#   └─────────────────────────┴──────────────────────┘
#
# Keyboard controls (in the record pane):
#   r        start a NEW bag segment (spawns fresh ros2 bag record)
#   s        stop & finalize current segment; script stays ready for
#            another r. Each r creates its own dir: NAME_seg1, _seg2, ...
#   Ctrl-C   finalize current segment (if any) and exit
#
# Assumes the live stack is already running (e.g. via start_jetson_session.sh).
# Fisheye compression (PNG via image_transport) is on by default for
# --cuvslam-odom / --vins-odom; turn off with --no-compress-fisheye.
#
# Usage:
#   # Replay (default)
#   ./scripts/start_rosbag_session.sh                              # latest bag, T265 odom
#   ./scripts/start_rosbag_session.sh --name=run_20260420_2155     # specific bag
#   ./scripts/start_rosbag_session.sh --cuvslam-odom               # match record flags
#   ./scripts/start_rosbag_session.sh --rate=0.5 --loop
#   ./scripts/start_rosbag_session.sh --keep-db                    # keep existing DB
#   ./scripts/start_rosbag_session.sh --no-viz                     # headless
#
#   # Record (default name: run_YYYYMMDD_HHMM; override with --name=...)
#   ./scripts/start_rosbag_session.sh --record --cuvslam-odom
#   ./scripts/start_rosbag_session.sh --record --t265-odom --name=kitchen_loop
#   ./scripts/start_rosbag_session.sh --record --vins-odom --wait-seconds=15
#   ./scripts/start_rosbag_session.sh --record --cuvslam-odom --no-compress-fisheye
#
# Compressed fisheye bags (record_bag.sh default for cuvslam/vins):
#   On replay, if the bag contains /t265/fisheye{1,2}/image_raw/compressed,
#   image_transport republishers are started automatically to decompress
#   them back to /t265/fisheye{1,2}/image_raw so downstream consumers
#   (rtabmap_viz, rviz, a live cuVSLAM re-run) still see raw topics.
#
# To stop everything:
#   ./scripts/stop_all.sh --session=rosbag
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"

MODE="replay"
SESSION="rosbag"
BAG_NAME=""
BAG_PATH_OVERRIDE=""
RATE="1.0"
LOOP=false
START_OFFSET=""
DEPTH_CAMERA="d435i"
T265_ODOM=false
CUVSLAM_ODOM=false
RGBD_ODOM=false
VINS_ODOM=false
RTABMAP_VIZ=true
DELETE_DB=true
ATTACH=true
WAIT_SECONDS="10"
EXTRA_TOPICS=""
COMPRESS_FISHEYE=""   # unset → let record_bag.sh default apply

for arg in "$@"; do
    case "${arg}" in
        --record)              MODE="record" ;;
        --replay)              MODE="replay" ;;
        --name=*)              BAG_NAME="${arg#--name=}" ;;
        --path=*)              BAG_PATH_OVERRIDE="${arg#--path=}" ;;
        --rate=*)              RATE="${arg#--rate=}" ;;
        --loop)                LOOP=true ;;
        --start=*)             START_OFFSET="${arg#--start=}" ;;
        --depth-camera=*)      DEPTH_CAMERA="${arg#--depth-camera=}" ;;
        --t265-odom)           T265_ODOM=true ;;
        --cuvslam-odom)        CUVSLAM_ODOM=true ;;
        --rgbd-odom)           RGBD_ODOM=true ;;
        --vins-odom)           VINS_ODOM=true ;;
        --no-viz)              RTABMAP_VIZ=false ;;
        --keep-db)             DELETE_DB=false ;;
        --delete-db)           DELETE_DB=true ;;
        --wait-seconds=*)      WAIT_SECONDS="${arg#--wait-seconds=}" ;;
        --extra=*)             EXTRA_TOPICS="${arg#--extra=}" ;;
        --compress-fisheye)    COMPRESS_FISHEYE=true ;;
        --no-compress-fisheye) COMPRESS_FISHEYE=false ;;
        --no-attach)           ATTACH=false ;;
        --session=*)           SESSION="${arg#--session=}" ;;
        -h|--help)
            sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
            exit 0 ;;
        *) echo "Unknown arg: ${arg}" >&2; exit 2 ;;
    esac
done

# Default odom: T265 (matches record_bag.sh default)
if [[ "${CUVSLAM_ODOM}" == false && "${RGBD_ODOM}" == false \
   && "${VINS_ODOM}" == false && "${T265_ODOM}" == false ]]; then
    T265_ODOM=true
fi

# Odom label for summary
if   [[ "${CUVSLAM_ODOM}" == true ]]; then ODOM_LABEL="cuvslam_odom"
elif [[ "${RGBD_ODOM}"    == true ]]; then ODOM_LABEL="cuvslam_rgbd_odom"
elif [[ "${VINS_ODOM}"    == true ]]; then ODOM_LABEL="vins_odom"
else                                       ODOM_LABEL="t265"
fi

ROS_SRC="source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash"

# ── Ensure container is running ──────────────────────────────────────
if ! dcomp ps --services --filter status=running 2>/dev/null \
        | grep -q '^ackermann_slam$'; then
    echo "Container not running — starting ackermann_slam..."
    dcomp up -d ackermann_slam
fi

# ── Kill old session ─────────────────────────────────────────────────
# Record mode assumes a live stack is already running (cameras, RTAB-Map,
# controllers) and only needs to clear the old rosbag tmux session.
# Replay mode wants a clean slate — it launches its own RTAB-Map against
# a bag, so kill any in-container ROS/Gazebo processes first.
if [[ "${MODE}" == "record" ]]; then
    tmux kill-session -t "${SESSION}" 2>/dev/null || true
else
    "${SCRIPT_DIR}/stop_all.sh" --session="${SESSION}" 2>/dev/null || true
fi
sleep 2

# =====================================================================
# ──  RECORD MODE  ─────────────────────────────────────────────────────
# =====================================================================
if [[ "${MODE}" == "record" ]]; then
    # Default bag name includes a timestamp so each record run is unique and
    # the summary shows the resolved name. record_bag.sh has the same default
    # but resolving it here means the user sees the exact path up-front.
    [[ -z "${BAG_NAME}" ]] && BAG_NAME="run_$(date +%Y%m%d_%H%M)"

    # Build record_bag.sh arg list
    REC_ARGS=("--name=${BAG_NAME}")
    REC_ARGS+=("--depth-camera=${DEPTH_CAMERA}")
    [[ "${T265_ODOM}"    == true ]]   && REC_ARGS+=("--t265-odom")
    [[ "${CUVSLAM_ODOM}" == true ]]   && REC_ARGS+=("--cuvslam-odom")
    [[ "${RGBD_ODOM}"    == true ]]   && REC_ARGS+=("--rgbd-odom")
    [[ "${VINS_ODOM}"    == true ]]   && REC_ARGS+=("--vins-odom")
    REC_ARGS+=("--wait-seconds=${WAIT_SECONDS}")
    [[ -n "${EXTRA_TOPICS}" ]]        && REC_ARGS+=("--extra=${EXTRA_TOPICS}")
    [[ "${COMPRESS_FISHEYE}" == true  ]] && REC_ARGS+=("--compress-fisheye")
    [[ "${COMPRESS_FISHEYE}" == false ]] && REC_ARGS+=("--no-compress-fisheye")

    echo "─────────────────────────────────────────────────────────────"
    echo "  rosbag record session: ${SESSION}"
    echo "  name:         ${BAG_NAME}"
    echo "  output:       ${PROJECT_DIR}/bags/${BAG_NAME}"
    echo "  depth camera: ${DEPTH_CAMERA}"
    echo "  odom source:  ${ODOM_LABEL}"
    echo "  record_bag.sh args: ${REC_ARGS[*]}"
    echo "─────────────────────────────────────────────────────────────"

    tmux kill-session -t "${SESSION}" 2>/dev/null || true
    tmux new-session -d -s "${SESSION}" -n "record"
    tmux set-option -t "${SESSION}" -g mouse on
    tmux set-option -t "${SESSION}" -g pane-base-index 0

    PANE_RECORD="$(tmux display-message -p -t "${SESSION}:record.0" "#{pane_id}")"
    PANE_INSPECT="$(tmux split-window -h -P -F "#{pane_id}" -t "${PANE_RECORD}")"

    # Pane: recorder — record_bag.sh wraps ros2 bag record and exec's into container
    tmux send-keys -t "${PANE_RECORD}" \
        "${SCRIPT_DIR}/record_bag.sh ${REC_ARGS[*]}" Enter

    # Pane: inspection shell
    tmux send-keys -t "${PANE_INSPECT}" \
        "source ${SCRIPT_DIR}/lib/dc.sh && xdcomp exec ackermann_slam bash" Enter

    tmux select-pane -t "${PANE_RECORD}"

    if [[ "${ATTACH}" == true && -t 1 ]]; then
        tmux attach-session -t "${SESSION}"
    else
        echo ""
        echo "Session '${SESSION}' started detached. Attach with:"
        echo "  tmux attach -t ${SESSION}"
        echo "Stop with Ctrl-C in the record pane (bag finalizes on SIGINT),"
        echo "then: ./scripts/stop_all.sh --session=${SESSION}"
    fi
    exit 0
fi

# =====================================================================
# ──  REPLAY MODE  ─────────────────────────────────────────────────────
# =====================================================================

# ── Resolve bag path ──────────────────────────────────────────────────
LATEST_HOST=""
if [[ -n "${BAG_PATH_OVERRIDE}" ]]; then
    BAG_PATH="${BAG_PATH_OVERRIDE}"
elif [[ -n "${BAG_NAME}" ]]; then
    BAG_PATH="/workspace/bags/${BAG_NAME}"
else
    LATEST_HOST="$(ls -1dt "${PROJECT_DIR}/bags"/run_* 2>/dev/null | head -n1 || true)"
    if [[ -z "${LATEST_HOST}" ]]; then
        echo "No bags under ${PROJECT_DIR}/bags. Record one first:" >&2
        echo "  ${SCRIPT_DIR}/start_rosbag_session.sh --record --cuvslam-odom" >&2
        exit 1
    fi
    BAG_PATH="/workspace/bags/$(basename "${LATEST_HOST}")"
fi

# ── Build RTAB-Map launch args ───────────────────────────────────────
# rtabmap_slam.launch.py has no `depth_camera` arg — topic names are set
# via rgb_*/depth_*/imu_* args. robot_bringup.launch.py does this remapping
# automatically; we replicate it here for the bag-replay path. Bags are
# recorded on HW, so HW topic naming applies.
RTABMAP_ARGS=(
    "use_sim_time:=true"
    "rtabmap_viz:=${RTABMAP_VIZ}"
    "delete_db_on_start:=${DELETE_DB}"
    "rtabmap_detection_rate:=0"
    "rtabmap_vis_min_inliers:=12"
    "rtabmap_vis_max_features:=1500"
    "rgb_image_topic:=/${DEPTH_CAMERA}/color/image_raw"
    "rgb_camera_info_topic:=/${DEPTH_CAMERA}/color/camera_info"
    "depth_image_topic:=/${DEPTH_CAMERA}/aligned_depth_to_color/image_raw"
    "depth_camera_info_topic:=/${DEPTH_CAMERA}/aligned_depth_to_color/camera_info"
    "imu_raw_topic:=/${DEPTH_CAMERA}/imu"
)
if [[ "${CUVSLAM_ODOM}" == true ]]; then
    RTABMAP_ARGS+=("use_cuvslam_odom:=true")
elif [[ "${RGBD_ODOM}" == true ]]; then
    RTABMAP_ARGS+=("use_rgbd_odom:=true")
elif [[ "${VINS_ODOM}" == true ]]; then
    RTABMAP_ARGS+=("use_vins_odom:=true")
else
    RTABMAP_ARGS+=("use_t265_odom:=true")
fi

# ── Build bag play command ────────────────────────────────────────────
PLAY_ARGS=("${BAG_PATH}" --clock --rate "${RATE}")
[[ "${LOOP}" == true ]]    && PLAY_ARGS+=(--loop)
[[ -n "${START_OFFSET}" ]] && PLAY_ARGS+=(--start-offset "${START_OFFSET}")

# ── Detect compressed fisheye topics in the bag ──────────────────────
HOST_BAG_PATH=""
if [[ -n "${BAG_PATH_OVERRIDE}" ]]; then
    HOST_BAG_PATH="${BAG_PATH_OVERRIDE/#\/workspace/${PROJECT_DIR}}"
elif [[ -n "${BAG_NAME}" ]]; then
    HOST_BAG_PATH="${PROJECT_DIR}/bags/${BAG_NAME}"
else
    HOST_BAG_PATH="${LATEST_HOST}"
fi
FISHEYE_DECOMPRESS=false
if [[ -f "${HOST_BAG_PATH}/metadata.yaml" ]] \
   && grep -q '/t265/fisheye1/image_raw/compressed' "${HOST_BAG_PATH}/metadata.yaml"; then
    FISHEYE_DECOMPRESS=true
fi

# ── Summary ──────────────────────────────────────────────────────────
echo "─────────────────────────────────────────────────────────────"
echo "  rosbag replay session: ${SESSION}"
echo "  bag:          ${BAG_PATH}"
echo "  rate:         ${RATE}  loop=${LOOP}"
[[ -n "${START_OFFSET}" ]] && echo "  start:        +${START_OFFSET}s"
echo "  depth camera: ${DEPTH_CAMERA}"
echo "  odom source:  ${ODOM_LABEL}"
echo "  delete db:    ${DELETE_DB}"
echo "  rtabmap viz:  ${RTABMAP_VIZ}"
echo "  fisheye:      $( [[ "${FISHEYE_DECOMPRESS}" == true ]] && echo 'compressed -> decompress' || echo 'raw (no decompress)' )"
echo "─────────────────────────────────────────────────────────────"

# ── Create tmux session ──────────────────────────────────────────────
tmux kill-session -t "${SESSION}" 2>/dev/null || true
tmux new-session -d -s "${SESSION}" -n "rosbag"
tmux set-option -t "${SESSION}" -g mouse on
tmux set-option -t "${SESSION}" -g pane-base-index 0

PANE_RTABMAP="$(tmux display-message -p -t "${SESSION}:rosbag.0" "#{pane_id}")"
PANE_INSPECT="$(tmux split-window -h -P -F "#{pane_id}" -t "${PANE_RTABMAP}")"
PANE_BAG="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_RTABMAP}")"

# ── Pane: RTAB-Map (immediate) ───────────────────────────────────────
tmux send-keys -t "${PANE_RTABMAP}" \
    "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc '${ROS_SRC} && ros2 launch rtabmap_bringup rtabmap_slam.launch.py ${RTABMAP_ARGS[*]}'" Enter

# ── Pane: Bag playback (after RTAB-Map is ready) ─────────────────────
# Wait for the /rtabmap node itself — in this launch the map topics are at
# root (/mapData, /mapGraph), so /rtabmap/info doesn't exist. Node presence
# is the reliable readiness signal.
#
# If the bag holds compressed fisheye topics, spawn decompressor republishers
# in the background first so downstream consumers see raw /t265/fisheye*.
DECOMPRESS_CMD=""
if [[ "${FISHEYE_DECOMPRESS}" == true ]]; then
    DECOMPRESS_CMD="echo Starting fisheye decompressors... && ros2 run image_transport republish --ros-args -p in_transport:=compressed -p out_transport:=raw -r in/compressed:=/t265/fisheye1/image_raw/compressed -r out:=/t265/fisheye1/image_raw >/tmp/decompress_fisheye1.log 2>&1 & ros2 run image_transport republish --ros-args -p in_transport:=compressed -p out_transport:=raw -r in/compressed:=/t265/fisheye2/image_raw/compressed -r out:=/t265/fisheye2/image_raw >/tmp/decompress_fisheye2.log 2>&1 & sleep 3 &&"
fi
tmux send-keys -t "${PANE_BAG}" \
    "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc '${ROS_SRC} && echo Waiting for RTAB-Map to be ready... && timeout 60 bash -lc \"until ros2 node list 2>/dev/null | grep -qx /rtabmap; do sleep 1; done\" && sleep 3 && ${DECOMPRESS_CMD} echo Starting bag playback... && ros2 bag play ${PLAY_ARGS[*]}'" Enter

# ── Pane: Inspection shell ───────────────────────────────────────────
tmux send-keys -t "${PANE_INSPECT}" \
    "source ${SCRIPT_DIR}/lib/dc.sh && xdcomp exec ackermann_slam bash" Enter

tmux select-pane -t "${PANE_RTABMAP}"

if [[ "${ATTACH}" == true && -t 1 ]]; then
    tmux attach-session -t "${SESSION}"
else
    echo ""
    echo "Session '${SESSION}' started detached. Attach with:"
    echo "  tmux attach -t ${SESSION}"
    echo "Stop with:"
    echo "  ./scripts/stop_all.sh --session=${SESSION}"
fi
