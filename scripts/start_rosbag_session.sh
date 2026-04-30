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
#   ./scripts/start_rosbag_session.sh --name=run_20260428_1340_seg3 --rtabmap-vis-min-inliers=8
#   ./scripts/start_rosbag_session.sh --name=run_20260428_1340_seg3 --depth-camera=d435i --t265-odom --rtabmap-detection-rate=3 --rtabmap-vis-min-inliers=10 --approx-sync-max-interval=0.03
#   ./scripts/start_rosbag_session.sh --name=run_20260428_1340_seg3 --t265-odom --rtabmap-param=Vis/EstimationType:=0 --rtabmap-param=Vis/BundleAdjustment:=1
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
RTABMAP_VIS_MIN_INLIERS="6"
RTABMAP_DETECTION_RATE="0"
RTABMAP_VIS_ESTIMATION_TYPE="1"
RTABMAP_VIS_MAX_DEPTH="4.0"
RTABMAP_KP_DETECTOR_STRATEGY="6"
RTABMAP_VIS_EPIPOLAR_VAR="0.02"
RTABMAP_REG_STRATEGY="2"
RTABMAP_LINEAR_UPDATE="0.2"
RTABMAP_ANGULAR_UPDATE="0.2"
APPROX_SYNC_MAX_INTERVAL="0.1"
ATTACH=true
WAIT_SECONDS="10"
EXTRA_TOPICS=""
COMPRESS_FISHEYE=""   # unset → let record_bag.sh default apply
RTABMAP_PARAM_OVERRIDES=()
RTABMAP_UDEBUG=false
JETSON_PROFILE=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --record)              MODE="record" ;;
        --replay)              MODE="replay" ;;
        --name=*)              BAG_NAME="${1#--name=}" ;;
        --path=*)              BAG_PATH_OVERRIDE="${1#--path=}" ;;
        --rate=*)              RATE="${1#--rate=}" ;;
        --loop)                LOOP=true ;;
        --start=*)             START_OFFSET="${1#--start=}" ;;
        --depth-camera=*)      DEPTH_CAMERA="${1#--depth-camera=}" ;;
        --t265-odom)           T265_ODOM=true ;;
        --cuvslam-odom)        CUVSLAM_ODOM=true ;;
        --rgbd-odom)           RGBD_ODOM=true ;;
        --vins-odom)           VINS_ODOM=true ;;
        --no-viz)              RTABMAP_VIZ=false ;;
        --keep-db)             DELETE_DB=false ;;
        --delete-db)           DELETE_DB=true ;;
        --rtabmap-detection-rate=*) RTABMAP_DETECTION_RATE="${1#--rtabmap-detection-rate=}" ;;
        --rtabmap-vis-min-inliers=*) RTABMAP_VIS_MIN_INLIERS="${1#--rtabmap-vis-min-inliers=}" ;;
        --rtabmap-vis-estimation-type=*) RTABMAP_VIS_ESTIMATION_TYPE="${1#--rtabmap-vis-estimation-type=}" ;;
        --rtabmap-vis-max-depth=*) RTABMAP_VIS_MAX_DEPTH="${1#--rtabmap-vis-max-depth=}" ;;
        --rtabmap-kp-detector-strategy=*) RTABMAP_KP_DETECTOR_STRATEGY="${1#--rtabmap-kp-detector-strategy=}" ;;
        --rtabmap-vis-epipolar-var=*) RTABMAP_VIS_EPIPOLAR_VAR="${1#--rtabmap-vis-epipolar-var=}" ;;
        --rtabmap-reg-strategy=*) RTABMAP_REG_STRATEGY="${1#--rtabmap-reg-strategy=}" ;;
        --rtabmap-linear-update=*) RTABMAP_LINEAR_UPDATE="${1#--rtabmap-linear-update=}" ;;
        --rtabmap-angular-update=*) RTABMAP_ANGULAR_UPDATE="${1#--rtabmap-angular-update=}" ;;
        --approx-sync-max-interval=*) APPROX_SYNC_MAX_INTERVAL="${1#--approx-sync-max-interval=}" ;;
        --rtabmap-param=*)       RTABMAP_PARAM_OVERRIDES+=("${1#--rtabmap-param=}") ;;
        --rtabmap-udebug)        RTABMAP_UDEBUG=true ;;
        --jetson-profile)        JETSON_PROFILE=true ;;
        --no-jetson-profile)     JETSON_PROFILE=false ;;
        --wait-seconds=*)      WAIT_SECONDS="${1#--wait-seconds=}" ;;
        --extra=*)             EXTRA_TOPICS="${1#--extra=}" ;;
        --compress-fisheye)    COMPRESS_FISHEYE=true ;;
        --no-compress-fisheye) COMPRESS_FISHEYE=false ;;
        --no-attach)           ATTACH=false ;;
        --session=*)           SESSION="${1#--session=}" ;;
        -h|--help)
            sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
            exit 0 ;;
        *) echo "Unknown arg: $1" >&2; exit 2 ;;
    esac
    shift
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
    "rtabmap_detection_rate:=${RTABMAP_DETECTION_RATE}"
    "rtabmap_vis_min_inliers:=${RTABMAP_VIS_MIN_INLIERS}"
    "rtabmap_vis_max_features:=1500"
    "rtabmap_vis_estimation_type:=${RTABMAP_VIS_ESTIMATION_TYPE}"
    "rtabmap_vis_max_depth:=${RTABMAP_VIS_MAX_DEPTH}"
    "rtabmap_kp_detector_strategy:=${RTABMAP_KP_DETECTOR_STRATEGY}"
    "rtabmap_vis_epipolar_var:=${RTABMAP_VIS_EPIPOLAR_VAR}"
    "rtabmap_reg_strategy:=${RTABMAP_REG_STRATEGY}"
    "rtabmap_linear_update:=${RTABMAP_LINEAR_UPDATE}"
    "rtabmap_angular_update:=${RTABMAP_ANGULAR_UPDATE}"
    "approx_sync_max_interval:=${APPROX_SYNC_MAX_INTERVAL}"
    "rgb_image_topic:=/${DEPTH_CAMERA}/color/image_raw"
    "rgb_camera_info_topic:=/${DEPTH_CAMERA}/color/camera_info"
    "depth_image_topic:=/${DEPTH_CAMERA}/aligned_depth_to_color/image_raw"
    "depth_camera_info_topic:=/${DEPTH_CAMERA}/aligned_depth_to_color/camera_info"
    "imu_raw_topic:=/${DEPTH_CAMERA}/imu"
    "rtabmap_udebug:=${RTABMAP_UDEBUG}"
    "jetson_profile:=${JETSON_PROFILE}"
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

RTABMAP_PARAM_FILE_HOST=""
RTABMAP_PARAM_FILE_CONTAINER=""
if [[ ${#RTABMAP_PARAM_OVERRIDES[@]} -gt 0 ]]; then
    mkdir -p "${PROJECT_DIR}/.tmp"
    RTABMAP_PARAM_FILE_HOST="$(mktemp "${PROJECT_DIR}/.tmp/rtabmap_params_${SESSION}_XXXXXX.yaml")"
    RTABMAP_PARAM_FILE_CONTAINER="${RTABMAP_PARAM_FILE_HOST/#${PROJECT_DIR}/\/workspace}"
    {
        printf "/rtabmap:\n"
        printf "  ros__parameters:\n"
        for override in "${RTABMAP_PARAM_OVERRIDES[@]}"; do
            if [[ "${override}" != *:=* ]]; then
                echo "Invalid --rtabmap-param value '${override}'. Expected NAME:=VALUE." >&2
                exit 2
            fi
            param_name="${override%%:=*}"
            param_value="${override#*:=}"
            # !!str forces the YAML loader to keep the value as a string regardless
            # of looking like a number/bool. RTAB-Map declares every tuning knob as
            # a string, so we must hand it a string — earlier post-launch
            # `ros2 param load` failed with type-coercion errors on bare 4/true.
            param_value="${param_value//\\/\\\\}"
            param_value="${param_value//\"/\\\"}"
            printf "    %s: !!str \"%s\"\n" "${param_name}" "${param_value}"
        done
    } > "${RTABMAP_PARAM_FILE_HOST}"
    RTABMAP_ARGS+=("extra_rtabmap_params_file:=${RTABMAP_PARAM_FILE_CONTAINER}")
fi

ROS_LAUNCH_CMD=(
    ros2 launch rtabmap_bringup rtabmap_slam.launch.py
    "${RTABMAP_ARGS[@]}"
)
printf -v ROS_LAUNCH_CMD_STR '%q ' "${ROS_LAUNCH_CMD[@]}"

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
echo "  detection:    ${RTABMAP_DETECTION_RATE} Hz"
echo "  min inliers:  ${RTABMAP_VIS_MIN_INLIERS}"
echo "  sync window:  ${APPROX_SYNC_MAX_INTERVAL}s"
if [[ ${#RTABMAP_PARAM_OVERRIDES[@]} -gt 0 ]]; then
    echo "  extra params: ${RTABMAP_PARAM_OVERRIDES[*]}"
fi
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
    "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc '${ROS_SRC} && ${ROS_LAUNCH_CMD_STR}'" Enter

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
# Param overrides are now passed as a launch arg (extra_rtabmap_params_file) so
# they apply at node construction; the post-launch `ros2 param load` path is gone
# because RTAB-Map's params are declared as strings and ros2 param load coerced
# YAML scalars to int/bool, which got rejected.
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
