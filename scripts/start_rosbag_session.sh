#!/usr/bin/env bash
# Start a tmux session for offline RTAB-Map tuning from a recorded bag
# (Phase A workflow). Companion to scripts/record_bag.sh — same flag set
# so the same invocation that produced the bag plays it back.
#
# Launches (in dependency order):
#   1. RTAB-Map SLAM (with rtabmap_viz)     — immediate, use_sim_time:=true
#   2. Bag playback                          — after 5s (RTAB-Map ready)
#   3. Inspection shell                      — immediate (for ros2 topic, rtabmap-databaseViewer)
#
# Layout (3 panes):
#   ┌─────────────────────────┬──────────────────────┐
#   │                         │                      │
#   │  RTAB-Map + viz         │  Inspection shell    │
#   │  (use_sim_time:=true)   │  (container bash)    │
#   ├─────────────────────────┤                      │
#   │  Bag playback           │                      │
#   │  (ros2 bag play --clock)│                      │
#   └─────────────────────────┴──────────────────────┘
#
# Usage:
#   ./scripts/start_rosbag_session.sh                              # latest bag, T265 odom
#   ./scripts/start_rosbag_session.sh --name=run_20260420_2155     # specific bag
#   ./scripts/start_rosbag_session.sh --cuvslam-odom               # match record flags
#   ./scripts/start_rosbag_session.sh --rate=0.5 --loop
#   ./scripts/start_rosbag_session.sh --keep-db                    # keep existing DB
#   ./scripts/start_rosbag_session.sh --no-viz                     # headless
#   ./scripts/start_rosbag_session.sh --no-attach
#
# To stop everything:
#   ./scripts/stop_all.sh --session=rosbag
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"

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

for arg in "$@"; do
    case "${arg}" in
        --name=*)          BAG_NAME="${arg#--name=}" ;;
        --path=*)          BAG_PATH_OVERRIDE="${arg#--path=}" ;;
        --rate=*)          RATE="${arg#--rate=}" ;;
        --loop)            LOOP=true ;;
        --start=*)         START_OFFSET="${arg#--start=}" ;;
        --depth-camera=*)  DEPTH_CAMERA="${arg#--depth-camera=}" ;;
        --t265-odom)       T265_ODOM=true ;;
        --cuvslam-odom)    CUVSLAM_ODOM=true ;;
        --rgbd-odom)       RGBD_ODOM=true ;;
        --vins-odom)       VINS_ODOM=true ;;
        --no-viz)          RTABMAP_VIZ=false ;;
        --keep-db)         DELETE_DB=false ;;
        --delete-db)       DELETE_DB=true ;;
        --no-attach)       ATTACH=false ;;
        --session=*)       SESSION="${arg#--session=}" ;;
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

# ── Resolve bag path ──────────────────────────────────────────────────
if [[ -n "${BAG_PATH_OVERRIDE}" ]]; then
    BAG_PATH="${BAG_PATH_OVERRIDE}"
elif [[ -n "${BAG_NAME}" ]]; then
    BAG_PATH="/workspace/bags/${BAG_NAME}"
else
    LATEST_HOST="$(ls -1dt "${PROJECT_DIR}/bags"/run_* 2>/dev/null | head -n1 || true)"
    if [[ -z "${LATEST_HOST}" ]]; then
        echo "No bags under ${PROJECT_DIR}/bags. Record one first with scripts/record_bag.sh." >&2
        exit 1
    fi
    BAG_PATH="/workspace/bags/$(basename "${LATEST_HOST}")"
fi

# ── Build RTAB-Map launch args ───────────────────────────────────────
RTABMAP_ARGS=(
    "use_sim_time:=true"
    "depth_camera:=${DEPTH_CAMERA}"
    "rtabmap_viz:=${RTABMAP_VIZ}"
    "delete_db_on_start:=${DELETE_DB}"
)
if [[ "${CUVSLAM_ODOM}" == true ]]; then
    RTABMAP_ARGS+=("use_cuvslam_odom:=true")
    ODOM_LABEL="cuvslam_odom"
elif [[ "${RGBD_ODOM}" == true ]]; then
    RTABMAP_ARGS+=("use_rgbd_odom:=true")
    ODOM_LABEL="cuvslam_rgbd_odom"
elif [[ "${VINS_ODOM}" == true ]]; then
    RTABMAP_ARGS+=("use_vins_odom:=true")
    ODOM_LABEL="vins_odom"
else
    RTABMAP_ARGS+=("use_t265_odom:=true")
    ODOM_LABEL="t265"
fi

# ── Build bag play command ────────────────────────────────────────────
PLAY_ARGS=("${BAG_PATH}" --clock --rate "${RATE}")
[[ "${LOOP}" == true ]]    && PLAY_ARGS+=(--loop)
[[ -n "${START_OFFSET}" ]] && PLAY_ARGS+=(--start-offset "${START_OFFSET}")

ROS_SRC="source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash"

# ── Ensure container is running ──────────────────────────────────────
if ! dcomp ps --services --filter status=running 2>/dev/null \
        | grep -q '^ackermann_slam$'; then
    echo "Container not running — starting ackermann_slam..."
    dcomp up -d ackermann_slam
fi

# ── Kill old session ─────────────────────────────────────────────────
"${SCRIPT_DIR}/stop_all.sh" --session="${SESSION}" 2>/dev/null || true
sleep 2

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
# Wait for /rtabmap/info so we know subscribers are attached before bag starts.
tmux send-keys -t "${PANE_BAG}" \
    "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc '${ROS_SRC} && echo Waiting for RTAB-Map to be ready... && timeout 60 bash -lc \"until ros2 topic list 2>/dev/null | grep -qx /rtabmap/info; do sleep 1; done\" && sleep 3 && echo Starting bag playback... && ros2 bag play ${PLAY_ARGS[*]}'" Enter

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
