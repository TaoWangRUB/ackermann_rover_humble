#!/usr/bin/env bash
# Replay a recorded rosbag2 for offline RTAB-Map tuning (Phase A workflow).
#
# This script is the companion to scripts/record_bag.sh. It only handles the
# bag-playback side — launch RTAB-Map separately in another pane with:
#   ros2 launch rtabmap_bringup rtabmap_slam.launch.py \
#     use_sim_time:=true delete_db_on_start:=true \
#     use_t265_odom:=true            # (or --cuvslam/--vins/--rgbd — match record)
#
# Usage:
#   ./scripts/replay_bag.sh                                # play latest bag at 1.0x
#   ./scripts/replay_bag.sh --name=run_20260420_2155       # specific bag
#   ./scripts/replay_bag.sh --path=/workspace/bags/run_X   # specific path
#   ./scripts/replay_bag.sh --rate=0.5                     # slower (for debugging sync)
#   ./scripts/replay_bag.sh --loop                         # loop the bag
#   ./scripts/replay_bag.sh --start=15                     # skip first 15s
#   ./scripts/replay_bag.sh --list                         # list bags under /workspace/bags
#
# Always passes --clock so use_sim_time nodes (RTAB-Map etc.) sync to bag time.
set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"

# ── args ───────────────────────────────────────────────────────────────
NAME=""
PATH_OVERRIDE=""
RATE="1.0"
LOOP=false
START_OFFSET=""
LIST=false

for arg in "$@"; do
    case "${arg}" in
        --name=*)   NAME="${arg#--name=}" ;;
        --path=*)   PATH_OVERRIDE="${arg#--path=}" ;;
        --rate=*)   RATE="${arg#--rate=}" ;;
        --loop)     LOOP=true ;;
        --start=*)  START_OFFSET="${arg#--start=}" ;;
        --list)     LIST=true ;;
        *) echo "Unknown arg: ${arg}" >&2; exit 2 ;;
    esac
done

# ── list bags ──────────────────────────────────────────────────────────
if [[ "${LIST}" == true ]]; then
    echo "Available bags under ${PROJECT_DIR}/bags:"
    ls -lt "${PROJECT_DIR}/bags" 2>/dev/null | awk 'NR>1 {printf "  %-30s %s %s %s\n", $NF, $6, $7, $8}' \
        || echo "  (no bags directory)"
    exit 0
fi

# ── resolve bag path ───────────────────────────────────────────────────
if [[ -n "${PATH_OVERRIDE}" ]]; then
    BAG_PATH="${PATH_OVERRIDE}"
elif [[ -n "${NAME}" ]]; then
    BAG_PATH="/workspace/bags/${NAME}"
else
    # Default: newest bag under <repo>/bags/
    if [[ ! -d "${PROJECT_DIR}/bags" ]]; then
        echo "No ${PROJECT_DIR}/bags directory found. Record a bag first with scripts/record_bag.sh" >&2
        exit 1
    fi
    LATEST_HOST="$(ls -1dt "${PROJECT_DIR}/bags"/run_* 2>/dev/null | head -n1 || true)"
    if [[ -z "${LATEST_HOST}" ]]; then
        echo "No bags matching 'run_*' under ${PROJECT_DIR}/bags. Use --path for custom names." >&2
        exit 1
    fi
    BAG_PATH="/workspace/bags/$(basename "${LATEST_HOST}")"
fi

# ── build play command ────────────────────────────────────────────────
PLAY_ARGS=(
    "${BAG_PATH}"
    --clock                # mandatory — drives use_sim_time
    --rate "${RATE}"
)
[[ "${LOOP}" == true ]]      && PLAY_ARGS+=(--loop)
[[ -n "${START_OFFSET}" ]]   && PLAY_ARGS+=(--start-offset "${START_OFFSET}")

echo "─────────────────────────────────────────────────────────────"
echo "  rosbag2 play"
echo "  bag:    ${BAG_PATH}"
echo "  rate:   ${RATE}"
echo "  loop:   ${LOOP}"
[[ -n "${START_OFFSET}" ]] && echo "  start:  +${START_OFFSET}s"
echo ""
echo "  Remember: launch RTAB-Map separately with use_sim_time:=true"
echo "─────────────────────────────────────────────────────────────"

xdcomp exec ackermann_slam bash -c "
  source /opt/ros/jazzy/setup.bash && \
  source /workspace/install/setup.bash && \
  ros2 bag play ${PLAY_ARGS[*]}
"
