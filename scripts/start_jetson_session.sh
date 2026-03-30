#!/usr/bin/env bash
# Start a tmux session with all Jetson rover components in one place.
#
# Launches (in dependency order):
#   1. Micro XRCE-DDS Agent (serial)           — immediate
#   2. System Monitor (rover_monitor)           — immediate (independent)
#   3. ROS 2 Nodes (cameras + RTAB-Map SLAM)   — after 3s (XRCE ready)
#   4. PX4 Bringup (custom mode + VO bridge)   — after 8s (ROS nodes up)
#   5. Mode Activation (arm + custom mode)      — after 28s (PX4 registered)
#
# Layout without --with-telemetry (5 panes):
#   ┌─────────────────────────┬──────────────────────┐
#   │                         │  XRCE Agent          │
#   │  ROS 2 Nodes            ├──────────────────────┤
#   │  (cameras + SLAM)       │  System Monitor      │
#   │                         │  (rover_monitor)     │
#   ├─────────────────────────┼──────────────────────┤
#   │  PX4 Bringup            │  Mode Activation     │
#   │  (bridge + VO)          │  → container shell   │
#   └─────────────────────────┴──────────────────────┘
#
# Layout with --with-telemetry (6 panes):
#   ┌─────────────────────────┬──────────────────────┐
#   │                         │  XRCE Agent          │
#   │  ROS 2 Nodes            ├──────────────────────┤
#   │  (cameras + SLAM)       │  System Monitor      │
#   │                         │  (w/ telemetry)      │
#   ├─────────────────────────┼──────────────────────┤
#   │  PX4 Bringup            │  Mode Activation     │
#   │  (bridge + VO)          ├──────────────────────┤
#   │                         │  MQTT verify         │
#   └─────────────────────────┴──────────────────────┘
#
# Usage:
#   ./scripts/start_jetson_session.sh
#   ./scripts/start_jetson_session.sh --with-telemetry
#   ./scripts/start_jetson_session.sh --nav2
#   ./scripts/start_jetson_session.sh --nav2 --with-telemetry
#   ./scripts/start_jetson_session.sh --mode-id=24
#   ./scripts/start_jetson_session.sh --mode-type=speed_steering
#   ./scripts/start_jetson_session.sh --reversible-drive
#   ./scripts/start_jetson_session.sh --no-activate
#   ./scripts/start_jetson_session.sh --no-attach
#   ./scripts/start_jetson_session.sh --session=mytest
#
# To stop everything:
#   ./scripts/stop_all.sh --session=jetson
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"

SESSION="jetson"
ENABLE_TELEMETRY=false
ENABLE_NAV2=false
ENABLE_T265=false
ACTIVATE=true
MODE_ID="23"
ATTACH=true
DEPTH_CAMERA="d435i"
BROKER_HOST=""
PUBLISHER_CONFIG_FILE="/workspace/src/rover_monitor/config/publisher.yaml"
PX4_MODE_TYPE="manual"
REVERSIBLE_DRIVE=false

for arg in "$@"; do
    case "${arg}" in
        --with-telemetry)  ENABLE_TELEMETRY=true ;;
        --nav2)            ENABLE_NAV2=true ;;
        --t265)            ENABLE_T265=true ;;
        --no-activate)     ACTIVATE=false ;;
        --mode-id=*)       MODE_ID="${arg#--mode-id=}" ;;
        --mode-type=*)     PX4_MODE_TYPE="${arg#--mode-type=}" ;;
        --reversible-drive) REVERSIBLE_DRIVE=true ;;
        --depth-camera=*)  DEPTH_CAMERA="${arg#--depth-camera=}" ;;
        --broker-host=*)   BROKER_HOST="${arg#--broker-host=}" ;;
        --publisher-config=*) PUBLISHER_CONFIG_FILE="${arg#--publisher-config=}" ;;
        --no-attach)       ATTACH=false ;;
        --session=*)       SESSION="${arg#--session=}" ;;
        -h|--help)
            sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
            exit 0 ;;
    esac
done

# ── Build sub-command arguments ───────────────────────────────────────
ROS2_ARGS="--hw --rtabmap --no-rviz --depth-camera=${DEPTH_CAMERA}"
if [[ "${ENABLE_T265}" == true ]]; then
    ROS2_ARGS+=" --t265"
fi
if [[ "${ENABLE_NAV2}" == true ]]; then
    ROS2_ARGS+=" --nav2"
fi

ROS_SRC="source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash"

TELEMETRY_ARG="enable_telemetry:=false"
PUBLISHER_CONFIG_ARG=""
BROKER_HOST_ARG=""
if [[ "${ENABLE_TELEMETRY}" == true ]]; then
    TELEMETRY_ARG="enable_telemetry:=true"
    PUBLISHER_CONFIG_ARG="publisher_config_file:=${PUBLISHER_CONFIG_FILE}"
fi

HOST_PUBLISHER_CONFIG_FILE="${PUBLISHER_CONFIG_FILE}"
if [[ "${HOST_PUBLISHER_CONFIG_FILE}" == /workspace/* ]]; then
    HOST_PUBLISHER_CONFIG_FILE="${PROJECT_DIR}/${HOST_PUBLISHER_CONFIG_FILE#/workspace/}"
fi

if [[ -z "${BROKER_HOST}" ]]; then
    if [[ -f "${HOST_PUBLISHER_CONFIG_FILE}" ]]; then
        BROKER_HOST="$(
            awk -F'"' '/broker_host:/ { print $2; exit }' \
            "${HOST_PUBLISHER_CONFIG_FILE}"
        )"
    fi
fi

BROKER_HOST="${BROKER_HOST:-localhost}"
if [[ "${ENABLE_TELEMETRY}" == true ]]; then
    BROKER_HOST_ARG="broker_host:=${BROKER_HOST}"
fi

PX4_BRINGUP_ARGS=(--bridge --vo-bridge --mode-type "${PX4_MODE_TYPE}")
if [[ "${REVERSIBLE_DRIVE}" == true ]]; then
    PX4_BRINGUP_ARGS+=(--reversible-drive)
fi

# ── Create tmux session ──────────────────────────────────────────────
tmux kill-session -t "${SESSION}" 2>/dev/null || true

tmux new-session -d -s "${SESSION}" -n "jetson"
tmux set-option -t "${SESSION}" -g mouse on
tmux set-option -t "${SESSION}" -g pane-base-index 0

# Create panes using pane IDs so the layout stays stable.
PANE_ROS2="$(tmux display-message -p -t "${SESSION}:jetson.0" "#{pane_id}")"
PANE_XRCE="$(tmux split-window -h -P -F "#{pane_id}" -t "${PANE_ROS2}")"
PANE_MONITOR="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_XRCE}")"
PANE_PX4="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_ROS2}" -l 15)"
PANE_ACTIVATE="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_MONITOR}")"

if [[ "${ENABLE_TELEMETRY}" == true ]]; then
    PANE_MQTT="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_ACTIVATE}" -l 8)"
fi

# ── Pane: XRCE Agent (immediate) ─────────────────────────────────────
tmux send-keys -t "${PANE_XRCE}" \
    "${SCRIPT_DIR}/start_microxrce_agent.sh --serial" Enter

# ── Pane: System Monitor (immediate, independent) ────────────────────
tmux send-keys -t "${PANE_MONITOR}" \
    "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc '${ROS_SRC} && ros2 launch rover_monitor monitor.launch.py use_sim_time:=false depth_camera:=${DEPTH_CAMERA} ${TELEMETRY_ARG} ${PUBLISHER_CONFIG_ARG} ${BROKER_HOST_ARG}'" Enter

# ── Pane: ROS 2 Nodes (wait for XRCE) ────────────────────────────────
tmux send-keys -t "${PANE_ROS2}" \
    "sleep 3 && ${SCRIPT_DIR}/start_ros2_nodes.sh ${ROS2_ARGS}" Enter

# ── Pane: PX4 Bringup (wait for ROS nodes) ───────────────────────────
tmux send-keys -t "${PANE_PX4}" \
    "sleep 8 && ${SCRIPT_DIR}/start_px4_bringup_vo.sh ${PX4_BRINGUP_ARGS[*]}" Enter

# ── Pane: Mode Activation (wait for PX4 registration) ────────────────
if [[ "${ACTIVATE}" == true ]]; then
    tmux send-keys -t "${PANE_ACTIVATE}" \
        "echo 'Waiting 28s for PX4 mode registration...' && sleep 28 && ${SCRIPT_DIR}/activate_rover_manual.sh ${MODE_ID}; source ${SCRIPT_DIR}/lib/dc.sh && xdcomp exec ackermann_slam bash" Enter
else
    tmux send-keys -t "${PANE_ACTIVATE}" \
        "source ${SCRIPT_DIR}/lib/dc.sh && xdcomp exec ackermann_slam bash" Enter
fi

# ── Pane: MQTT verify (only with telemetry) ───────────────────────────
if [[ "${ENABLE_TELEMETRY}" == true ]]; then
    tmux send-keys -t "${PANE_MQTT}" \
        "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc 'if ! command -v mosquitto_sub >/dev/null 2>&1; then echo \"mosquitto_sub not installed\"; exit 1; fi; if ! python3 -c \"import google.protobuf\" >/dev/null 2>&1; then echo \"protobuf not installed\"; exit 1; fi; cd /tmp && protoc --python_out=/tmp -I /workspace/src/rover_monitor/proto /workspace/src/rover_monitor/proto/rover_health.proto >/dev/null 2>&1 || true; echo \"Subscribing to MQTT broker ${BROKER_HOST}...\"; mosquitto_sub -h ${BROKER_HOST} -t \"rover/health/#\" -F \"%x\" | python3 /workspace/scripts/decode_rover_health_mqtt.py --hex'" Enter
fi

tmux select-pane -t "${PANE_ROS2}"

if [[ "${ATTACH}" == true && -t 1 ]]; then
    tmux attach-session -t "${SESSION}"
else
    echo "Tmux session '${SESSION}' created."
    echo "Attach with: tmux attach-session -t ${SESSION}"
fi
