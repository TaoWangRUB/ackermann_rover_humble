#!/usr/bin/env bash
# Start a tmux session for system monitor + control center verification.
#
# Modes:
#   --auto (default) Auto-detect hardware — use real cameras/PX4 when found,
#                    mock when absent. Reports HW/MOCK status to CC dashboard.
#   --hw             Real hardware — no mock publishers, uses publisher.yaml
#   --mock           x86 dev — starts mock camera/PX4 publishers, uses publisher.localhost.yaml
#
# Layout (3×2 grid):
#   ┌──────────────────────────┬──────────────────────────┐
#   │  rover_monitor launch    │  cam topic / mock cam    │
#   ├──────────────────────────┼──────────────────────────┤
#   │  Health topic echo       │  px4 topic / mock px4    │
#   ├──────────────────────────┼──────────────────────────┤
#   │  Control Center stack    │  MQTT / PX4 topic        │
#   └──────────────────────────┴──────────────────────────┘
#
# The --with-telemetry flag enables MQTT telemetry publishing from rover_monitor
# to the Control Center stack. Without it, rover_monitor runs ROS-only and the
# bottom-left pane shows a disabled message.
#
# Usage:
#   ./scripts/start_system_monitor_session.sh                        # auto mode
#   ./scripts/start_system_monitor_session.sh --hw --with-telemetry
#   ./scripts/start_system_monitor_session.sh --mock --with-telemetry
#   ./scripts/start_system_monitor_session.sh --session=montest
#   ./scripts/start_system_monitor_session.sh --no-attach
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"
source "${SCRIPT_DIR}/lib/detect_hw.sh"

SESSION="sysmon"
ENABLE_TELEMETRY=false
ATTACH=true
MODE="auto"
DEPTH_CAMERA="d435i"

for arg in "$@"; do
    case "${arg}" in
        --auto)            MODE="auto" ;;
        --hw)              MODE="hw" ;;
        --mock)            MODE="mock" ;;
        --with-telemetry)  ENABLE_TELEMETRY=true ;;
        --no-attach)       ATTACH=false ;;
        --session=*)       SESSION="${arg#--session=}" ;;
        --depth-camera=*)  DEPTH_CAMERA="${arg#--depth-camera=}" ;;
        -h|--help)
            sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
            exit 0 ;;
    esac
done

# ── Auto-detection ────────────────────────────────────────────────────
# In auto mode, detect connected hardware and decide per-component.
# Sets MOCK_CAM, MOCK_PX4 to "true"/"false".
MOCK_CAM="false"
MOCK_PX4="false"

if [[ "${MODE}" == "auto" ]]; then
    echo "Auto-detecting hardware..."
    detect_realsense_cameras
    detect_px4_dds
    print_hw_detection

    # Decide depth camera: prefer D435i, fall back to L515
    if [[ "${HW_HAS_D435I}" == "true" ]]; then
        DEPTH_CAMERA="d435i"
    elif [[ "${HW_HAS_L515}" == "true" ]]; then
        DEPTH_CAMERA="l515"
    else
        MOCK_CAM="true"
    fi

    if [[ "${HW_HAS_PX4_DDS}" != "true" ]]; then
        MOCK_PX4="true"
    fi

    echo "Auto-detect result:"
    echo "  Camera: $([ "${MOCK_CAM}" == "true" ] && echo "MOCK" || echo "HW (${DEPTH_CAMERA})")"
    echo "  T265:   $([ "${HW_HAS_T265}" == "true" ] && echo "HW" || echo "not detected")"
    echo "  PX4:    $([ "${MOCK_PX4}" == "true" ] && echo "MOCK" || echo "HW (DDS)")"
    echo ""
elif [[ "${MODE}" == "mock" ]]; then
    MOCK_CAM="true"
    MOCK_PX4="true"
fi

# ── Build launch arguments based on mode ──────────────────────────────
TELEMETRY_ARG="enable_telemetry:=false"
PUBLISHER_CONFIG_ARG=""

if [[ "${ENABLE_TELEMETRY}" == true ]]; then
    TELEMETRY_ARG="enable_telemetry:=true"
    if [[ "${MODE}" == "mock" || "${MODE}" == "auto" ]]; then
        PUBLISHER_CONFIG_ARG="publisher_config_file:=/workspace/src/rover_monitor/config/publisher.localhost.yaml"
    fi
    # hw mode: uses default publisher.yaml (broker_host from config, e.g. 192.168.1.100)
fi

# ── ROS source preamble (reused in pane commands) ─────────────────────
ROS_SRC="source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash"

# ── Ensure Docker container is running ────────────────────────────────
if ! dcomp ps --services --filter status=running 2>/dev/null \
        | grep -q '^ackermann_slam$'; then
    echo "Container not running — starting ackermann_slam..."
    dcomp up -d ackermann_slam
fi

# ── Kill old ROS nodes before starting new ones ─────────────────────
# NOTE: With pid:host the container sees host PIDs, so exclude
# docker/docker-compose processes to avoid killing ourselves.
echo "Stopping old ROS nodes..."
dcomp exec -T ackermann_slam bash -c \
    'pgrep -f "ros2|python3.*/workspace|component_container|publish_mock" 2>/dev/null \
     | while read pid; do
         cmdline=$(tr "\0" " " < /proc/$pid/cmdline 2>/dev/null)
         case "$cmdline" in *docker*) continue ;; esac
         kill -9 "$pid" 2>/dev/null
       done; sleep 1' 2>/dev/null || true
echo "Clean."

tmux kill-session -t "${SESSION}" 2>/dev/null || true

tmux new-session -d -s "${SESSION}" -n "monitor"
tmux set-option -t "${SESSION}" -g mouse on
tmux set-option -t "${SESSION}" -g pane-base-index 0

# Create panes using pane IDs so the layout stays stable.
PANE_LAUNCH="$(tmux display-message -p -t "${SESSION}:monitor.0" "#{pane_id}")"
PANE_CAM="$(tmux split-window -h -P -F "#{pane_id}" -t "${PANE_LAUNCH}")"
PANE_PX4="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_CAM}")"
PANE_HEALTH="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_LAUNCH}")"
PANE_MQTT="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_PX4}" -l 10)"
PANE_CC="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_HEALTH}" -l 8)"

# ── Pane 0: rover_monitor launch ──────────────────────────────────────
tmux send-keys -t "${PANE_LAUNCH}" \
    "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc '${ROS_SRC} && ros2 launch rover_monitor monitor.launch.py use_sim_time:=false depth_camera:=${DEPTH_CAMERA} ${TELEMETRY_ARG} ${PUBLISHER_CONFIG_ARG}'" Enter

# ── Pane 1: Camera ────────────────────────────────────────────────────
if [[ "${MOCK_CAM}" == "true" ]]; then
    tmux send-keys -t "${PANE_CAM}" \
        "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc '${ROS_SRC} && python3 /workspace/scripts/publish_mock_camera.py'" Enter
else
    tmux send-keys -t "${PANE_CAM}" \
        "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc '${ROS_SRC} && sleep 3 && ros2 topic echo /monitor/cam'" Enter
fi

# ── Pane 2: Health topic echo ─────────────────────────────────────────
tmux send-keys -t "${PANE_HEALTH}" \
    "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc '${ROS_SRC} && sleep 3 && ros2 topic echo /monitor/health'" Enter

# ── Pane 3: PX4 ───────────────────────────────────────────────────────
if [[ "${MOCK_PX4}" == "true" ]]; then
    tmux send-keys -t "${PANE_PX4}" \
        "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc '${ROS_SRC} && python3 /workspace/scripts/publish_mock_px4.py'" Enter
else
    tmux send-keys -t "${PANE_PX4}" \
        "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc '${ROS_SRC} && sleep 3 && ros2 topic echo /monitor/px4'" Enter
fi

# ── Pane 4: MQTT decoded / PX4 topic ─────────────────────────────────
if [[ "${ENABLE_TELEMETRY}" == true ]]; then
    tmux send-keys -t "${PANE_MQTT}" \
        "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc 'if ! command -v mosquitto_sub >/dev/null 2>&1; then echo \"mosquitto_sub is not installed in the container\"; exit 1; fi; if ! python3 -c \"import google.protobuf\" >/dev/null 2>&1; then echo \"python3-protobuf is not installed in the container\"; exit 1; fi; cd /tmp && protoc --python_out=/tmp -I /workspace/src/rover_monitor/proto /workspace/src/rover_monitor/proto/rover_health.proto >/dev/null 2>&1 || true; mosquitto_sub -h localhost -t \"rover/health/#\" -F \"%x\" | python3 /workspace/scripts/decode_rover_health_mqtt.py --hex'" Enter
else
    tmux send-keys -t "${PANE_MQTT}" \
        "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc '${ROS_SRC} && sleep 4 && ros2 topic echo /monitor/px4'" Enter
fi

# ── Pane 5: Control Center stack ──────────────────────────────────────
if [[ "${ENABLE_TELEMETRY}" == true ]]; then
    tmux send-keys -t "${PANE_CC}" \
        "echo '--- Starting Control Center stack ---' && docker compose -f ${PROJECT_DIR}/control_center/docker-compose.yaml up -d && sleep 3 && python3 ${PROJECT_DIR}/control_center/scripts/setup_influxdb_dashboard.py && echo '' && echo '  Dashboard:  http://localhost:8080' && echo '  InfluxDB:   http://localhost:8086  (rover / rover-password)' && echo '  Test:       ./scripts/test_control_center.sh' && echo '' && echo '--- Control Center ready ---'" Enter
else
    tmux send-keys -t "${PANE_CC}" \
        "echo 'Control Center disabled (use --with-telemetry to enable)'" Enter
fi

tmux select-pane -t "${PANE_LAUNCH}"

# ── Report HW mode to Control Center ─────────────────────────────────
if [[ "${ENABLE_TELEMETRY}" == true ]]; then
    # Give CC a moment to process the new session, then report hw/mock state.
    (sleep 5 && report_hw_mode_to_cc) &
fi

if [[ "${ATTACH}" == true && -t 1 ]]; then
    tmux attach-session -t "${SESSION}"
else
    echo "Tmux session '${SESSION}' created (mode: ${MODE})."
    echo "Attach with: tmux attach-session -t ${SESSION}"
fi
