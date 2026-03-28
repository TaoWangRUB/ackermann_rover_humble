#!/usr/bin/env bash
# Start a tmux session for system monitor + control center x86_64 verification.
#
# Layout:
#   ┌──────────────────────────┬──────────────────────────┐
#   │                          │  Mock Camera             │
#   │  rover_monitor launch    ├──────────────────────────┤
#   │                          │  Mock PX4 pubs           │
#   ├──────────────────────────┼──────────────────────────┤
#   │  Health topic            │  MQTT / PX4 topic        │
#   ├──────────────────────────┴──────────────────────────┤
#   │  Control Center stack (docker compose + dashboard)  │
#   └─────────────────────────────────────────────────────┘
#
# The --with-telemetry flag enables MQTT telemetry publishing from rover_monitor
# to the Control Center stack. Without it, rover_monitor runs ROS-only and the
# bottom pane shows a disabled message.
#
# Usage:
#   ./scripts/start_system_monitor_session.sh
#   ./scripts/start_system_monitor_session.sh --with-telemetry
#   ./scripts/start_system_monitor_session.sh --session=montest
#   ./scripts/start_system_monitor_session.sh --no-attach
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"

SESSION="sysmon"
ENABLE_TELEMETRY=false
ATTACH=true
PUBLISHER_CONFIG_ARG=""

for arg in "$@"; do
    case "${arg}" in
        --with-telemetry) ENABLE_TELEMETRY=true ;;
        --no-attach) ATTACH=false ;;
        --session=*) SESSION="${arg#--session=}" ;;
        -h|--help)
            sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
            exit 0 ;;
    esac
done

TELEMETRY_ARG="enable_telemetry:=false"
if [[ "${ENABLE_TELEMETRY}" == true ]]; then
    TELEMETRY_ARG="enable_telemetry:=true"
    PUBLISHER_CONFIG_ARG="publisher_config_file:=/workspace/src/rover_monitor/config/publisher.localhost.yaml"
fi

tmux kill-session -t "${SESSION}" 2>/dev/null || true

tmux new-session -d -s "${SESSION}" -n "monitor"
tmux set-option -t "${SESSION}" -g mouse on
tmux set-option -t "${SESSION}" -g pane-base-index 0

# Create panes using pane IDs so the layout stays stable.
PANE_LAUNCH="$(tmux display-message -p -t "${SESSION}:monitor.0" "#{pane_id}")"
PANE_CAMERA="$(tmux split-window -h -P -F "#{pane_id}" -t "${PANE_LAUNCH}")"
PANE_PX4_MOCK="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_CAMERA}")"
PANE_HEALTH="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_LAUNCH}")"
PANE_MQTT="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_PX4_MOCK}" -l 10)"
PANE_CC="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_HEALTH}" -l 8)"

# ── Pane 0: rover_monitor launch ──────────────────────────────────────
tmux send-keys -t "${PANE_LAUNCH}" \
    "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && ros2 launch rover_monitor monitor.launch.py use_sim_time:=false ${TELEMETRY_ARG} ${PUBLISHER_CONFIG_ARG}'" Enter

# ── Pane 1: Mock camera ──────────────────────────────────────────────
tmux send-keys -t "${PANE_CAMERA}" \
    "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && python3 /workspace/scripts/publish_mock_camera.py'" Enter

# ── Pane 2: Health topic echo ────────────────────────────────────────
tmux send-keys -t "${PANE_HEALTH}" \
    "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && sleep 3 && ros2 topic echo /monitor/health'" Enter

# ── Pane 3: Mock PX4 ─────────────────────────────────────────────────
tmux send-keys -t "${PANE_PX4_MOCK}" \
    "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && python3 /workspace/scripts/publish_mock_px4.py'" Enter

# ── Pane 4: MQTT decoded / PX4 topic ─────────────────────────────────
if [[ "${ENABLE_TELEMETRY}" == true ]]; then
    tmux send-keys -t "${PANE_MQTT}" \
        "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc 'if ! command -v mosquitto_sub >/dev/null 2>&1; then echo \"mosquitto_sub is not installed in the container\"; exit 1; fi; if ! python3 -c \"import google.protobuf\" >/dev/null 2>&1; then echo \"python3-protobuf is not installed in the container\"; exit 1; fi; cd /tmp && protoc --python_out=/tmp -I /workspace/src/rover_monitor/proto /workspace/src/rover_monitor/proto/rover_health.proto >/dev/null 2>&1 || true; while true; do if ! mosquitto_sub -h localhost -t \"rover/health/#\" -C 1 | python3 /workspace/scripts/decode_rover_health_mqtt.py; then echo \"waiting for local mosquitto...\"; sleep 2; fi; done'" Enter
else
    tmux send-keys -t "${PANE_MQTT}" \
        "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && sleep 4 && ros2 topic echo /monitor/px4'" Enter
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

if [[ "${ATTACH}" == true && -t 1 ]]; then
    tmux attach-session -t "${SESSION}"
else
    echo "Tmux session '${SESSION}' created."
    echo "Attach with: tmux attach-session -t ${SESSION}"
fi
