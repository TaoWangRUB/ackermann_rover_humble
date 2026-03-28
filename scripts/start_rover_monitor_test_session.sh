#!/usr/bin/env bash
# Start a tmux session for rover_monitor x86_64 verification.
#
# Layout:
#   ┌──────────────────────────┬──────────────────────────┐
#   │                          │  Mock Camera            │
#   │  rover_monitor launch    ├──────────────────────────┤
#   │                          │  Mock PX4 pubs          │
#   ├──────────────────────────┼──────────────────────────┤
#   │  Health topic            │  Px4 topic / MQTT       │
#   └──────────────────────────┴──────────────────────────┘
#
# By default this launches rover_monitor without TelemetryPublisher so local
# x86 verification does not depend on a reachable MQTT broker. When
# --with-telemetry is enabled, the session also starts a local Mosquitto broker
# in the launch pane and watches rover/health/# in the lower-right pane.
#
# Usage:
#   ./scripts/start_rover_monitor_test_session.sh
#   ./scripts/start_rover_monitor_test_session.sh --with-telemetry
#   ./scripts/start_rover_monitor_test_session.sh --session=monitorx86
#   ./scripts/start_rover_monitor_test_session.sh --no-attach
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"

SESSION="monitorx86"
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

# Create panes using pane IDs so the layout stays stable at 5 panes.
PANE_LAUNCH="$(tmux display-message -p -t "${SESSION}:monitor.0" "#{pane_id}")"
PANE_CAMERA="$(tmux split-window -h -P -F "#{pane_id}" -t "${PANE_LAUNCH}")"
PANE_PX4_MOCK="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_CAMERA}")"
PANE_HEALTH="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_LAUNCH}")"
PANE_PX4_TOPIC="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_PX4_MOCK}" -l 10)"

tmux send-keys -t "${PANE_LAUNCH}" \
    "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && if [[ \"${ENABLE_TELEMETRY}\" == true ]]; then if ! command -v mosquitto >/dev/null 2>&1; then echo \"mosquitto is not installed in the container\"; exit 1; fi; pkill -x mosquitto || true; mosquitto -d -p 1883; for _ in \$(seq 1 20); do python3 -c \"import socket; s=socket.create_connection((\\\"127.0.0.1\\\", 1883), 0.5); s.close()\" >/dev/null 2>&1 && break; sleep 0.5; done; fi; ros2 launch rover_monitor monitor.launch.py use_sim_time:=false ${TELEMETRY_ARG} ${PUBLISHER_CONFIG_ARG}'" Enter

tmux send-keys -t "${PANE_CAMERA}" \
    "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && python3 /workspace/scripts/publish_mock_camera.py'" Enter

tmux send-keys -t "${PANE_HEALTH}" \
    "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && sleep 3 && ros2 topic echo /monitor/health'" Enter

tmux send-keys -t "${PANE_PX4_MOCK}" \
    "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && python3 /workspace/scripts/publish_mock_px4.py'" Enter

if [[ "${ENABLE_TELEMETRY}" == true ]]; then
    tmux send-keys -t "${PANE_PX4_TOPIC}" \
        "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc 'if ! command -v mosquitto_sub >/dev/null 2>&1; then echo \"mosquitto_sub is not installed in the container\"; exit 1; fi; if ! python3 -c \"import google.protobuf\" >/dev/null 2>&1; then echo \"python3-protobuf is not installed in the container\"; exit 1; fi; cd /tmp && protoc --python_out=/tmp -I /workspace/src/rover_monitor/proto /workspace/src/rover_monitor/proto/rover_health.proto >/dev/null 2>&1 || true; while true; do if ! mosquitto_sub -h localhost -t \"rover/health/#\" -C 1 | python3 /workspace/scripts/decode_rover_health_mqtt.py; then echo \"waiting for local mosquitto...\"; sleep 2; fi; done'" Enter
else
    tmux send-keys -t "${PANE_PX4_TOPIC}" \
        "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && sleep 4 && ros2 topic echo /monitor/px4'" Enter
fi

tmux select-pane -t "${PANE_LAUNCH}"

if [[ "${ATTACH}" == true && -t 1 ]]; then
    tmux attach-session -t "${SESSION}"
else
    echo "Tmux session '${SESSION}' created."
    echo "Attach with: tmux attach-session -t ${SESSION}"
fi
