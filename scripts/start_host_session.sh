#!/usr/bin/env bash
# Start a tmux session on the host (base station / laptop) for monitoring
# the Jetson rover remotely via the Control Center stack.
#
# Launches:
#   1. Control Center stack (Mosquitto + InfluxDB + dashboard via docker compose)
#   2. InfluxDB dashboard provisioning (6-panel "Rover Health Monitor")
#   3. Test suite pane (ready to run test_control_center.sh)
#
# Layout (2×2 grid):
#   ┌─────────────────────────┬──────────────────────┐
#   │  Control Center stack   │  CC logs             │
#   │  (docker compose)       │  (docker compose     │
#   │                         │   logs -f)           │
#   ├─────────────────────────┼──────────────────────┤
#   │  Test / shell           │  MQTT monitor        │
#   │                         │  (mosquitto_sub)     │
#   └─────────────────────────┴──────────────────────┘
#
# Prerequisites:
#   - Docker CE installed (apt, not snap)
#   - Jetson rover running start_jetson_session.sh --with-telemetry
#
# Usage:
#   ./scripts/start_host_session.sh
#   ./scripts/start_host_session.sh --no-attach
#   ./scripts/start_host_session.sh --session=basestation
#   ./scripts/start_host_session.sh --keep-conflicting-mosquitto
#
# To stop everything:
#   docker compose -f control_center/docker-compose.yaml down
#   tmux kill-session -t host
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

if docker compose version >/dev/null 2>&1; then
    COMPOSE_CMD="docker compose"
elif command -v docker-compose >/dev/null 2>&1; then
    COMPOSE_CMD="docker-compose"
else
    echo "No working Docker Compose command found." >&2
    exit 1
fi

SESSION="host"
ATTACH=true
STOP_CONFLICTING_MOSQUITTO=true

for arg in "$@"; do
    case "${arg}" in
        --no-attach)                  ATTACH=false ;;
        --session=*)                  SESSION="${arg#--session=}" ;;
        --keep-conflicting-mosquitto) STOP_CONFLICTING_MOSQUITTO=false ;;
        -h|--help)
            sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
            exit 0 ;;
    esac
done

CC_COMPOSE="${PROJECT_DIR}/control_center/docker-compose.yaml"

CONFLICTING_MOSQUITTO_CONTAINERS="$(
    docker ps --format '{{.Names}} {{.Image}}' \
    | awk '$2 == "eclipse-mosquitto:2" && $1 != "control_center-mosquitto-1" { print $1 }'
)"

if [[ -n "${CONFLICTING_MOSQUITTO_CONTAINERS}" ]]; then
    echo "Conflicting Mosquitto containers detected on this host:"
    printf '  %s\n' "${CONFLICTING_MOSQUITTO_CONTAINERS}"
    echo ""
    if [[ "${STOP_CONFLICTING_MOSQUITTO}" == true ]]; then
        echo "Stopping conflicting Mosquitto containers so Control Center can claim host port 1883..."
        while IFS= read -r container_name; do
            [[ -z "${container_name}" ]] && continue
            docker stop "${container_name}" >/dev/null
            echo "  stopped ${container_name}"
        done <<< "${CONFLICTING_MOSQUITTO_CONTAINERS}"
        echo ""
    else
        echo "Cannot start Control Center: MQTT port 1883 is likely already occupied."
        echo "Stop those containers first, then rerun:"
        echo "  docker stop ${CONFLICTING_MOSQUITTO_CONTAINERS//$'\n'/ }"
        exit 1
    fi
fi

tmux kill-session -t "${SESSION}" 2>/dev/null || true

tmux new-session -d -s "${SESSION}" -n "host"
tmux set-option -t "${SESSION}" -g mouse on
tmux set-option -t "${SESSION}" -g pane-base-index 0

# Create panes using pane IDs so the layout stays stable.
PANE_CC="$(tmux display-message -p -t "${SESSION}:host.0" "#{pane_id}")"
PANE_LOGS="$(tmux split-window -h -P -F "#{pane_id}" -t "${PANE_CC}")"
PANE_TEST="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_CC}")"
PANE_MQTT="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_LOGS}")"

# ── Pane 0: Control Center stack ──────────────────────────────────────
tmux send-keys -t "${PANE_CC}" \
    "echo '--- Starting Control Center stack ---' && ${COMPOSE_CMD} -f ${CC_COMPOSE} up -d && sleep 3 && python3 ${PROJECT_DIR}/control_center/scripts/setup_influxdb_dashboard.py && echo '' && echo '  Dashboard:  http://localhost:8080' && echo '  InfluxDB:   http://localhost:8086  (rover / rover-password)' && echo '  Test:       ./scripts/test_control_center.sh' && echo '' && echo '--- Control Center ready ---'" Enter

# ── Pane 1: CC container logs ─────────────────────────────────────────
tmux send-keys -t "${PANE_LOGS}" \
    "sleep 5 && ${COMPOSE_CMD} -f ${CC_COMPOSE} logs -f" Enter

# ── Pane 2: Test / shell ─────────────────────────────────────────────
tmux send-keys -t "${PANE_TEST}" \
    "echo 'Control Center test suite ready.' && echo 'Run: ./scripts/test_control_center.sh' && echo ''" Enter

# ── Pane 3: MQTT monitor (subscribe + decode Protobuf) ────────────────
tmux send-keys -t "${PANE_MQTT}" \
    "sleep 8 && bash ${PROJECT_DIR}/scripts/run_host_mqtt_monitor.sh" Enter

tmux select-pane -t "${PANE_CC}"

if [[ "${ATTACH}" == true && -t 1 ]]; then
    tmux attach-session -t "${SESSION}"
else
    echo "Tmux session '${SESSION}' created."
    echo "Attach with: tmux attach-session -t ${SESSION}"
fi
