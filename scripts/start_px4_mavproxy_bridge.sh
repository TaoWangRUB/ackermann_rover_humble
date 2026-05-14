#!/usr/bin/env bash
# Start, stop, or inspect a host-side MAVProxy bridge for PX4 USB -> UDP.
#
# Usage:
#   ./scripts/start_px4_mavproxy_bridge.sh                  # start once and reuse for later px4_cmd/upload runs
#   ./scripts/start_px4_mavproxy_bridge.sh --status
#   ./scripts/start_px4_mavproxy_bridge.sh --stop
#   ./scripts/start_px4_mavproxy_bridge.sh --restart
#   ./scripts/start_px4_mavproxy_bridge.sh --foreground
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

ACTION="start"
FOREGROUND=false
USB_DEVICE="${PX4_USB_DEVICE:-/dev/ttyACM0}"
USB_BAUD="${PX4_USB_BAUD:-115200}"
UDP_PORT="${PX4_MAVPROXY_UDP_PORT:-14550}"
MAVPROXY_BIN="${MAVPROXY_BIN:-${HOME}/.local/bin/mavproxy.py}"
PID_FILE="${PROJECT_DIR}/artifacts/ai/px4_mavproxy_bridge.pid"
LOG_FILE="${PROJECT_DIR}/artifacts/ai/px4_mavproxy_bridge.log"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --status) ACTION="status" ;;
        --stop) ACTION="stop" ;;
        --restart) ACTION="restart" ;;
        --foreground) FOREGROUND=true ;;
        --device)
            USB_DEVICE="${2:?--device requires a value}"
            shift
            ;;
        --baud)
            USB_BAUD="${2:?--baud requires a value}"
            shift
            ;;
        --udp-port)
            UDP_PORT="${2:?--udp-port requires a value}"
            shift
            ;;
        -h|--help)
            sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
            exit 0
            ;;
        *)
            echo "ERROR: Unknown argument: $1" >&2
            exit 1
            ;;
    esac
    shift
done

resolve_usb_device() {
    local requested="$1"
    local by_id

    if [[ -e "${requested}" ]]; then
        printf '%s\n' "${requested}"
        return 0
    fi

    by_id=$(compgen -G '/dev/serial/by-id/*PX4*' | head -n 1 || true)
    if [[ -n "${by_id}" ]]; then
        printf '%s\n' "${by_id}"
        return 0
    fi

    printf '%s\n' "${requested}"
}

is_running() {
    local pid
    pid="$(find_running_bridge_pid)"
    [[ -n "${pid}" ]]
}

find_running_bridge_pid() {
    if [[ -f "${PID_FILE}" ]]; then
        local pid
        pid="$(<"${PID_FILE}")"
        if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
            printf '%s\n' "${pid}"
            return 0
        fi
    fi

    pgrep -f "mavproxy.py.*127.0.0.1:${UDP_PORT}" | head -n 1 || true
}

show_status() {
    if is_running; then
        local pid
        pid="$(find_running_bridge_pid)"
        echo "MAVProxy bridge running (pid ${pid})"
        echo "Log: ${LOG_FILE}"
        tail -n 10 "${LOG_FILE}" 2>/dev/null || true
    else
        echo "MAVProxy bridge not running"
    fi
}

stop_bridge() {
    if ! is_running; then
        rm -f "${PID_FILE}"
        echo "MAVProxy bridge not running"
        return 0
    fi

    local pid
    pid="$(find_running_bridge_pid)"
    kill "${pid}" 2>/dev/null || true
    for _ in $(seq 1 10); do
        if ! kill -0 "${pid}" 2>/dev/null; then
            break
        fi
        sleep 0.5
    done
    if kill -0 "${pid}" 2>/dev/null; then
        kill -9 "${pid}" 2>/dev/null || true
    fi
    rm -f "${PID_FILE}"
    echo "Stopped MAVProxy bridge"
}

start_bridge() {
    local resolved_device
    resolved_device="$(resolve_usb_device "${USB_DEVICE}")"

    if [[ ! -x "${MAVPROXY_BIN}" ]]; then
        echo "ERROR: MAVProxy not found at ${MAVPROXY_BIN}" >&2
        exit 1
    fi

    mkdir -p "$(dirname "${PID_FILE}")"

    if is_running; then
        show_status
        return 0
    fi

    if [[ "${FOREGROUND}" == true ]]; then
        echo "Starting MAVProxy bridge in foreground: ${resolved_device} -> 127.0.0.1:${UDP_PORT}"
        exec "${MAVPROXY_BIN}" --master="${resolved_device},${USB_BAUD}" --out="127.0.0.1:${UDP_PORT}"
    fi

    nohup "${MAVPROXY_BIN}" \
        --master="${resolved_device},${USB_BAUD}" \
        --out="127.0.0.1:${UDP_PORT}" \
        >"${LOG_FILE}" 2>&1 < /dev/null &
    local pid=$!
    echo "${pid}" > "${PID_FILE}"
    sleep 2

    if ! kill -0 "${pid}" 2>/dev/null; then
        echo "ERROR: MAVProxy bridge exited during startup" >&2
        tail -n 20 "${LOG_FILE}" 2>/dev/null || true
        rm -f "${PID_FILE}"
        exit 1
    fi

    echo "Started MAVProxy bridge (pid ${pid})"
    echo "USB: ${resolved_device} @ ${USB_BAUD}"
    echo "UDP: 127.0.0.1:${UDP_PORT}"
    echo "Log: ${LOG_FILE}"
}

case "${ACTION}" in
    start)
        start_bridge
        ;;
    stop)
        stop_bridge
        ;;
    restart)
        stop_bridge
        start_bridge
        ;;
    status)
        show_status
        ;;
esac