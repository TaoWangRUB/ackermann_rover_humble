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
USB_DEVICE_FROM_ENV=false
if [[ -n "${PX4_USB_DEVICE+x}" ]]; then
    USB_DEVICE_FROM_ENV=true
fi
USB_DEVICE="${PX4_USB_DEVICE:-/dev/ttyACM0}"
USB_BAUD="${PX4_USB_BAUD:-57600}"
UDP_PORT="${PX4_MAVPROXY_UDP_PORT:-14550}"
MAVPROXY_BIN="${MAVPROXY_BIN:-${HOME}/.local/bin/mavproxy.py}"
MAVPROXY_LAUNCHER="${SCRIPT_DIR}/run_mavproxy_with_dtr.py"
PID_FILE="${PROJECT_DIR}/artifacts/ai/px4_mavproxy_bridge.pid"
LOG_FILE="${PROJECT_DIR}/artifacts/ai/px4_mavproxy_bridge.log"
DEVICE_FILE="${PROJECT_DIR}/artifacts/ai/px4_mavproxy_bridge.device"

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
    local prefer_by_id="$2"
    local by_id

    by_id=$(compgen -G '/dev/serial/by-id/*PX4*' | head -n 1 || true)

    if [[ "${prefer_by_id}" == true && -n "${by_id}" ]]; then
        printf '%s\n' "${by_id}"
        return 0
    fi

    if [[ -e "${requested}" ]]; then
        printf '%s\n' "${requested}"
        return 0
    fi

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

find_running_bridge_pids() {
    local pid_from_file=""

    if [[ -f "${PID_FILE}" ]]; then
        pid_from_file="$(<"${PID_FILE}")"
        if [[ -n "${pid_from_file}" ]] && kill -0 "${pid_from_file}" 2>/dev/null; then
            printf '%s\n' "${pid_from_file}"
        fi
    fi

    pgrep -f "run_mavproxy_with_dtr.py.*127.0.0.1:${UDP_PORT}|mavproxy.py.*127.0.0.1:${UDP_PORT}" 2>/dev/null || true
}

find_running_bridge_pid() {
    find_running_bridge_pids | awk '!seen[$0]++' | head -n 1 || true
}

normalize_device_path() {
    local device="$1"

    if [[ "${device}" == /dev/* ]]; then
        readlink -f "${device}" 2>/dev/null || printf '%s\n' "${device}"
        return 0
    fi

    printf '%s\n' "${device}"
}

get_bridge_master_device() {
    local pid="$1"
    local cmdline
    local arg

    if [[ ! -r "/proc/${pid}/cmdline" ]]; then
        return 1
    fi

    cmdline="$(tr '\0' '\n' < "/proc/${pid}/cmdline")"
    while IFS= read -r arg; do
        if [[ "${arg}" == --master=* ]]; then
            arg="${arg#--master=}"
            printf '%s\n' "${arg%%,*}"
            return 0
        fi
    done <<< "${cmdline}"

    return 1
}

get_bridge_python_pid() {
    local pid="$1"
    local child_pid

    if [[ -r "/proc/${pid}/cmdline" ]]; then
        local cmdline
        cmdline="$(tr '\0' '\n' < "/proc/${pid}/cmdline")"
        if grep -qx 'python3' <<< "${cmdline}"; then
            printf '%s\n' "${pid}"
            return 0
        fi
    fi

    while IFS= read -r child_pid; do
        [[ -n "${child_pid}" ]] || continue
        if [[ -r "/proc/${child_pid}/cmdline" ]]; then
            local child_cmdline
            child_cmdline="$(tr '\0' '\n' < "/proc/${child_pid}/cmdline")"
            if grep -qx 'python3' <<< "${child_cmdline}"; then
                printf '%s\n' "${child_pid}"
                return 0
            fi
        fi
    done < <(ps -o pid= --ppid "${pid}" 2>/dev/null)

    return 1
}

get_bridge_active_device() {
    local pid="$1"
    local python_pid
    local fd_path
    local target

    python_pid="$(get_bridge_python_pid "${pid}")" || return 1

    while IFS= read -r fd_path; do
        [[ -n "${fd_path}" ]] || continue
        target="$(readlink -f "${fd_path}" 2>/dev/null || true)"
        if [[ "${target}" == /dev/ttyACM* || "${target}" == /dev/ttyUSB* ]]; then
            printf '%s\n' "${target}"
            return 0
        fi
    done < <(find "/proc/${python_pid}/fd" -maxdepth 1 -mindepth 1 -type l 2>/dev/null | sort)

    return 1
}

bridge_matches_device() {
    local pid="$1"
    local requested_device="$2"
    local normalized_requested
    local active_device

    normalized_requested="$(normalize_device_path "${requested_device}")"

    active_device="$(get_bridge_active_device "${pid}" 2>/dev/null || true)"
    if [[ -n "${active_device}" ]]; then
        printf '%s\n' "${active_device}" > "${DEVICE_FILE}"
        [[ "${active_device}" == "${normalized_requested}" ]]
        return 0
    fi

    if [[ -f "${DEVICE_FILE}" ]]; then
        active_device="$(<"${DEVICE_FILE}")"
        [[ -n "${active_device}" ]] || return 1
        [[ "${active_device}" == "${normalized_requested}" ]]
        return 0
    fi

    return 1
}

show_status() {
    if is_running; then
        local pid
        pid="$(find_running_bridge_pid)"
        echo "MAVProxy bridge running (pid ${pid})"
        get_bridge_master_device "${pid}" | sed 's/^/USB: /' || true
        get_bridge_active_device "${pid}" | sed 's/^/Held tty: /' || true
        if [[ -f "${DEVICE_FILE}" ]]; then
            sed 's/^/Active device: /' "${DEVICE_FILE}"
        fi
        echo "Log: ${LOG_FILE}"
        tail -n 10 "${LOG_FILE}" 2>/dev/null || true
    else
        echo "MAVProxy bridge not running"
    fi
}

stop_bridge() {
    if ! is_running; then
        rm -f "${PID_FILE}"
        rm -f "${DEVICE_FILE}"
        echo "MAVProxy bridge not running"
        return 0
    fi

    local pids
    local pid

    pids="$(find_running_bridge_pids | awk '!seen[$0]++')"
    while IFS= read -r pid; do
        [[ -n "${pid}" ]] || continue
        kill "${pid}" 2>/dev/null || true
    done <<< "${pids}"

    for _ in $(seq 1 10); do
        local remaining=false
        while IFS= read -r pid; do
            [[ -n "${pid}" ]] || continue
            if kill -0 "${pid}" 2>/dev/null; then
                remaining=true
                break
            fi
        done <<< "${pids}"
        if [[ "${remaining}" == false ]]; then
            break
        fi
        sleep 0.5
    done

    while IFS= read -r pid; do
        [[ -n "${pid}" ]] || continue
        if kill -0 "${pid}" 2>/dev/null; then
            kill -9 "${pid}" 2>/dev/null || true
        fi
    done <<< "${pids}"

    rm -f "${PID_FILE}"
    rm -f "${DEVICE_FILE}"
    echo "Stopped MAVProxy bridge"
}

start_bridge() {
    local resolved_device
    local active_device
    local prefer_by_id=false
    if [[ "${USB_DEVICE_FROM_ENV}" == false ]]; then
        prefer_by_id=true
    fi
    resolved_device="$(resolve_usb_device "${USB_DEVICE}" "${prefer_by_id}")"
    active_device="$(normalize_device_path "${resolved_device}")"

    if [[ ! -x "${MAVPROXY_BIN}" ]]; then
        echo "ERROR: MAVProxy not found at ${MAVPROXY_BIN}" >&2
        exit 1
    fi

    if [[ ! -f "${MAVPROXY_LAUNCHER}" ]]; then
        echo "ERROR: MAVProxy launcher not found at ${MAVPROXY_LAUNCHER}" >&2
        exit 1
    fi

    mkdir -p "$(dirname "${PID_FILE}")"

    if is_running; then
        local pid
        pid="$(find_running_bridge_pid)"
        if bridge_matches_device "${pid}" "${resolved_device}"; then
            show_status
            return 0
        fi

        echo "Restarting MAVProxy bridge for updated device: ${resolved_device}" >&2
        stop_bridge
    fi

    if [[ "${FOREGROUND}" == true ]]; then
        echo "Starting MAVProxy bridge in foreground: ${resolved_device} -> 127.0.0.1:${UDP_PORT}"
        exec python3 "${MAVPROXY_LAUNCHER}" --master="${resolved_device},${USB_BAUD}" --out="127.0.0.1:${UDP_PORT}"
    fi

    nohup bash -lc '
        exec tail -f /dev/null | "$@"
    ' _ \
        python3 \
        "${MAVPROXY_LAUNCHER}" \
        --master="${resolved_device},${USB_BAUD}" \
        --out="127.0.0.1:${UDP_PORT}" \
        >"${LOG_FILE}" 2>&1 < /dev/null &
    local pid=$!
    echo "${pid}" > "${PID_FILE}"
    echo "${active_device}" > "${DEVICE_FILE}"
    sleep 2

    if ! kill -0 "${pid}" 2>/dev/null; then
        echo "ERROR: MAVProxy bridge exited during startup" >&2
        tail -n 20 "${LOG_FILE}" 2>/dev/null || true
        rm -f "${PID_FILE}"
        rm -f "${DEVICE_FILE}"
        exit 1
    fi

    local actual_device
    actual_device="$(get_bridge_active_device "${pid}" 2>/dev/null || true)"
    if [[ -n "${actual_device}" ]]; then
        printf '%s\n' "${actual_device}" > "${DEVICE_FILE}"
    else
        printf '%s\n' "${active_device}" > "${DEVICE_FILE}"
    fi

    echo "Started MAVProxy bridge (pid ${pid})"
    echo "USB: ${resolved_device} @ ${USB_BAUD}"
    if [[ -n "${actual_device}" ]]; then
        echo "Held tty: ${actual_device}"
    fi
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