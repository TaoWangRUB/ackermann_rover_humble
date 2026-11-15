#!/usr/bin/env bash
# Readiness gates for the rover-*.service units (ExecStartPre / ExecStartPost).
# Each check waits until the condition holds or the timeout expires (exit 1),
# and says what it is waiting for, so `journalctl -u rover-<x>` shows which
# stage of the startup failed.
#
#   wait_for.sh container [--timeout S]
#       Start the dev container if needed and wait until its entrypoint has
#       finished (it runs apt-get update and rosdep on every start) and
#       `docker compose exec` works.
#   wait_for.sh usb NAME=VID:PID[,VID:PID] ... [--timeout S]
#       Wait until every named USB device is enumerated (alternatives after a
#       comma, e.g. the T265 shows up as 03e7:2150 until its firmware boots).
#   wait_for.sh dev PATH ... [--timeout S]
#       Wait until device nodes exist (e.g. /dev/ttyUSB0).
#   wait_for.sh topic TOPIC TYPE [--latched] [--timeout S]
#       Wait until a message actually arrives on TOPIC (not just until the topic
#       exists). --latched reads transient-local topics such as
#       /robot_description.
#   wait_for.sh nav2 [--timeout S]
#       Wait until Nav2's bt_navigator lifecycle node is active.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../lib/dc.sh
source "${SCRIPT_DIR}/../lib/dc.sh"

check="${1:?usage: wait_for.sh container|usb|dev|topic|nav2 ...}"
shift
timeout_s=60
latched=false
positional=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --timeout) timeout_s="$2"; shift ;;
        --timeout=*) timeout_s="${1#--timeout=}" ;;
        --latched) latched=true ;;
        *) positional+=("$1") ;;
    esac
    shift
done
deadline=$(( $(date +%s) + timeout_s ))
remaining() { echo $(( deadline - $(date +%s) )); }
log() { echo "[wait_for ${check}] $*"; }

# Run a command in the container with the same environment as the components.
in_container() {
    dcomp exec -T ackermann_slam bash -c '
        [[ -f "$HOME/.fastdds_eth_env.sh" ]] && source "$HOME/.fastdds_eth_env.sh"
        source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
        [[ -f /workspace/install/setup.bash ]] && source /workspace/install/setup.bash
        "$@"' _ "$@"
}

case "${check}" in
container)
    log "waiting for the ackermann_slam container (timeout ${timeout_s}s)"
    last_up=0
    while (( $(remaining) > 0 )); do
        cid="$(dcomp ps -q ackermann_slam 2>/dev/null)"
        if [[ -z "${cid}" ]] || [[ "$(docker inspect -f '{{.State.Running}}' "${cid}" 2>/dev/null)" != true ]]; then
            # Retried: at boot dcomp refuses to run until tailscale0 has its
            # address (ROS_DDS_MODE=tailscale), and Docker may still be starting.
            if (( $(date +%s) - last_up >= 10 )); then
                log "container not running: starting it"
                dcomp up -d ackermann_slam || true
                last_up=$(date +%s)
            fi
            sleep 2
            continue
        fi
        # The entrypoint ends with `exec "$@"`; until then the main process is
        # still entrypoint.sh (apt-get update, rosdep install).
        main_pid="$(docker inspect -f '{{.State.Pid}}' "${cid}" 2>/dev/null)"
        if tr '\0' ' ' < "/proc/${main_pid}/cmdline" 2>/dev/null | grep -q 'entrypoint'; then
            sleep 2
            continue
        fi
        if dcomp exec -T ackermann_slam true >/dev/null 2>&1; then
            log "container ready (main process: $(tr '\0' ' ' < "/proc/${main_pid}/cmdline" 2>/dev/null))"
            exit 0
        fi
        sleep 2
    done
    log "TIMEOUT: container not ready after ${timeout_s}s"
    exit 1
    ;;

usb)
    [[ ${#positional[@]} -gt 0 ]] || { log "no devices given"; exit 2; }
    log "waiting for USB devices: ${positional[*]} (timeout ${timeout_s}s)"
    while true; do
        present="$(lsusb 2>/dev/null | awk '{print $6}')"
        missing=()
        for spec in "${positional[@]}"; do
            name="${spec%%=*}"
            found=false
            IFS=',' read -r -a ids <<< "${spec#*=}"
            for id in "${ids[@]}"; do
                grep -qx "${id}" <<< "${present}" && found=true
            done
            [[ "${found}" == true ]] || missing+=("${name}")
        done
        if [[ ${#missing[@]} -eq 0 ]]; then
            log "all present"
            exit 0
        fi
        if (( $(remaining) <= 0 )); then
            log "TIMEOUT: missing after ${timeout_s}s: ${missing[*]}"
            exit 1
        fi
        sleep 2
    done
    ;;

dev)
    [[ ${#positional[@]} -gt 0 ]] || { log "no paths given"; exit 2; }
    log "waiting for ${positional[*]} (timeout ${timeout_s}s)"
    while true; do
        missing=()
        for path in "${positional[@]}"; do
            [[ -e "${path}" ]] || missing+=("${path}")
        done
        if [[ ${#missing[@]} -eq 0 ]]; then
            log "all present"
            exit 0
        fi
        if (( $(remaining) <= 0 )); then
            log "TIMEOUT: missing after ${timeout_s}s: ${missing[*]}"
            exit 1
        fi
        sleep 1
    done
    ;;

topic)
    [[ ${#positional[@]} -eq 2 ]] || { log "usage: wait_for.sh topic TOPIC TYPE [--latched] [--timeout S]"; exit 2; }
    topic="${positional[0]}"
    msg_type="${positional[1]}"
    qos=(--qos-reliability best_effort)
    if [[ "${latched}" == true ]]; then
        qos=(--qos-reliability reliable --qos-durability transient_local)
    fi
    log "waiting for a message on ${topic} (timeout ${timeout_s}s)"
    # One subscriber for the whole wait; SIGINT on timeout so rclpy shuts down
    # cleanly instead of leaving a dead DDS participant behind.
    if in_container timeout -s INT -k 5 "${timeout_s}" \
        ros2 topic echo --once --no-arr --no-str "${qos[@]}" "${topic}" "${msg_type}" >/dev/null 2>&1; then
        log "got a message on ${topic}"
        exit 0
    fi
    log "TIMEOUT: no message on ${topic} within ${timeout_s}s"
    exit 1
    ;;

nav2)
    log "waiting for Nav2 (bt_navigator active, timeout ${timeout_s}s)"
    while (( $(remaining) > 0 )); do
        state="$(in_container timeout -s INT -k 5 20 ros2 lifecycle get /bt_navigator 2>/dev/null | head -1)"
        if [[ "${state}" == active* ]]; then
            log "Nav2 active"
            exit 0
        fi
        sleep 5
    done
    log "TIMEOUT: bt_navigator not active after ${timeout_s}s (last state: ${state:-unknown})"
    exit 1
    ;;

*)
    log "unknown check '${check}'"
    exit 2
    ;;
esac
