#!/usr/bin/env bash
# Install (or remove) the systemd units that start the rover stack at boot.
#
# Two modes; installing one disables the other:
#   units (default)  one unit per component (scripts/systemd/units), started in
#                    dependency order with readiness gates: container -> USB
#                    devices -> robot description -> T265 -> D435i -> PX4 link
#                    -> MAVLink bridge -> VO bridge + RoverManual -> monitor ->
#                    RTAB-Map -> Nav2. Grouped by rover-stack.target.
#   tmux             one unit (scripts/systemd/rover-stack.service.in) that runs
#                    scripts/start_jetson_session.sh in the tmux session "jetson".
# Neither ever arms the rover. Run this on the rover's Jetson as the workspace
# user (it calls sudo itself), or with sudo from that user's shell, e.g. when
# sudo reads the password from stdin: `sudo -S ./scripts/install_rover_service.sh ...`.
#
# Usage:
#   ./scripts/install_rover_service.sh [--mode=units|tmux]   # install + enable at boot
#   ./scripts/install_rover_service.sh --now                 # ... and (re)start it now
#   ./scripts/install_rover_service.sh --broker-host=192.168.0.225
#   ./scripts/install_rover_service.sh --mode=tmux --args="--t265-odom --nav2 --with-telemetry"
#   ./scripts/install_rover_service.sh --uninstall           # stop, disable, remove both modes
#
# Settings are kept in /etc/default/rover-stack: ROVER_BROKER_HOST (Control
# Center / MQTT broker) for the units, ROVER_STACK_ARGS for tmux mode, and the
# optional overrides listed there. The file is only rewritten when
# --broker-host or --args is given, so local edits survive a re-install.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
TMUX_TEMPLATE="${SCRIPT_DIR}/systemd/rover-stack.service.in"
UNITS_DIR="${SCRIPT_DIR}/systemd/units"
SYSTEMD_DIR="/etc/systemd/system"
DEFAULTS_PATH="/etc/default/rover-stack"
LOGIND_DROPIN="/etc/systemd/logind.conf.d/10-rover-keep-ipc.conf"
JOURNALD_DROPIN="/etc/systemd/journald.conf.d/10-rover-persistent.conf"
DEFAULT_ARGS="--t265-odom --nav2 --with-telemetry"
DEFAULT_BROKER_HOST="192.168.0.225"
SESSION="jetson"

MODE="units"
ACTION="install"
START_NOW=false
ARGS=""
BROKER_HOST=""
for arg in "$@"; do
    case "${arg}" in
        --mode=*)        MODE="${arg#--mode=}" ;;
        --now)           START_NOW=true ;;
        --uninstall)     ACTION="uninstall" ;;
        --args=*)        ARGS="${arg#--args=}" ;;
        --broker-host=*) BROKER_HOST="${arg#--broker-host=}" ;;
        -h|--help)       sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"; exit 0 ;;
        *) echo "ERROR: unknown argument: ${arg}" >&2; exit 1 ;;
    esac
done
[[ "${MODE}" == units || "${MODE}" == tmux ]] || { echo "ERROR: --mode must be units or tmux" >&2; exit 1; }

# The service runs as the workspace user: the caller, or the user who ran sudo.
if [[ "${EUID}" -eq 0 ]]; then
    if [[ -z "${SUDO_USER:-}" || "${SUDO_USER}" == "root" ]]; then
        echo "ERROR: run as the workspace user, or with sudo from that user's shell." >&2
        exit 1
    fi
    SERVICE_USER="${SUDO_USER}"
else
    SERVICE_USER="$(id -un)"
fi
SERVICE_GROUP="$(id -gn "${SERVICE_USER}")"

as_root() { if [[ "${EUID}" -eq 0 ]]; then "$@"; else sudo "$@"; fi; }
as_user() { if [[ "${EUID}" -eq 0 ]]; then sudo -u "${SERVICE_USER}" -H "$@"; else "$@"; fi; }

unit_names() { local f; for f in "${UNITS_DIR}"/*.in; do basename "${f%.in}"; done; }
component_services() { unit_names | grep '\.service$'; }

render() {  # render TEMPLATE DEST
    local tmp
    tmp="$(mktemp)"
    sed -e "s|@USER@|${SERVICE_USER}|g" -e "s|@GROUP@|${SERVICE_GROUP}|g" -e "s|@REPO@|${REPO_DIR}|g" "$1" > "${tmp}"
    as_root install -m 644 "${tmp}" "$2"
    rm -f "${tmp}"
}

remove_tmux_mode() {
    as_root systemctl disable --now rover-stack.service 2>/dev/null || true
    as_root rm -f "${SYSTEMD_DIR}/rover-stack.service"
}
remove_units_mode() {
    local unit
    as_root systemctl stop rover-stack.target 2>/dev/null || true
    for unit in $(unit_names); do
        as_root systemctl disable "${unit}" 2>/dev/null || true
        as_root rm -f "${SYSTEMD_DIR}/${unit}"
    done
}

if [[ "${ACTION}" == "uninstall" ]]; then
    remove_units_mode
    remove_tmux_mode
    as_root rm -f "${LOGIND_DROPIN}" "${JOURNALD_DROPIN}"
    as_root systemctl daemon-reload
    echo "Removed the rover units (both modes), ${LOGIND_DROPIN} and ${JOURNALD_DROPIN}."
    echo "${DEFAULTS_PATH} and the logs in /var/log/journal were kept;"
    echo "lingering for ${SERVICE_USER} was left on (loginctl disable-linger ${SERVICE_USER} turns it off)."
    exit 0
fi

# ── Preconditions ────────────────────────────────────────────────────
command -v tmux >/dev/null || { echo "ERROR: tmux is not installed." >&2; exit 1; }
as_user docker info >/dev/null 2>&1 || {
    echo "ERROR: '${SERVICE_USER}' cannot use docker without sudo (add it to the docker group)." >&2
    exit 1
}

# ── Settings ─────────────────────────────────────────────────────────
if [[ -n "${ARGS}" || -n "${BROKER_HOST}" || ! -f "${DEFAULTS_PATH}" ]] \
        || ! grep -q '^ROVER_BROKER_HOST=' "${DEFAULTS_PATH}"; then
    broker="${BROKER_HOST:-${DEFAULT_BROKER_HOST}}"
    session_args="${ARGS:-${DEFAULT_ARGS}} --broker-host=${broker}"
    as_root tee "${DEFAULTS_PATH}" >/dev/null <<EOF
# Settings for the rover systemd units (written by scripts/install_rover_service.sh).
# Control Center machine running the MQTT broker (units mode).
ROVER_BROKER_HOST=${broker}
# tmux mode: arguments for scripts/start_jetson_session.sh (the unit adds
# --no-attach --no-activate) and the start delay in seconds.
ROVER_STACK_ARGS="${session_args}"
ROVER_STACK_START_DELAY_S=20
# Optional overrides for units mode (defaults in each unit):
#ROVER_REVERSIBLE_DRIVE=true
#ROVER_NAV2_CONTROLLER=mppi
#ROVER_RTABMAP_DELETE_DB=true
#ROVER_XRCE_DEVICE=/dev/ttyUSB0
#ROVER_MAVLINK_DEVICE=/dev/ttyACM0
#ROVER_T265_HW_RESET=false
EOF
    echo "Wrote ${DEFAULTS_PATH} (broker ${broker})"
else
    echo "Kept ${DEFAULTS_PATH}:"
    grep -E '^ROVER_' "${DEFAULTS_PATH}" | sed 's/^/  /'
fi

# ── Keep the stack's shared memory ───────────────────────────────────
# systemd-logind deletes all of a user's /dev/shm objects when that user's
# last login session ends (RemoveIPC=yes, Ubuntu's default). The stack runs as
# that user but outside any session, and Fast DDS moves camera data through
# /dev/shm segments: every SSH logout wiped them, the cameras kept running and
# no subscriber got data any more (found 2026-11-15). Lingering keeps the user
# logged in (effective now); the drop-in turns the removal off (next boot).
as_root loginctl enable-linger "${SERVICE_USER}"
as_root mkdir -p "${LOGIND_DROPIN%/*}"
printf '# Written by scripts/install_rover_service.sh: keep the rover stack'"'"'s Fast DDS\n# shared memory (/dev/shm) when the last login session of its user ends.\n[Login]\nRemoveIPC=no\n' \
    | as_root tee "${LOGIND_DROPIN}" >/dev/null
echo "Lingering on for ${SERVICE_USER}; ${LOGIND_DROPIN} sets RemoveIPC=no (applies after the next boot)."

# ── Keep the journal across reboots ──────────────────────────────────
# Without /var/log/journal, journald keeps the log in RAM: lost at every reboot
# and capped at about 68 MB, roughly 6 hours of this stack (about 12,000 lines,
# 17 MB, an hour; found 2026-11-17). On disk it is capped at 4 GB and keeps
# 5 GB of the disk free: about a week (2 GB filled in 3.5 days). `journalctl -b -1` misses earlier boots
# on the Xavier; scripts/rover_units/journal_boots.sh lists them.
as_root mkdir -p "${JOURNALD_DROPIN%/*}" /var/log/journal
printf '# Written by scripts/install_rover_service.sh: keep the journal on disk, capped.\n[Journal]\nStorage=persistent\nSystemMaxUse=4G\nSystemKeepFree=5G\n' \
    | as_root tee "${JOURNALD_DROPIN}" >/dev/null
as_root systemd-tmpfiles --create --prefix /var/log/journal
as_root journalctl --flush
echo "${JOURNALD_DROPIN}: journal kept on disk in /var/log/journal (4 GB cap; restart systemd-journald or reboot to apply a changed cap)."

# ── Units ────────────────────────────────────────────────────────────
if [[ "${MODE}" == units ]]; then
    remove_tmux_mode
    for unit in $(unit_names); do
        render "${UNITS_DIR}/${unit}.in" "${SYSTEMD_DIR}/${unit}"
    done
    as_root systemctl daemon-reload
    # shellcheck disable=SC2046
    as_root systemctl enable rover-stack.target $(component_services) >/dev/null
    echo "Installed and enabled rover-stack.target + $(component_services | wc -l) component units (runs as ${SERVICE_USER})."
else
    remove_units_mode
    render "${TMUX_TEMPLATE}" "${SYSTEMD_DIR}/rover-stack.service"
    as_root systemctl daemon-reload
    as_root systemctl enable rover-stack.service >/dev/null
    echo "Installed and enabled rover-stack.service (tmux session '${SESSION}', runs as ${SERVICE_USER})."
fi

if [[ "${START_NOW}" == true ]]; then
    # Stop a session that was started by hand, so the components start from a
    # clean state (and the T265 gets its pause between stop and start).
    if as_user tmux has-session -t "${SESSION}" 2>/dev/null; then
        echo "Stopping the '${SESSION}' tmux session..."
        as_user "${REPO_DIR}/scripts/stop_all.sh" --session="${SESSION}"
        sleep 15
    fi
    if [[ "${MODE}" == units ]]; then
        echo "Starting rover-stack.target (components start in order; several minutes)..."
        as_root systemctl stop rover-stack.target 2>/dev/null || true
        as_root systemctl start --no-block rover-stack.target
        echo "Follow it with: journalctl -u 'rover-*' -f   or   ${REPO_DIR}/scripts/rover_units/status.sh"
    else
        echo "Starting rover-stack.service (start delay, then the session; 1-2 min)..."
        as_root systemctl restart rover-stack.service
        systemctl --no-pager --lines=5 status rover-stack.service || true
    fi
fi

if [[ "${MODE}" == units ]]; then
    cat <<EOF

  state:   ${REPO_DIR}/scripts/rover_units/status.sh   (or systemctl list-units 'rover-*')
  logs:    journalctl -u 'rover-*' -b -f
  restart: sudo systemctl restart rover-stack.target
  remove:  ./scripts/install_rover_service.sh --uninstall
EOF
else
    cat <<EOF

  state:   systemctl status rover-stack.service
  output:  journalctl -u rover-stack.service -b
  panes:   tmux attach -t ${SESSION}
  remove:  ./scripts/install_rover_service.sh --uninstall
EOF
fi
