#!/usr/bin/env bash
# Install (or remove) the systemd service that starts the rover stack at boot.
#
# The service runs scripts/start_jetson_session.sh --no-attach --no-activate
# as the workspace user, in the tmux session "jetson". It never arms the rover.
# Run this on the rover's Jetson as that user (it calls sudo itself), or with
# sudo from that user's shell, e.g. when sudo has to read the password from
# stdin: `sudo -S ./scripts/install_rover_service.sh ...`.
#
# Usage:
#   ./scripts/install_rover_service.sh                      # install + enable at boot
#   ./scripts/install_rover_service.sh --now                # ... and (re)start it now
#   ./scripts/install_rover_service.sh --broker-host=192.168.0.225
#   ./scripts/install_rover_service.sh --args="--t265-odom --nav2 --with-telemetry"
#   ./scripts/install_rover_service.sh --uninstall          # stop, disable, remove
#
# Settings are kept in /etc/default/rover-stack (ROVER_STACK_ARGS, the
# arguments for start_jetson_session.sh, and ROVER_STACK_START_DELAY_S). The
# file is only rewritten when --args or --broker-host is given, so local edits
# survive a re-install.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
TEMPLATE="${SCRIPT_DIR}/systemd/rover-stack.service.in"
UNIT_NAME="rover-stack.service"
UNIT_PATH="/etc/systemd/system/${UNIT_NAME}"
DEFAULTS_PATH="/etc/default/rover-stack"
DEFAULT_ARGS="--t265-odom --nav2 --with-telemetry"
SESSION="jetson"

ACTION="install"
START_NOW=false
ARGS=""
BROKER_HOST=""
for arg in "$@"; do
    case "${arg}" in
        --now)           START_NOW=true ;;
        --uninstall)     ACTION="uninstall" ;;
        --args=*)        ARGS="${arg#--args=}" ;;
        --broker-host=*) BROKER_HOST="${arg#--broker-host=}" ;;
        -h|--help)       sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"; exit 0 ;;
        *) echo "ERROR: unknown argument: ${arg}" >&2; exit 1 ;;
    esac
done

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

if [[ "${ACTION}" == "uninstall" ]]; then
    as_root systemctl disable --now "${UNIT_NAME}" 2>/dev/null || true
    as_root rm -f "${UNIT_PATH}"
    as_root systemctl daemon-reload
    echo "Removed ${UNIT_NAME}. ${DEFAULTS_PATH} was kept."
    exit 0
fi

# ── Preconditions ────────────────────────────────────────────────────
command -v tmux >/dev/null || { echo "ERROR: tmux is not installed." >&2; exit 1; }
as_user docker info >/dev/null 2>&1 || {
    echo "ERROR: '${SERVICE_USER}' cannot use docker without sudo (add it to the docker group)." >&2
    exit 1
}
[[ -f "${TEMPLATE}" ]] || { echo "ERROR: missing ${TEMPLATE}" >&2; exit 1; }

# ── Unit file ────────────────────────────────────────────────────────
tmp_unit="$(mktemp)"
trap 'rm -f "${tmp_unit}"' EXIT
sed -e "s|@USER@|${SERVICE_USER}|g" -e "s|@GROUP@|${SERVICE_GROUP}|g" -e "s|@REPO@|${REPO_DIR}|g" \
    "${TEMPLATE}" > "${tmp_unit}"
as_root install -m 644 "${tmp_unit}" "${UNIT_PATH}"
echo "Installed ${UNIT_PATH} (runs as ${SERVICE_USER})"

# ── Settings ─────────────────────────────────────────────────────────
if [[ -n "${ARGS}" || -n "${BROKER_HOST}" || ! -f "${DEFAULTS_PATH}" ]]; then
    session_args="${ARGS:-${DEFAULT_ARGS}}"
    if [[ -n "${BROKER_HOST}" ]]; then
        session_args="${session_args} --broker-host=${BROKER_HOST}"
    fi
    as_root tee "${DEFAULTS_PATH}" >/dev/null <<EOF
# Settings for ${UNIT_NAME} (written by scripts/install_rover_service.sh).
# Arguments for scripts/start_jetson_session.sh; the service always adds
# --no-attach --no-activate. --broker-host is the Control Center machine.
ROVER_STACK_ARGS="${session_args}"
# Seconds to wait before starting (USB devices settling, T265 restart pause).
ROVER_STACK_START_DELAY_S=20
EOF
    echo "Wrote ${DEFAULTS_PATH}: ROVER_STACK_ARGS=\"${session_args}\""
else
    echo "Kept ${DEFAULTS_PATH}:"
    grep -E '^ROVER_STACK_' "${DEFAULTS_PATH}" | sed 's/^/  /'
fi

as_root systemctl daemon-reload
as_root systemctl enable "${UNIT_NAME}" >/dev/null
echo "Enabled ${UNIT_NAME}: the stack starts at the next boot."

if [[ "${START_NOW}" == true ]]; then
    # A session started by hand is stopped first, so the service's start delay
    # also gives the T265 its pause between stop and start.
    if ! systemctl is-active --quiet "${UNIT_NAME}" \
            && as_user tmux has-session -t "${SESSION}" 2>/dev/null; then
        echo "Stopping the '${SESSION}' session that was started by hand..."
        as_user "${REPO_DIR}/scripts/stop_all.sh" --session="${SESSION}"
    fi
    echo "Starting ${UNIT_NAME} (start delay, then the session; takes 1-2 min)..."
    as_root systemctl restart "${UNIT_NAME}"
    systemctl --no-pager --lines=5 status "${UNIT_NAME}" || true
fi

cat <<EOF

  state:   systemctl status ${UNIT_NAME}
  output:  journalctl -u ${UNIT_NAME} -b
  panes:   tmux attach -t ${SESSION}
  remove:  ./scripts/install_rover_service.sh --uninstall
EOF
