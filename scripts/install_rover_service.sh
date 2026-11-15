#!/usr/bin/env bash
# Install (or remove) the systemd service that starts the rover stack at boot.
#
# The service runs scripts/start_jetson_session.sh --no-attach --no-activate
# as the current user, in the tmux session "jetson". It never arms the rover.
# Run this on the rover's Jetson as the normal user (it calls sudo itself).
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

if [[ "${EUID}" -eq 0 ]]; then
    echo "ERROR: run as the user who owns the workspace, not root (sudo is used where needed)." >&2
    exit 1
fi

if [[ "${ACTION}" == "uninstall" ]]; then
    sudo systemctl disable --now "${UNIT_NAME}" 2>/dev/null || true
    sudo rm -f "${UNIT_PATH}"
    sudo systemctl daemon-reload
    echo "Removed ${UNIT_NAME}. ${DEFAULTS_PATH} was kept."
    exit 0
fi

# ── Preconditions ────────────────────────────────────────────────────
command -v tmux >/dev/null || { echo "ERROR: tmux is not installed." >&2; exit 1; }
docker info >/dev/null 2>&1 || {
    echo "ERROR: '$(id -un)' cannot use docker without sudo (add it to the docker group)." >&2
    exit 1
}
[[ -f "${TEMPLATE}" ]] || { echo "ERROR: missing ${TEMPLATE}" >&2; exit 1; }

# ── Unit file ────────────────────────────────────────────────────────
tmp_unit="$(mktemp)"
trap 'rm -f "${tmp_unit}"' EXIT
sed -e "s|@USER@|$(id -un)|g" -e "s|@GROUP@|$(id -gn)|g" -e "s|@REPO@|${REPO_DIR}|g" \
    "${TEMPLATE}" > "${tmp_unit}"
sudo install -m 644 "${tmp_unit}" "${UNIT_PATH}"
echo "Installed ${UNIT_PATH}"

# ── Settings ─────────────────────────────────────────────────────────
if [[ -n "${ARGS}" || -n "${BROKER_HOST}" || ! -f "${DEFAULTS_PATH}" ]]; then
    session_args="${ARGS:-${DEFAULT_ARGS}}"
    if [[ -n "${BROKER_HOST}" ]]; then
        session_args="${session_args} --broker-host=${BROKER_HOST}"
    fi
    sudo tee "${DEFAULTS_PATH}" >/dev/null <<EOF
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

sudo systemctl daemon-reload
sudo systemctl enable "${UNIT_NAME}" >/dev/null
echo "Enabled ${UNIT_NAME}: the stack starts at the next boot."

if [[ "${START_NOW}" == true ]]; then
    echo "Restarting ${UNIT_NAME} now (stops any running 'jetson' session first)..."
    sudo systemctl restart "${UNIT_NAME}"
    systemctl --no-pager --lines=5 status "${UNIT_NAME}" || true
fi

cat <<EOF

  state:   systemctl status ${UNIT_NAME}
  output:  journalctl -u ${UNIT_NAME} -b
  panes:   tmux attach -t jetson
  remove:  ./scripts/install_rover_service.sh --uninstall
EOF
