#!/usr/bin/env bash
# Install (or remove) rover-wifi-watchdog.service on the rover's Jetson. It
# reconnects the WiFi when NetworkManager gives up on it, so the rover stays
# on the tailnet with nobody logged in (why: scripts/wifi_watchdog.sh). It is
# independent of the rover stack units (scripts/install_rover_service.sh).
#
# The service runs a copy, /usr/local/sbin/rover-wifi-watchdog: re-run this
# script after changing scripts/wifi_watchdog.sh.
#
# Usage (as the workspace user; it calls sudo itself):
#   ./scripts/install_wifi_watchdog.sh              # install or update, enable, (re)start
#   ./scripts/install_wifi_watchdog.sh --uninstall  # stop, disable, remove
#
# Settings: /etc/default/rover-wifi-watchdog, written once with the defaults
# commented out (local edits survive a re-install).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_SCRIPT="${SCRIPT_DIR}/wifi_watchdog.sh"
SRC_UNIT="${SCRIPT_DIR}/systemd/rover-wifi-watchdog.service"
BIN_PATH="/usr/local/sbin/rover-wifi-watchdog"
UNIT_NAME="rover-wifi-watchdog.service"
UNIT_PATH="/etc/systemd/system/${UNIT_NAME}"
DEFAULTS_PATH="/etc/default/rover-wifi-watchdog"

ACTION="install"
for arg in "$@"; do
    case "${arg}" in
        --uninstall) ACTION="uninstall" ;;
        -h|--help)   sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"; exit 0 ;;
        *) echo "ERROR: unknown argument: ${arg}" >&2; exit 1 ;;
    esac
done

as_root() { if [[ "${EUID}" -eq 0 ]]; then "$@"; else sudo "$@"; fi; }

if [[ "${ACTION}" == "uninstall" ]]; then
    as_root systemctl disable --now "${UNIT_NAME}" 2>/dev/null || true
    as_root rm -f "${UNIT_PATH}" "${BIN_PATH}"
    as_root systemctl daemon-reload
    echo "Removed ${UNIT_NAME} and ${BIN_PATH} (${DEFAULTS_PATH} was kept)."
    exit 0
fi

command -v nmcli >/dev/null || { echo "ERROR: nmcli not found; the watchdog drives NetworkManager." >&2; exit 1; }

as_root install -m 755 "${SRC_SCRIPT}" "${BIN_PATH}"
as_root install -m 644 "${SRC_UNIT}" "${UNIT_PATH}"
if [[ ! -f "${DEFAULTS_PATH}" ]]; then
    as_root tee "${DEFAULTS_PATH}" >/dev/null <<'EOF'
# Settings for rover-wifi-watchdog.service (written by scripts/install_wifi_watchdog.sh).
# Defaults shown. After a change: sudo systemctl restart rover-wifi-watchdog
#WIFI_WATCHDOG_IFACE=wlan0            # default: the first WiFi device NetworkManager lists
#WIFI_WATCHDOG_CHECK_S=15             # poll interval
#WIFI_WATCHDOG_GRACE_S=60             # NetworkManager's own time to reconnect
#WIFI_WATCHDOG_RETRY_S=60             # nmcli device connect, repeated while down
#WIFI_WATCHDOG_NM_RESTART_S=600       # restart NetworkManager, repeated while down
#WIFI_WATCHDOG_DRIVER_RELOAD_S=1200   # reload the WiFi driver, repeated while down
EOF
    echo "Wrote ${DEFAULTS_PATH}"
fi
as_root systemctl daemon-reload
as_root systemctl enable "${UNIT_NAME}" >/dev/null
as_root systemctl restart "${UNIT_NAME}"
sleep 1
systemctl --no-pager --lines=3 status "${UNIT_NAME}" || true

cat <<EOF

  logs:   journalctl -u rover-wifi-watchdog -f
  remove: ./scripts/install_wifi_watchdog.sh --uninstall
EOF
