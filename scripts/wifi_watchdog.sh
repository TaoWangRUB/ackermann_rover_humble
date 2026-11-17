#!/usr/bin/env bash
# Keep the rover's Jetson on WiFi, and with it on the tailnet, while nobody is
# logged in. Installed as /usr/local/sbin/rover-wifi-watchdog and run as root
# by rover-wifi-watchdog.service (scripts/install_wifi_watchdog.sh; re-run it
# after changing this file, the service runs the installed copy).
#
# NetworkManager 1.22 (Ubuntu 20.04, L4T R35) gives up on a WiFi network for
# good after one failed WPA handshake: it asks a secret agent for a new
# password, the headless rover has none ("no secrets: No agents were available
# for this request"), and auto-connect stays blocked until someone activates
# the network by hand. Seen 2026-11-16 14:35: the router's 5 GHz radio dropped
# out (beacon loss on DFS channel 64) and came back refusing the first
# handshake (deauth PREV_AUTH_NOT_VALID). The rover stayed offline until it
# was power-cycled 17 h later.
#
# While the WiFi device is not connected, NetworkManager first gets GRACE_S to
# reconnect on its own. Then the watchdog runs `nmcli device connect`, which
# activates the best saved network in range and lifts the block, and repeats
# it every RETRY_S. If the device stays down, it restarts NetworkManager
# (every NM_RESTART_S) and reloads the WiFi driver (every DRIVER_RELOAD_S).
# It leaves the device alone while the WiFi radio is switched off in
# NetworkManager (`nmcli radio wifi off`) or the device is unmanaged.
#
# Settings (WIFI_WATCHDOG_*): /etc/default/rover-wifi-watchdog, optional.
set -uo pipefail

IFACE="${WIFI_WATCHDOG_IFACE:-}"
CHECK_S="${WIFI_WATCHDOG_CHECK_S:-15}"
GRACE_S="${WIFI_WATCHDOG_GRACE_S:-60}"
RETRY_S="${WIFI_WATCHDOG_RETRY_S:-60}"
NM_RESTART_S="${WIFI_WATCHDOG_NM_RESTART_S:-600}"
DRIVER_RELOAD_S="${WIFI_WATCHDOG_DRIVER_RELOAD_S:-1200}"

log() { echo "[wifi-watchdog] $*"; }
# nmcli --wait clears its progress line (ESC[2K, CR) even into a pipe, which
# makes journald store the line as a blob that `journalctl` hides: strip it.
indent() { sed -e 's/\x1b\[[0-9;]*[A-Za-z]//g' -e 's/\r//g' -e 's/^/[wifi-watchdog]   /'; }
# Seconds since boot: the Xavier has no RTC battery, so the wall clock jumps by
# hours when NTP first syncs.
uptime_s() { cut -d. -f1 /proc/uptime; }

first_wifi_device() { nmcli -t -f DEVICE,TYPE device 2>/dev/null | awk -F: '$2 == "wifi" { print $1; exit }'; }
device_state() { nmcli -t -f DEVICE,STATE device 2>/dev/null | awk -F: -v d="$1" '$1 == d { print $2; exit }'; }
active_connection() { nmcli -t -f GENERAL.CONNECTION device show "$1" 2>/dev/null | cut -d: -f2-; }
driver_module() {  # empty for a missing device or a built-in driver
    local link="/sys/class/net/$1/device/driver/module"
    [[ -e "${link}" ]] && basename "$(readlink -f "${link}")"
}
# Empty output (NetworkManager not answering) counts as on, so the escalation
# that restarts NetworkManager still runs.
radio_off() { [[ "$(nmcli radio wifi 2>/dev/null)" == disabled ]]; }
# due LAST INTERVAL: LAST (uptime) is unset (-1) or INTERVAL seconds ago.
due() { (( $1 < 0 || now - $1 >= $2 )); }

[[ -n "${IFACE}" ]] || IFACE="$(first_wifi_device)"
IFACE="${IFACE:-wlan0}"
# Remembered, so the driver can be reloaded even after the device vanished.
driver="$(driver_module "${IFACE}")"
log "watching ${IFACE} (driver ${driver:-unknown}): connect after ${GRACE_S} s down and every ${RETRY_S} s," \
    "restart NetworkManager every ${NM_RESTART_S} s, reload the driver every ${DRIVER_RELOAD_S} s"

down_since=-1      # uptime when the device was first seen down; -1 while up
last_connect=-1
last_nm_restart=-1
last_reload=-1
idle_reason=""     # why the device is left alone, logged once

while true; do
    now="$(uptime_s)"
    state="$(device_state "${IFACE}")"
    module="$(driver_module "${IFACE}")"
    [[ -n "${module}" ]] && driver="${module}"

    if [[ "${state}" == connected ]]; then
        if (( down_since >= 0 )); then
            log "${IFACE} connected to $(active_connection "${IFACE}") after $(( now - down_since )) s down"
        fi
        down_since=-1; last_connect=-1; last_nm_restart=-1; last_reload=-1; idle_reason=""
    elif radio_off || [[ "${state}" == unmanaged ]]; then
        reason="the WiFi radio is switched off in NetworkManager"
        radio_off || reason="${IFACE} is unmanaged"
        [[ "${reason}" == "${idle_reason}" ]] || log "${reason}: leaving it alone"
        idle_reason="${reason}"; down_since=-1
    else
        idle_reason=""
        if (( down_since < 0 )); then
            down_since="${now}"
            log "${IFACE} is ${state:-not reported (NetworkManager not answering)};" \
                "NetworkManager gets ${GRACE_S} s to reconnect"
        fi
        down=$(( now - down_since ))
        if (( down >= DRIVER_RELOAD_S )) && due "${last_reload}" "${DRIVER_RELOAD_S}" && [[ -n "${driver}" ]]; then
            log "down for ${down} s: reloading the WiFi driver (${driver})"
            { modprobe -r "${driver}" && sleep 2 && modprobe "${driver}"; } 2>&1 | indent
            # Give the fresh device a full retry interval before the next step.
            last_reload="${now}"; last_nm_restart="${now}"; last_connect="${now}"
        elif (( down >= NM_RESTART_S )) && due "${last_nm_restart}" "${NM_RESTART_S}"; then
            log "down for ${down} s: restarting NetworkManager"
            systemctl restart NetworkManager 2>&1 | indent
            last_nm_restart="${now}"; last_connect="${now}"
        elif (( down >= GRACE_S )) && due "${last_connect}" "${RETRY_S}"; then
            log "down for ${down} s (${state:-?}): nmcli device connect ${IFACE}"
            nmcli --wait 45 device connect "${IFACE}" 2>&1 | indent
            last_connect="$(uptime_s)"
        fi
    fi
    sleep "${CHECK_S}"
done
