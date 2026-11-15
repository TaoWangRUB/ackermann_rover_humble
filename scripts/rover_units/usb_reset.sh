#!/usr/bin/env bash
# Re-enumerate a USB device by VID:PID (a software unplug/replug). Needs root:
# rover-xrce.service runs it as `ExecStartPre=+...` before the XRCE agent opens
# the CH340 adapter, which can wedge after a hard stop
# (kernel: `ch341-uart ttyUSB0: failed to read modem status: -110`, agent:
# `open device error ... errno: 5`; seen 2026-11-15).
#
#   usb_reset.sh VID:PID
set -euo pipefail

vidpid="${1:?usage: usb_reset.sh VID:PID}"
vid="${vidpid%%:*}"
pid="${vidpid##*:}"
found=false
for dev in /sys/bus/usb/devices/*; do
    [[ -f "${dev}/idVendor" && -f "${dev}/authorized" ]] || continue
    if [[ "$(cat "${dev}/idVendor")" == "${vid}" && "$(cat "${dev}/idProduct")" == "${pid}" ]]; then
        echo "usb_reset: re-enumerating ${vidpid} at ${dev}"
        echo 0 > "${dev}/authorized"
        sleep 1
        echo 1 > "${dev}/authorized"
        found=true
    fi
done
[[ "${found}" == true ]] || echo "usb_reset: ${vidpid} not present, nothing to reset"
exit 0
