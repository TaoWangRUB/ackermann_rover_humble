#!/usr/bin/env bash
# One line per rover unit (units mode): state and the last readiness-gate or
# error message from its journal, in startup order.
#
#   scripts/rover_units/status.sh
set -uo pipefail

units=(rover-container rover-usb rover-base rover-t265 rover-d435i rover-xrce
       rover-mavlink rover-px4-hw-monitor rover-px4-bringup rover-monitor
       rover-slam rover-nav2 rover-recorder)

printf '%-21s %-9s %-13s %s\n' UNIT ACTIVE SUB "LAST GATE / ERROR"
for unit in "${units[@]}"; do
    active="$(systemctl show -p ActiveState --value "${unit}.service" 2>/dev/null)"
    sub="$(systemctl show -p SubState --value "${unit}.service" 2>/dev/null)"
    last="$(journalctl -u "${unit}.service" -b -n 200 --no-pager -o cat 2>/dev/null \
        | grep -E '\[wait_for|TIMEOUT|ERROR|usb_reset|Failed|killing leftover' | tail -1 | cut -c1-95)"
    printf '%-21s %-9s %-13s %s\n' "${unit}" "${active:-?}" "${sub:-?}" "${last}"
done
echo
echo "target: rover-stack.target $(systemctl show -p ActiveState --value rover-stack.target 2>/dev/null)"
