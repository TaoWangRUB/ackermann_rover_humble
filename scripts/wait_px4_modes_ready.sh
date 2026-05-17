#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"

EXPECTED_MODES="${1:-4}"
TIMEOUT_S="${2:-120}"
DEADLINE=$((SECONDS + TIMEOUT_S))

while (( SECONDS < DEADLINE )); do
    mode_count="$({ dcomp exec -T ackermann_slam bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && timeout 3s ros2 topic echo /px4_modes/announce 2>/dev/null | awk '/^data: / { sub(/^data: /, \"\"); print }' | sort -u | wc -l"; } 2>/dev/null | tr -d '[:space:]')"
    node_count="$({ dcomp exec -T ackermann_slam bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && ros2 node list 2>/dev/null | grep -Ec '^/(rover_manual_mode|rover_speed_attitude_mode|rover_speed_rate_mode|rover_speed_steering_mode)$'"; } 2>/dev/null | tr -d '[:space:]')"
    preflight="$({ dcomp exec -T ackermann_slam bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && ros2 topic echo /fmu/out/vehicle_status_v2 --once 2>/dev/null | awk '/pre_flight_checks_pass:/ { print \$2; exit }'"; } 2>/dev/null | tr -d '[:space:]')"

    mode_count="${mode_count:-0}"
    node_count="${node_count:-0}"
    preflight="${preflight:-unknown}"

    echo "Waiting for PX4 modes: nodes=${node_count}/${EXPECTED_MODES} announcements=${mode_count}/${EXPECTED_MODES} preflight=${preflight}"

    if [[ "${node_count}" -ge "${EXPECTED_MODES}" && "${mode_count}" -ge "${EXPECTED_MODES}" && "${preflight}" == "true" ]]; then
        exit 0
    fi

    sleep 2
done

echo "Timed out waiting for PX4 modes to become selectable" >&2
exit 1