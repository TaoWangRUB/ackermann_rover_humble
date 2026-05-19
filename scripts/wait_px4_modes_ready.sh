#!/usr/bin/env bash
# Wait until the expected number of PX4 custom rover mode nodes are running
# and PX4 pre-flight checks pass.
#
# Usage: ./scripts/wait_px4_modes_ready.sh [EXPECTED_MODES] [TIMEOUT_S]
# Default: 4 modes, 120s timeout
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"

EXPECTED_MODES="${1:-4}"
TIMEOUT_S="${2:-120}"
DEADLINE=$((SECONDS + TIMEOUT_S))

ROS_SOURCE="source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash"

while (( SECONDS < DEADLINE )); do
    node_count="$({ dcomp exec -T ackermann_slam bash -lc "${ROS_SOURCE} && ros2 node list 2>/dev/null | grep -Ec '^/(rover_manual_mode|rover_speed_attitude_mode|rover_speed_rate_mode|rover_speed_steering_mode)$'"; } 2>/dev/null | tr -d '[:space:]')"
    preflight="$({ dcomp exec -T ackermann_slam bash -lc "${ROS_SOURCE} && ros2 topic echo /fmu/out/vehicle_status_v2 --once 2>/dev/null | awk '/pre_flight_checks_pass:/ { print \$2; exit }'"; } 2>/dev/null | tr -d '[:space:]')"

    node_count="${node_count:-0}"
    preflight="${preflight:-unknown}"

    echo "Waiting for PX4 modes: nodes=${node_count}/${EXPECTED_MODES} preflight=${preflight}"

    if [[ "${node_count}" -ge "${EXPECTED_MODES}" && "${preflight}" == "true" ]]; then
        echo "All ${EXPECTED_MODES} mode nodes running and preflight passed."
        exit 0
    fi

    sleep 3
done

echo "Timed out waiting for PX4 modes to become ready" >&2
exit 1
