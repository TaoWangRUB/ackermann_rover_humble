#!/usr/bin/env bash
# Verify odometry alignment across all sources (EKF, T265, PX4).
# Polls each odom topic once and prints positions side-by-side.
# For a stationary robot, X-Y should agree within ~1 cm.
#
# Usage:
#   ./scripts/verify_odom.sh              # one-shot
#   ./scripts/verify_odom.sh --loop       # repeat every 5s
#   ./scripts/verify_odom.sh --loop 10    # repeat every 10s
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="$(dirname "$SCRIPT_DIR")/docker/docker-compose.yml"
DC="docker-compose -f ${COMPOSE_FILE}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

LOOP=false
INTERVAL=5
if [[ "${1:-}" == "--loop" ]]; then
    LOOP=true
    INTERVAL="${2:-5}"
fi

SOURCE="source /opt/ros/\$ROS_DISTRO/setup.bash && source /workspace/install/setup.bash 2>/dev/null"

print_odom() {
    local label="$1" topic="$2"
    local result
    result=$(${DC} exec -T ackermann_slam bash -c "
        ${SOURCE}
        timeout 3 ros2 topic echo ${topic} --once 2>/dev/null | grep -A3 'position:' | head -4
    " 2>/dev/null) || true

    if [[ -z "$result" ]]; then
        printf "  %-30s  (no data)\n" "${label}"
    else
        local x y z
        x=$(echo "$result" | grep 'x:' | head -1 | awk '{print $2}')
        y=$(echo "$result" | grep 'y:' | head -1 | awk '{print $2}')
        z=$(echo "$result" | grep 'z:' | head -1 | awk '{print $2}')
        printf "  %-30s  X: %8s  Y: %8s  Z: %8s\n" "${label}" "$x" "$y" "$z"
    fi
}

check_topics() {
    echo "── Active odom topics ──"
    local topics
    topics=$(${DC} exec -T ackermann_slam bash -c "
        ${SOURCE}
        ros2 topic list 2>/dev/null | grep -E 'odometry/filtered|t265/odom_base|px4_vehicle_odom_base' || true
    " 2>/dev/null) || true
    if [[ -z "$topics" ]]; then
        echo "  (none found)"
    else
        echo "$topics" | sed 's/^/  /'
    fi
    echo ""
}

run_check() {
    echo "══════════════════════════════════════════════════════"
    echo "  Odometry Verification  $(date '+%H:%M:%S')"
    echo "══════════════════════════════════════════════════════"
    check_topics
    echo "── Positions ──"
    print_odom "/odometry/filtered (EKF)"       "/odometry/filtered"
    print_odom "/t265/odom_base (T265)"          "/t265/odom_base"
    print_odom "/px4_vehicle_odom_base (PX4)"    "/px4_vehicle_odom_base"
    echo ""
    echo "X-Y should agree within ~1 cm for a stationary robot."
    echo "Differences > 8 cm in X suggest a frame transform bug."
    echo ""
}

if [[ "$LOOP" == true ]]; then
    echo "Polling every ${INTERVAL}s. Press Ctrl+C to stop."
    echo ""
    while true; do
        run_check
        sleep "$INTERVAL"
    done
else
    run_check
fi
