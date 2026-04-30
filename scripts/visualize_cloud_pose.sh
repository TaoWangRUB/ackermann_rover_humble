#!/usr/bin/env bash
# Visualize .ply point clouds and pose .txt files inside the running Docker container.
#
# Usage:
#   ./scripts/visualize_cloud_pose.sh cloud.ply poses.txt
#   ./scripts/visualize_cloud_pose.sh map.ply traj.txt --frame-step 10 --z-up
set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"

if [[ -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null 2>&1; then
    xhost +local: >/dev/null 2>&1 || true
fi

if ! dcomp ps --services --filter status=running 2>/dev/null \
        | grep -q '^ackermann_slam$'; then
    echo "Container not running — starting ackermann_slam..."
    dcomp up -d ackermann_slam
fi

if [ -t 0 ]; then
    xdcomp exec ackermann_slam \
        bash -lc 'source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash && python3 /workspace/scripts/visualize_cloud_pose.py "$@"' \
        bash "$@"
else
    xdcomp exec -T ackermann_slam \
        bash -lc 'source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash && python3 /workspace/scripts/visualize_cloud_pose.py "$@"' \
        bash "$@"
fi
