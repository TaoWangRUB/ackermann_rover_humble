#!/usr/bin/env bash
# Stop all simulation and hardware ROS 2 processes.
# Kills inside the Docker container (docker exec, not docker-compose exec) and on the host.
# Covers: ROS 2 launch/run, Gazebo, PX4 SITL, MicroXRCE-DDS, RViz, all camera and slam nodes.
#
# Usage:
#   ./scripts/stop_all.sh

set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER="${CONTAINER_NAME:-jazzy_slam}"

# Matches all processes started by start_ros2_nodes.sh + PX4 sim processes
KILL_PATTERN='ros2 launch|ros2 run|rviz2|gz sim|gz_bridge|rtabmap|realsense_camera_node|ekf_node|imu_filter|imu_transformer|rgbd_sync|rgbd_odometry|robot_state_publisher|cmd_vel_joint_relay|odom_tf_relay|depthimage_to_laserscan|point_cloud_xyz|obstacles_detection|px4_bringup|MicroXRCEAgent|px4 -i|px4 -s'

echo "Stopping all simulation/hardware ROS 2 processes..."

# --- Docker container (uses docker exec — works without a TTY) ---
if docker inspect "${CONTAINER}" &>/dev/null 2>&1; then
    docker exec "${CONTAINER}" bash -c "
        PIDS=\$(ps aux | grep -E '${KILL_PATTERN}' | grep -v grep | awk '{print \$2}')
        if [ -n \"\$PIDS\" ]; then
            echo \"\$PIDS\" | xargs kill -9 2>/dev/null || true
            echo 'Killed container processes.'
        else
            echo 'No matching processes in container.'
        fi
    "
else
    echo "Container '${CONTAINER}' not running — skipping."
fi

# --- Host ---
HOST_PIDS=$(ps aux | grep -E "${KILL_PATTERN}" | grep -v grep | awk '{print $2}' || true)
if [ -n "${HOST_PIDS}" ]; then
    echo "${HOST_PIDS}" | xargs kill -9 2>/dev/null || true
    echo "Killed host processes."
else
    echo "No matching processes on host."
fi

sleep 1
echo "Done."
