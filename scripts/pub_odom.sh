#!/usr/bin/env bash
# Publish a continuous mock Odometry message on /odometry/filtered with valid covariances
set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${PROJECT_DIR}/docker/docker-compose.yml"

echo "Publishing mock Odometry to /odometry/filtered at 10Hz inside Docker container..."

exec docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -c "
  source /opt/ros/jazzy/setup.bash
  source /workspace/install/setup.bash
  ros2 topic pub -r 10 /odometry/filtered nav_msgs/msg/Odometry '{
    header: {frame_id: '\''odom'\''},
    child_frame_id: '\''ackermann/base_link'\'',
    pose: {
      pose: {
        position: {x: 0.1, y: 0.2, z: 0.3},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      covariance: [0.01, 0, 0, 0, 0, 0,
                   0, 0.01, 0, 0, 0, 0,
                   0, 0, 0.01, 0, 0, 0,
                   0, 0, 0, 0.01, 0, 0,
                   0, 0, 0, 0, 0.01, 0,
                   0, 0, 0, 0, 0, 0.01]
    },
    twist: {
      twist: {
        linear: {x: 0.1, y: 0.0, z: 0.0},
        angular: {x: 0.0, y: 0.0, z: 0.0}
      },
      covariance: [0.01, 0, 0, 0, 0, 0,
                   0, 0.01, 0, 0, 0, 0,
                   0, 0, 0.01, 0, 0, 0,
                   0, 0, 0, 0.01, 0, 0,
                   0, 0, 0, 0, 0.01, 0,
                   0, 0, 0, 0, 0, 0.01]
    }
  }'
"
