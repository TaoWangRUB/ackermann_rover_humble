#!/bin/bash
set -e

# Base ROS 2 distro (default to jazzy if not set)
: "${ROS_DISTRO:=jazzy}"

# Update apt index and install ROS package dependencies via rosdep
sudo apt-get update -qq
source "/opt/ros/${ROS_DISTRO}/setup.bash"
rosdep install --from-paths src --ignore-src -r -y || \
  echo "WARNING: rosdep install encountered errors; continuing startup."

# Source overlay workspace if it has been built
if [ -f /workspace/install/setup.bash ]; then
  source /workspace/install/setup.bash
fi

# Hand off to the requested command (bash by default)
exec "$@"
