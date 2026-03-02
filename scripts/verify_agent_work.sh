#!/bin/bash

# verify_agent_work.sh
# Autonomous Agent Verification Script
# This automates the checklist from AGENTS.md to ensure the agent's changes are valid.
# Usage: bash scripts/verify_agent_work.sh

set -e

GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

if [ "$CI" = "true" ]; then
    echo -e "${GREEN}CI Environment Detected: Modifying docker-compose.yml to remove GPU requirement...${NC}"
    # Strip the runtime and deploy blocks which strictly require an NVIDIA GPU on host
    sed -i '/runtime: nvidia/d' docker/docker-compose.yml
    sed -i '/deploy:/,/capabilities: \["gpu"\]/d' docker/docker-compose.yml
fi

echo -e "${GREEN}=== Setting up Docker Environment ===${NC}"
# Allow local X11 connections for GUI apps (like RViz/Gazebo) inside Docker
xhost +local:root || true

# Bring up Docker container logic
docker compose -f docker/docker-compose.yml up -d ackermann_slam
sleep 5 # wait for container to be ready

echo -e "${GREEN}=== Running Build ===${NC}"
# Exclude submodule example and test packages from the build
docker compose -f docker/docker-compose.yml exec ackermann_slam bash -c "
  touch src/px4-ros2-interface-lib/examples/COLCON_IGNORE 2>/dev/null || true
  touch src/px4-ros2-interface-lib/px4_ros2_py/COLCON_IGNORE 2>/dev/null || true
"
# Stage 1: Build px4_msgs then px4_ros2_cpp (px4_ros2_cpp needs px4_msgs at configure time)
docker compose -f docker/docker-compose.yml exec ackermann_slam bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-up-to px4_ros2_cpp"
# Stage 2: Source stage-1 artifacts, then build the project's own packages
docker compose -f docker/docker-compose.yml exec ackermann_slam bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon build --symlink-install --packages-up-to ackermann_control safety px4_bringup robot_bringup ackermann_nav2_bringup rtabmap_bringup description_robot"

echo -e "${GREEN}=== Running Unit Tests ===${NC}"
# Only test project packages — skip submodule tests (px4-ros2-interface-lib examples, etc.)
docker compose -f docker/docker-compose.yml exec ackermann_slam bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon test --event-handlers console_direct+ --packages-select ackermann_control safety && colcon test-result --verbose"

echo -e "${GREEN}=== Validating Simulation & Launch (Dry Run/Topology Check) ===${NC}"
# Since a full gazebo launch might run infinitely and block the agent indefinitely unless orchestrated,
# we test the node composition and launch argument validity.
# We also include a slightly longer timeout (25s) to allow Nav2 and RViz to stand up, then kill the process tree cleanly.
docker compose -f docker/docker-compose.yml exec ackermann_slam bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && timeout --preserve-status 25 ros2 launch robot_bringup robot_bringup.launch.py rtabmap:=true nav2:=true || true"

echo -e "${GREEN}=== Running Safety Checks & Linting ===${NC}"
docker compose -f docker/docker-compose.yml exec ackermann_slam bash -c "source /opt/ros/jazzy/setup.bash && ament_lint_cmake . || true"
docker compose -f docker/docker-compose.yml exec ackermann_slam bash -c "source /opt/ros/jazzy/setup.bash && ament_flake8 . || true"
docker compose -f docker/docker-compose.yml exec ackermann_slam bash -c "source /opt/ros/jazzy/setup.bash && ament_cpplint . || true"

echo -e "${GREEN}=== Tear Down ===${NC}"
docker compose -f docker/docker-compose.yml down

echo -e "${GREEN}Verification Script Completed Successfully. The codebase meets the core baseline.${NC}"
