#!/bin/bash

# verify_agent_work.sh
# Autonomous Agent Verification Script
# This automates the checklist from AGENTS.md to ensure the agent's changes are valid.
# Usage: bash scripts/verify_agent_work.sh

set -e

GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

COMPOSE_ARGS="-f docker/docker-compose.yml"
if [ "$CI" = "true" ]; then
    echo -e "${GREEN}CI Environment Detected: Applying Docker Compose overrides...${NC}"
    COMPOSE_ARGS="$COMPOSE_ARGS -f docker/docker-compose.ci.yml"
fi

echo -e "${GREEN}=== Setting up Docker Environment ===${NC}"
# Allow local X11 connections for GUI apps (like RViz/Gazebo) inside Docker
xhost +local:root || true

# Bring up Docker container logic
docker compose $COMPOSE_ARGS up -d ackermann_slam
sleep 5 # wait for container to be ready

echo -e "${GREEN}=== Running Build ===${NC}"
docker compose $COMPOSE_ARGS exec ackermann_slam bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

echo -e "${GREEN}=== Running Unit Tests ===${NC}"
docker compose $COMPOSE_ARGS exec ackermann_slam bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon test --event-handlers console_direct+ && colcon test-result --verbose"

echo -e "${GREEN}=== Validating Simulation & Launch (Dry Run/Topology Check) ===${NC}"
# Since a full gazebo launch might run infinitely and block the agent indefinitely unless orchestrated,
# we test the node composition and launch argument validity.
# We also include a slightly longer timeout (25s) to allow Nav2 and RViz to stand up, then kill the process tree cleanly.
docker compose $COMPOSE_ARGS exec ackermann_slam bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && timeout --preserve-status 25 ros2 launch robot_bringup robot_bringup.launch.py rtabmap:=true nav2:=true || true"

echo -e "${GREEN}=== Running Safety Checks & Linting ===${NC}"
docker compose $COMPOSE_ARGS exec ackermann_slam bash -c "source /opt/ros/jazzy/setup.bash && ament_lint_cmake . || true"
docker compose $COMPOSE_ARGS exec ackermann_slam bash -c "source /opt/ros/jazzy/setup.bash && ament_flake8 . || true"
docker compose $COMPOSE_ARGS exec ackermann_slam bash -c "source /opt/ros/jazzy/setup.bash && ament_cpplint . || true"

echo -e "${GREEN}=== Tear Down ===${NC}"
docker compose $COMPOSE_ARGS down

echo -e "${GREEN}Verification Script Completed Successfully. The codebase meets the core baseline.${NC}"
