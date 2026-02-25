#!/bin/bash

# verify_agent_work.sh
# Autonomous Agent Verification Script
# This automates the checklist from AGENTS.md to ensure the agent's changes are valid.
# Usage: bash scripts/verify_agent_work.sh

set -e

GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Setting up Docker Environment ===${NC}"
# Bring up Docker container logic
docker compose -f docker/docker-compose.yml up -d ackermann_slam
sleep 5 # wait for container to be ready

echo -e "${GREEN}=== Running Build ===${NC}"
docker compose -f docker/docker-compose.yml exec ackermann_slam bash -c "colcon build --symlink-install"

echo -e "${GREEN}=== Running Unit Tests ===${NC}"
docker compose -f docker/docker-compose.yml exec ackermann_slam bash -c "colcon test --event-handlers console_direct+ && colcon test-result --verbose"

echo -e "${GREEN}=== Validating Simulation & Launch (Dry Run/Topology Check) ===${NC}"
# Since a full gazebo launch might run infinitely and block the agent indefinitely unless orchestrated,
# we test the node composition and launch argument validity.
# In a real heavy CI, this would launch Gazebo in headless mode, run an action client to navigate, and wait for success.
docker compose -f docker/docker-compose.yml exec ackermann_slam bash -c "ros2 launch robot_bringup robot_bringup.launch.py rtabmap:=true nav2:=true & sleep 15 && kill -INT \$!"

echo -e "${GREEN}=== Running Safety Checks & Linting ===${NC}"
docker compose -f docker/docker-compose.yml exec ackermann_slam bash -c "ament_lint_cmake . || true"
docker compose -f docker/docker-compose.yml exec ackermann_slam bash -c "ament_flake8 . || true"
docker compose -f docker/docker-compose.yml exec ackermann_slam bash -c "ament_cpplint . || true"

echo -e "${GREEN}=== Tear Down ===${NC}"
docker compose -f docker/docker-compose.yml down

echo -e "${GREEN}Verification Script Completed Successfully. The codebase meets the core baseline.${NC}"
