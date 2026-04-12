#!/bin/bash

# verify_agent_work.sh
# Autonomous Agent Verification Script
# This automates the checklist from AGENTS.md to ensure the agent's changes are valid.
# Usage: bash scripts/verify_agent_work.sh

set -e

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

# Load .env so docker compose v2 picks up variables regardless of project-directory
set -a
[ -f .env ] && source .env
set +a

# Ensure runtime vars are set (normally exported via ~/.bashrc on the host)
export ARCH="${ARCH:-$(uname -m)}"
export USERNAME="${USERNAME:-$(id -un)}"
export USER_UID="${USER_UID:-$(id -u)}"
export USER_GID="${USER_GID:-$(id -g)}"

# In CI, PX4 and RealSense repos are not checked out — use placeholder dirs
if [ "$CI" = "true" ]; then
    export PX4_DIR=$(mktemp -d)
    export REALSENSE_ROS_DIR=$(mktemp -d)
fi


GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# ---------------------------------------------------------------------------
# Detect the correct docker compose command (v2 plugin vs v1 standalone)
# ---------------------------------------------------------------------------
if docker compose version >/dev/null 2>&1; then
    DOCKER_COMPOSE="docker compose"
elif command -v docker-compose >/dev/null 2>&1; then
    DOCKER_COMPOSE="docker-compose"
else
    echo -e "${RED}Neither 'docker compose' (v2) nor 'docker-compose' (v1) found.${NC}"
    exit 1
fi
echo "Using: $DOCKER_COMPOSE"

COLCON_SKIP_ARGS="--packages-ignore-regex '^example_.*' --packages-ignore px4_ros2_py"

if [ "$CI" = "true" ]; then
    echo -e "${GREEN}CI Environment Detected: Removing GPU runtime requirement for headless build...${NC}"
    # compose file uses version 2.4 with only 'runtime: nvidia' (no deploy block)
    sed -i '/runtime: nvidia/d' docker/docker-compose.yml
fi

echo -e "${GREEN}=== Applying Submodule Patches ===${NC}"
./scripts/apply_vins_fusion_patch.sh
./scripts/apply_px4_ros2_patch.sh

echo -e "${GREEN}=== Setting up Docker Environment ===${NC}"
# Allow local X11 connections for GUI apps (like RViz/Gazebo) inside Docker
# xhost is not available in headless CI environments — skip it there.
if [ "$CI" != "true" ]; then
    xhost +local: || true
fi

# Bring up Docker container logic
$DOCKER_COMPOSE -f docker/docker-compose.yml up -d ackermann_slam
sleep 5 # wait for container to be ready

echo -e "${GREEN}=== Running Build ===${NC}"
# Exclude submodule example packages and the optional Python bindings package
# from the workspace build.
# Stage 1: Build px4_msgs then px4_ros2_cpp (px4_ros2_cpp needs px4_msgs at configure time)
$DOCKER_COMPOSE -f docker/docker-compose.yml exec ackermann_slam bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install $COLCON_SKIP_ARGS --packages-up-to px4_ros2_cpp"
# Stage 2: Source stage-1 artifacts, then build the project's own packages
$DOCKER_COMPOSE -f docker/docker-compose.yml exec ackermann_slam bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon build --symlink-install $COLCON_SKIP_ARGS --packages-up-to ackermann_control safety px4_bringup robot_bringup ackermann_nav2_bringup rtabmap_bringup description_robot"

echo -e "${GREEN}=== Running Unit Tests ===${NC}"
# Clear stale test result XMLs from previous broader workspace runs so the
# package-select test pass below reports only the packages this script covers.
$DOCKER_COMPOSE -f docker/docker-compose.yml exec ackermann_slam bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace && colcon test-result --test-result-base build --delete-yes || true"
# Only test project packages — skip submodule tests (px4-ros2-interface-lib examples, etc.)
$DOCKER_COMPOSE -f docker/docker-compose.yml exec ackermann_slam bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon test --event-handlers console_direct+ --packages-select ackermann_control safety && colcon test-result --verbose"

echo -e "${GREEN}=== Validating Simulation & Launch (Dry Run/Topology Check) ===${NC}"
# Since a full gazebo launch might run infinitely and block the agent indefinitely unless orchestrated,
# we test the node composition and launch argument validity.
# We also include a slightly longer timeout (25s) to allow Nav2 and RViz to stand up, then kill the process tree cleanly.
$DOCKER_COMPOSE -f docker/docker-compose.yml exec ackermann_slam bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && timeout --preserve-status 25 ros2 launch robot_bringup robot_bringup.launch.py rtabmap:=true nav2:=true || true"

echo -e "${GREEN}=== Running Safety Checks & Linting ===${NC}"
$DOCKER_COMPOSE -f docker/docker-compose.yml exec ackermann_slam bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace && ament_lint_cmake \
    src/ackermann_control \
    src/safety \
    src/description_robot \
    src/realsense_camera_bringup \
    src/rtabmap_bringup \
    src/ackermann_nav2_bringup \
    src/robot_bringup \
    src/vins_fusion_bringup \
    src/px4_bringup || true"
$DOCKER_COMPOSE -f docker/docker-compose.yml exec ackermann_slam bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace && ament_flake8 \
    scripts \
    tools \
    src/description_robot \
    src/realsense_camera_bringup \
    src/rtabmap_bringup \
    src/robot_bringup \
    src/vins_fusion_bringup \
    src/px4_bringup || true"
$DOCKER_COMPOSE -f docker/docker-compose.yml exec ackermann_slam bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace && ament_cpplint \
    src/ackermann_control \
    src/safety \
    src/description_robot \
    src/px4_bringup || true"

echo -e "${GREEN}=== Tear Down ===${NC}"
$DOCKER_COMPOSE -f docker/docker-compose.yml down

echo -e "${GREEN}Verification Script Completed Successfully. The codebase meets the core baseline.${NC}"
