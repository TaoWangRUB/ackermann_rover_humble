#!/usr/bin/env bash
# Start PX4 SITL inside the Docker container.
#
# Usage (from host):
#   ./scripts/start_px4_sitl.sh               # custom ackermann model (standalone, needs Gazebo)
#   ./scripts/start_px4_sitl.sh --official    # official gz_rover_ackermann (PX4 spawns Gazebo itself)
#   ./scripts/start_px4_sitl.sh build         # rebuild PX4 inside Docker first
#
# --- Default mode (custom model, standalone) ---
# Prerequisites:
#   1. Docker container running (docker-compose up -d)
#   2. Gazebo already running (robot_bringup with enable_px4_sitl:=true)
#   3. PX4-Autopilot mounted read-write at /px4 via docker-compose.yml
#   4. MicroXRCEAgent running (./scripts/start_microxrce_agent.sh)
#
# Environment overrides:
#   PX4_MODEL_NAME  Gazebo model to bind to  (default: ackermann)
#   PX4_AUTOSTART   Airframe ID              (default: 51000)
#   PX4_GZ_WORLD    Gazebo world name        (default: warehouse)
#
# --- Official mode (--official) ---
# Prerequisites:
#   1. Docker container running (docker-compose up -d)
#   2. No prior Gazebo needed — PX4 spawns Gazebo + model automatically.
#   3. PX4-Autopilot mounted read-write at /px4 via docker-compose.yml
#   4. MicroXRCEAgent running (./scripts/start_microxrce_agent.sh)
#
# Environment overrides:
#   PX4_GZ_WORLD    Gazebo world name        (default: rover)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${PROJECT_DIR}/docker/docker-compose.yml"

PX4_BUILD_DIR="/px4/build/px4_sitl_default"

# --- Parse flags ---
OFFICIAL_MODEL=0
POSITIONAL=()
for arg in "$@"; do
  case "$arg" in
    --official) OFFICIAL_MODEL=1 ;;
    *) POSITIONAL+=("$arg") ;;
  esac
done
set -- "${POSITIONAL[@]+"${POSITIONAL[@]}"}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

# --- Build subcommand ---
if [[ "${1:-}" == "build" ]]; then
    echo "Building PX4 SITL inside Docker container..."
    exec docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -c "
      git config --global --add safe.directory '*'
      export GZ_DISTRO=harmonic
      cd /px4 && make px4_sitl_default
    "
fi

# --- Mode-specific config ---
if [[ "${OFFICIAL_MODEL}" == "1" ]]; then
    # Official gz_rover_ackermann: PX4 resolves airframe via PX4_SIM_MODEL (-> 51000),
    # spawns Gazebo itself, and loads the model from its own resource paths.
    # Mirrors exactly what `make px4_sitl gz_rover_ackermann` does.
    PX4_GZ_WORLD="${PX4_GZ_WORLD:-rover}"
    MODE_LABEL="official gz_rover_ackermann (PX4-managed Gazebo, world: ${PX4_GZ_WORLD})"

    echo "Starting PX4 SITL (official mode) inside Docker container..."
    echo "  Mode:          ${MODE_LABEL}"
    echo "  PX4 build dir: ${PX4_BUILD_DIR} (inside container)"
    echo ""

    exec docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -c "
      git config --global --add safe.directory '*'
      # Use rootfs/ as working dir — matches SITL_WORKING_DIR used by the make target.
      cd ${PX4_BUILD_DIR}/rootfs
      rm -f lockfile

      # Mirror the exact env the CMake-generated make target sets (no gz_env.sh sourcing —
      # px4-rc.gzsim sets GZ_SIM_RESOURCE_PATH internally from PX4_GZ_MODELS/WORLDS/PLUGINS).
      export PX4_SIM_MODEL=gz_rover_ackermann
      export PX4_GZ_WORLD=${PX4_GZ_WORLD}
      export GZ_IP=127.0.0.1
      export GZ_DISTRO=harmonic

      # Run px4 in foreground with a real TTY — matches `make px4_sitl gz_rover_ackermann`
      # (CMake USES_TERMINAL). This gives you an interactive pxh> shell.
      #
      # Once initialized, apply rover overrides from the pxh> prompt:
      #   param set COM_RC_IN_MODE 4
      #   commander mode auto:loiter
      #   commander set_ekf_origin 51.4934 7.4120 100.0
      #   commander set_home
      ${PX4_BUILD_DIR}/bin/px4
    "
else
    # Custom model: connect to an already-running Gazebo instance (standalone mode).
    PX4_MODEL_NAME="${PX4_MODEL_NAME:-ackermann}"
    PX4_AUTOSTART="${PX4_AUTOSTART:-51000}"
    PX4_GZ_WORLD="${PX4_GZ_WORLD:-warehouse}"
    MODE_LABEL="custom (model: ${PX4_MODEL_NAME}, standalone)"

    echo "Starting PX4 SITL (custom model mode) inside Docker container..."
    echo "  Mode:          ${MODE_LABEL}"
    echo "  Airframe:      ${PX4_AUTOSTART}"
    echo "  World:         ${PX4_GZ_WORLD}"
    echo "  PX4 build dir: ${PX4_BUILD_DIR} (inside container)"
    echo ""

    exec docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -c "
      git config --global --add safe.directory '*'
      # rootfs/ is the working dir PX4 expects (matches SITL_WORKING_DIR).
      cd ${PX4_BUILD_DIR}/rootfs
      rm -f lockfile
      export PX4_SYS_AUTOSTART=${PX4_AUTOSTART}
      export PX4_GZ_STANDALONE=1
      export PX4_GZ_MODEL_NAME=${PX4_MODEL_NAME}
      export PX4_GZ_WORLD=${PX4_GZ_WORLD}
      export PX4_SIMULATOR=gz
      export GZ_DISTRO=harmonic

      # Run px4 in foreground so pxh> shell is fully interactive.
      #
      # Once initialized, apply rover overrides from the pxh> prompt:
      #   param set COM_RC_IN_MODE 4
      #   commander mode auto:loiter
      #   commander set_ekf_origin 51.4934 7.4120 100.0
      #   commander set_home
      #
      # === Actuator Test Notes ===
      # Rover must be DISARMED before actuator_test — armed Hold mode sends
      # zero-velocity commands that override test output. From pxh>:
      #   commander disarm
      #   actuator_test set -m 1 -v 0.5 -t 3   # motor test
      #   actuator_test set -s 1 -v 0.5 -t 3   # servo test
      #
      # To drive via ROS 2 /cmd_vel, launch px4_bridge after this script:
      #   ros2 launch px4_bringup px4_bringup.launch.py mode_type:=speed_steering
      ${PX4_BUILD_DIR}/bin/px4 -s ${PX4_BUILD_DIR}/etc/init.d-posix/rcS ${PX4_BUILD_DIR}/etc
    "
fi
