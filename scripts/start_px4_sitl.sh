#!/usr/bin/env bash
# Start PX4 SITL in standalone mode inside the Docker container.
# PX4 connects to the already-running Gazebo instance.
#
# Usage (from host):
#   ./scripts/start_px4_sitl.sh          # run PX4 SITL
#   ./scripts/start_px4_sitl.sh build    # rebuild PX4 inside Docker first
#
# Prerequisites:
#   1. Docker container must be running (docker-compose up -d)
#   2. Gazebo must already be running (robot_bringup with enable_px4_sitl:=true)
#   3. PX4-Autopilot is mounted read-write at /px4 via docker-compose.yml
#
# Environment overrides:
#   PX4_MODEL_NAME Gazebo model to bind to     (default: ackermann)
#   PX4_AUTOSTART  Airframe ID                 (default: 4012 = gz_rover_ackermann)
#   PX4_GZ_WORLD   Gazebo world name           (default: warehouse)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${PROJECT_DIR}/docker/docker-compose.yml"

PX4_MODEL_NAME="${PX4_MODEL_NAME:-ackermann}"
PX4_AUTOSTART="${PX4_AUTOSTART:-51000}"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-warehouse}"
PX4_BUILD_DIR="/px4/build/px4_sitl_default"

# --- Build subcommand ---
if [[ "${1:-}" == "build" ]]; then
    echo "Building PX4 SITL inside Docker container..."
    exec docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -c "
      git config --global --add safe.directory '*'
      export GZ_DISTRO=harmonic
      cd /px4 && make px4_sitl_default
    "
fi

# --- Run PX4 SITL ---
echo "Starting PX4 SITL inside Docker container..."
echo "  Model name:         ${PX4_MODEL_NAME}"
echo "  Airframe:           ${PX4_AUTOSTART}"
echo "  World:              ${PX4_GZ_WORLD}"
echo "  PX4 build dir:      ${PX4_BUILD_DIR} (inside container)"
echo ""

exec docker-compose -f "${COMPOSE_FILE}" exec ackermann_slam bash -c "
  git config --global --add safe.directory '*'
  cd ${PX4_BUILD_DIR}
  rm -f lockfile
  export PX4_SYS_AUTOSTART=${PX4_AUTOSTART}
  export PX4_GZ_STANDALONE=1
  export PX4_GZ_MODEL_NAME=${PX4_MODEL_NAME}
  export PX4_GZ_WORLD=${PX4_GZ_WORLD}
  export PX4_SIMULATOR=gz
  export GZ_DISTRO=harmonic
  ${PX4_BUILD_DIR}/bin/px4 -s ${PX4_BUILD_DIR}/etc/init.d-posix/rcS ${PX4_BUILD_DIR}/etc
"
