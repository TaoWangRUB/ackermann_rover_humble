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
#   4. For ROS 2 DDS bridge: start MicroXRCEAgent BEFORE this script
#      (./scripts/start_microxrce_agent.sh in a separate terminal)
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
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

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
  ${PX4_BUILD_DIR}/bin/px4 -s ${PX4_BUILD_DIR}/etc/init.d-posix/rcS ${PX4_BUILD_DIR}/etc &
  PX4_PID=\$!

  # Wait for PX4 to finish initialization
  echo 'Waiting for PX4 to initialize...'
  for i in \$(seq 1 30); do
    if ${PX4_BUILD_DIR}/bin/px4-param show SYS_AUTOSTART >/dev/null 2>&1; then
      break
    fi
    sleep 1
  done
  sleep 3

  # Rover-specific parameter overrides:
  # - COM_RC_IN_MODE=4: Disable manual RC requirement for SITL without joystick.
  echo 'Applying rover SITL parameter overrides...'
  ${PX4_BUILD_DIR}/bin/px4-param set COM_RC_IN_MODE 4

  # Switch to Hold mode so preflight check passes (Manual mode requires RC).
  ${PX4_BUILD_DIR}/bin/px4-commander mode auto:loiter

  # Set EKF global origin (SET_GPS_GLOBAL_ORIGIN) — required for Hold without GPS.
  # Without a global position reference, home_position_invalid stays true and
  # arming in Hold is blocked.
  ${PX4_BUILD_DIR}/bin/px4-commander set_ekf_origin 51.4934 7.4120 100.0

  # Set home position to current location (MAV_CMD_DO_SET_HOME).
  ${PX4_BUILD_DIR}/bin/px4-commander set_home

  # === Actuator Test Notes ===
  # To test motors/servos via actuator_test, the rover MUST be DISARMED.
  # When armed (even in Hold mode), PX4's rover controller sends zero-velocity
  # commands that override actuator_test output.
  #
  # Disarm first, then run from the PX4 shell:
  #   ./px4-commander disarm
  #   ./px4-actuator_test set -m 1 -v 0.5 -t 3   # motor test
  #   ./px4-actuator_test set -s 1 -v 0.5 -t 3   # servo test
  #
  # To drive the rover via ROS 2 /cmd_vel, launch px4_bridge AFTER this script:
  #   ros2 launch px4_bringup px4_bringup.launch.py mode_type:=speed_steering

  wait \$PX4_PID
"
