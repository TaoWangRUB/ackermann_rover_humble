#!/usr/bin/env bash
# Start PX4 SITL inside the Docker container.
#
# Usage (from host):
#   ./scripts/start_px4_sitl.sh               # custom ackermann model (standalone, needs Gazebo)
#   ./scripts/start_px4_sitl.sh --official    # official gz_rover_ackermann (PX4 spawns Gazebo itself)
#   ./scripts/start_px4_sitl.sh --vio         # custom model + VIO-only EKF2 (no GPS)
#   ./scripts/start_px4_sitl.sh --official --vio  # official model + VIO-only EKF2
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
#
# --- VIO mode (--vio) ---
# Disables GPS fusion, enables External Vision (EV) for position + velocity + yaw.
# After PX4 boots, run from pxh>:
#   commander set_ekf_origin 51.4934 7.4120 100.0
#   commander mode auto:loiter
# Verify with:
#   listener vehicle_local_position     (xy_valid and z_valid should be true)
#   ekf2 status                         (should show "Using EV position / EV velocity")
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"

PX4_BUILD_DIR="/px4/build/px4_sitl_default"

# --- Parse flags ---
OFFICIAL_MODEL=0
VIO_MODE=0
POSITIONAL=()
for arg in "$@"; do
  case "$arg" in
    --official) OFFICIAL_MODEL=1 ;;
    --vio)      VIO_MODE=1 ;;
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
    xdcomp exec ackermann_slam bash -c "
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

    [[ "${VIO_MODE}" == "1" ]] && MODE_LABEL="${MODE_LABEL} [VIO]"
    echo "Starting PX4 SITL (official mode) inside Docker container..."
    echo "  Mode:          ${MODE_LABEL}"
    echo "  PX4 build dir: ${PX4_BUILD_DIR} (inside container)"
    echo ""
    if [[ "${VIO_MODE}" == "1" ]]; then
        echo "  [VIO] After boot, run from pxh>:"
        echo "    commander set_ekf_origin 51.4934 7.4120 100.0"
        echo "    commander mode auto:loiter"
        echo ""
    fi

    xdcomp exec ackermann_slam bash -c "
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

      $([ "${VIO_MODE}" == "1" ] && echo '
      # --vio: PX4_PARAM_* env vars are applied by init.d-posix/rcS after param import
      # and airframe defaults — this is the correct SITL override mechanism.
      export PX4_PARAM_EKF2_GPS_CTRL=0
      export PX4_PARAM_EKF2_EV_CTRL=15
      export PX4_PARAM_EKF2_HGT_REF=3
      export PX4_PARAM_EKF2_MAG_TYPE=5
      export PX4_PARAM_EKF2_EVP_NOISE=0.1
      export PX4_PARAM_EKF2_EVV_NOISE=0.1
      export PX4_PARAM_EKF2_EVA_NOISE=0.1
      export PX4_PARAM_EKF2_EV_DELAY=50.0
      export PX4_PARAM_COM_RC_IN_MODE=4
      ')

      # Run px4 in foreground with a real TTY — matches \`make px4_sitl gz_rover_ackermann\`
      # (CMake USES_TERMINAL). This gives you an interactive pxh> shell.
      #
      # Once initialized, run from the pxh> prompt:
      #   commander set_ekf_origin 51.4934 7.4120 100.0
      #   commander mode auto:loiter
      ${PX4_BUILD_DIR}/bin/px4
    "
else
    # Custom model: connect to an already-running Gazebo instance (standalone mode).
    PX4_MODEL_NAME="${PX4_MODEL_NAME:-ackermann}"
    PX4_AUTOSTART="${PX4_AUTOSTART:-51000}"
    PX4_GZ_WORLD="${PX4_GZ_WORLD:-warehouse}"
    MODE_LABEL="custom (model: ${PX4_MODEL_NAME}, standalone)"

    [[ "${VIO_MODE}" == "1" ]] && MODE_LABEL="${MODE_LABEL} [VIO]"
    echo "Starting PX4 SITL (custom model mode) inside Docker container..."
    echo "  Mode:          ${MODE_LABEL}"
    echo "  Airframe:      ${PX4_AUTOSTART}"
    echo "  World:         ${PX4_GZ_WORLD}"
    echo "  PX4 build dir: ${PX4_BUILD_DIR} (inside container)"
    echo ""
    if [[ "${VIO_MODE}" == "1" ]]; then
        echo "  [VIO] After boot, run from pxh>:"
        echo "    commander set_ekf_origin 51.4934 7.4120 100.0"
        echo "    commander mode auto:loiter"
        echo ""
    fi

    xdcomp exec ackermann_slam bash -c "
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

      $([ "${VIO_MODE}" == "1" ] && echo '
      # --vio: PX4_PARAM_* env vars are applied by init.d-posix/rcS after param import
      # and airframe defaults — this is the correct SITL override mechanism.
      export PX4_PARAM_EKF2_GPS_CTRL=0
      export PX4_PARAM_EKF2_EV_CTRL=15
      export PX4_PARAM_EKF2_HGT_REF=3
      export PX4_PARAM_EKF2_MAG_TYPE=5
      export PX4_PARAM_EKF2_EVP_NOISE=0.1
      export PX4_PARAM_EKF2_EVV_NOISE=0.1
      export PX4_PARAM_EKF2_EVA_NOISE=0.1
      export PX4_PARAM_EKF2_EV_DELAY=50.0
      export PX4_PARAM_COM_RC_IN_MODE=4
      ')

      # Run px4 in foreground so pxh> shell is fully interactive.
      #
      # Once initialized, run from the pxh> prompt:
      #   commander set_ekf_origin 51.4934 7.4120 100.0
      #   commander mode auto:loiter
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
