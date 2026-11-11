#!/usr/bin/env bash
# Start PX4 SITL inside the Docker container.
#
# Usage (from host):
#   ./scripts/start_px4_sitl.sh               # custom ackermann model (standalone, needs Gazebo)
#   ./scripts/start_px4_sitl.sh --official    # official gz_rover_ackermann (PX4 spawns Gazebo itself)
#   ./scripts/start_px4_sitl.sh --vio         # custom model + VIO-only EKF2 (no GPS)
#   ./scripts/start_px4_sitl.sh --official --vio  # official model + VIO-only EKF2
#   ./scripts/start_px4_sitl.sh --reversible-drive  # bidirectional wheel ESC (see below)
#   ./scripts/start_px4_sitl.sh build         # rebuild PX4 inside Docker first
#
# --- Wheel ESC model ---
# By default the simulated wheel motor behaves like the rover's unidirectional
# ESC: throttle -1 = stop, +1 = full forward. This is what px4_bringup's
# RoverManual sends (bidirectional_esc=false). The stock airframe instead maps
# -1 to full reverse, so arming in RoverManual drove the sim rover backward.
# --reversible-drive keeps the stock mapping (0 = stop, -1 = full reverse);
# pass it together with start_ros2_nodes.sh --reversible-drive, and use it
# when testing PX4's closed-loop rover modes (speed_*), which assume 0 = stop.
# Both ways the throttle rate limits are disabled as on the rover
# (RO_ACCEL_LIM/RO_DECEL_LIM/RO_JERK_LIM = -1).
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
REVERSIBLE_DRIVE=0
POSITIONAL=()
for arg in "$@"; do
  case "$arg" in
    --official) OFFICIAL_MODEL=1 ;;
    --vio)      VIO_MODE=1 ;;
    --reversible-drive) REVERSIBLE_DRIVE=1 ;;
    *) POSITIONAL+=("$arg") ;;
  esac
done
set -- "${POSITIONAL[@]+"${POSITIONAL[@]}"}"

# --- Wheel ESC model ---
# Airframe 51000 maps the reversible wheel motor [-1, 1] onto
# SIM_GZ_WH_MIN1..MAX1 = 70..130, and the Gazebo bridge sets wheel speed to
# output - 100 (GZMixingInterfaceWheel.cpp). So -1 is full reverse. Raising
# the minimum to 100 makes -1 = 0 rad/s, like PWM_MIN on the rover's
# unidirectional ESC. Set it explicitly either way: SITL keeps parameters
# across restarts, so a previous run's value would otherwise stick.
if [[ "${REVERSIBLE_DRIVE}" == "1" ]]; then
    WHEEL_MIN1=70
    ESC_LABEL="bidirectional (0 = stop, -1 = full reverse)"
else
    WHEEL_MIN1=100
    ESC_LABEL="unidirectional (-1 = stop, +1 = full forward)"
fi
# Throttle rate limits as on the rover (RO_ACCEL_LIM/RO_DECEL_LIM/RO_JERK_LIM
# = -1, disabled; airframe 51000 sets 4/6/10). PX4 arms with the motor at 0
# and the SITL limits then ramp it toward the setpoint: with the
# unidirectional mapping above, 0 is half throttle, so the ramp jolted the
# rover ~10 cm forward on every arm.
SITL_PARAM_EXPORTS="export PX4_PARAM_SIM_GZ_WH_MIN1=${WHEEL_MIN1}
      export PX4_PARAM_RO_ACCEL_LIM=-1
      export PX4_PARAM_RO_DECEL_LIM=-1
      export PX4_PARAM_RO_JERK_LIM=-1"

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
    echo "  Wheel ESC:     ${ESC_LABEL}"
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
      # Wheel ESC model + rover throttle limits (applied by rcS after the
      # airframe defaults).
      ${SITL_PARAM_EXPORTS}

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
    echo "  Wheel ESC:     ${ESC_LABEL}"
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
      # Wheel ESC model + rover throttle limits (applied by rcS after the
      # airframe defaults).
      ${SITL_PARAM_EXPORTS}

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
