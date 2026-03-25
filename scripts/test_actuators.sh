#!/usr/bin/env bash
# Test PX4 actuators (motors/servos) inside the Docker container.
# Wraps px4-actuator_test so you can run it from the host.
#
# Usage (from host):
#   ./scripts/test_actuators.sh motor              # motor at 50% for 3s
#   ./scripts/test_actuators.sh motor 0.8 5        # motor at 80% for 5s
#   ./scripts/test_actuators.sh servo              # servo at 50% for 3s
#   ./scripts/test_actuators.sh servo 0.3 2        # servo at 30% for 2s
#   ./scripts/test_actuators.sh iterate-motors     # cycle through all motors
#   ./scripts/test_actuators.sh iterate-servos     # cycle through all servos
#   ./scripts/test_actuators.sh shell              # open interactive PX4 shell
#
# Prerequisites:
#   1. Docker container must be running
#   2. Gazebo + PX4 SITL must already be running
#   3. Rover must be DISARMED (script disarms automatically)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${PROJECT_DIR}/docker/docker-compose.yml"
ENV_FILE="${PROJECT_DIR}/.env"
PX4_BIN="/px4/build/px4_sitl_default/bin"

docker_exec() {
    docker-compose --env-file "${ENV_FILE}" -f "${COMPOSE_FILE}" exec ackermann_slam bash -c "cd /px4/build/px4_sitl_default && $1"
}

usage() {
    echo "Usage: $0 <command> [value] [duration]"
    echo ""
    echo "Commands:"
    echo "  motor [value] [duration]   Test wheel motor (value: 0-1, default: 0.5, duration in seconds, default: 3)"
    echo "  servo [value] [duration]   Test steering servo (value: 0-1, default: 0.5, duration in seconds, default: 3)"
    echo "  iterate-motors             Cycle through all motors at 15%"
    echo "  iterate-servos             Cycle through all servos at 30%"
    echo "  shell                      Open interactive shell in PX4 build directory"
    exit 1
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

CMD="${1:-}"
VALUE="${2:-0.5}"
DURATION="${3:-3}"

case "$CMD" in
    motor)
        echo "Testing motor at ${VALUE} (${DURATION}s)..."
        echo "  Disarming first, then running actuator_test."
        echo ""
        docker_exec "${PX4_BIN}/px4-commander disarm 2>/dev/null; ${PX4_BIN}/px4-actuator_test set -m 1 -v ${VALUE} -t ${DURATION}"
        ;;
    servo)
        echo "Testing servo at ${VALUE} (${DURATION}s)..."
        echo "  Disarming first, then running actuator_test."
        echo ""
        docker_exec "${PX4_BIN}/px4-commander disarm 2>/dev/null; ${PX4_BIN}/px4-actuator_test set -s 1 -v ${VALUE} -t ${DURATION}"
        ;;
    iterate-motors)
        echo "Iterating all motors..."
        docker_exec "${PX4_BIN}/px4-commander disarm 2>/dev/null; ${PX4_BIN}/px4-actuator_test iterate-motors"
        ;;
    iterate-servos)
        echo "Iterating all servos..."
        docker_exec "${PX4_BIN}/px4-commander disarm 2>/dev/null; ${PX4_BIN}/px4-actuator_test iterate-servos"
        ;;
    shell)
        echo "Opening interactive shell in PX4 build directory..."
        exec docker-compose --env-file "${ENV_FILE}" -f "${COMPOSE_FILE}" exec ackermann_slam bash -c "cd /px4/build/px4_sitl_default/bin && exec bash"
        ;;
    *)
        usage
        ;;
esac
