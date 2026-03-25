#!/usr/bin/env bash
# Run the px4_mavlink_vpe.py ROS2->MAVLink bridge inside the Docker container.
#
# Usage (from host):
#   ./scripts/px4_mavlink_vpe.sh [--device /dev/ttyACM0] [--baud 57600] \
#       [--rate 20] [--topic /odometry/filtered]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${PROJECT_DIR}/docker/docker-compose.yml"
ENV_FILE="${PROJECT_DIR}/.env"

# Defaults (match px4_mavlink_vpe.py)
DEVICE="/dev/ttyACM0"
BAUD=57600
RATE=20
TOPIC="/odometry/filtered"

EXTRA_ARGS=()

print_usage() {
  cat <<EOF
Usage: $0 [--device DEVICE] [--baud BAUD] [--rate RATE] [--topic TOPIC] [-- extra args]

Defaults:
  --device ${DEVICE}
  --baud   ${BAUD}
  --rate   ${RATE}
  --topic  ${TOPIC}

Any additional arguments after "--" are forwarded to the python script.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --device)
      DEVICE="$2"; shift 2;;
    --baud)
      BAUD="$2"; shift 2;;
    --rate)
      RATE="$2"; shift 2;;
    --topic)
      TOPIC="$2"; shift 2;;
    -h|--help)
      print_usage; exit 0;;
    --)
      shift; EXTRA_ARGS=("$@"); break;;
    *)
      # allow passing positional args directly through
      EXTRA_ARGS+=("$1"); shift;;
  esac
done

CMD="source /workspace/install/setup.bash && exec python3 /workspace/src/px4_bringup/scripts/px4_mavlink_vpe.py \
  --device '${DEVICE}' --baud '${BAUD}' --rate '${RATE}' --topic '${TOPIC}'"

if [ ${#EXTRA_ARGS[@]} -gt 0 ]; then
  for a in "${EXTRA_ARGS[@]}"; do
    CMD+=" \"${a}\""
  done
fi

exec docker-compose --env-file "${ENV_FILE}" -f "${COMPOSE_FILE}" exec ackermann_slam \
  bash -lc "$CMD"
