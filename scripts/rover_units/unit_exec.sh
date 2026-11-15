#!/usr/bin/env bash
# ExecStart of the rover-*.service units: run one component in the dev
# container, in the foreground, so systemd sees it exit.
#
#   unit_exec.sh <name> -- <command> [args...]
#
# Stopping the `docker compose exec` client does not stop the process inside
# the container, so every unit also has ExecStop=unit_stop.sh <name>.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../lib/dc.sh
source "${SCRIPT_DIR}/../lib/dc.sh"

name="${1:?usage: unit_exec.sh <name> -- <command> [args...]}"
shift
[[ "${1:-}" == "--" ]] && shift

xdcomp exec -T ackermann_slam /workspace/scripts/rover_units/in_container.sh "${name}" "$@"
