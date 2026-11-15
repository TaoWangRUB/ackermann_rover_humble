#!/usr/bin/env bash
# Run one rover component inside the dev container (called by unit_exec.sh).
#
#   in_container.sh <name> <command> [args...]
#
# Every component gets the same environment: the Fast DDS profile hook the
# entrypoint writes (normally only sourced by login shells, so `bash -c`
# launches used to run with Fast DDS defaults while `bash -lc` ones used the
# profile), ROS 2 and the workspace overlay. The PID goes to /dev/shm (the
# container shares the host's IPC and PID namespaces), where unit_stop.sh on the
# host reads it.
set -eo pipefail

name="${1:?usage: in_container.sh <name> <command> [args...]}"
shift
[[ $# -gt 0 ]] || { echo "in_container.sh: no command for '${name}'" >&2; exit 2; }

run_dir=/dev/shm/rover_units
mkdir -p "${run_dir}"

if [[ -f "${HOME}/.fastdds_eth_env.sh" ]]; then
    # shellcheck source=/dev/null
    source "${HOME}/.fastdds_eth_env.sh"
fi
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [[ -f /workspace/install/setup.bash ]]; then
    # shellcheck source=/dev/null
    source /workspace/install/setup.bash
fi

echo "$$" > "${run_dir}/${name}.pid"
echo "[${name}] starting (pid $$, DDS profile ${FASTRTPS_DEFAULT_PROFILES_FILE:-none}): $*"
exec "$@"
