#!/usr/bin/env bash
# ExecStop of the rover-*.service units: stop a component started by
# unit_exec.sh. SIGINT first, so `ros2 launch` shuts its nodes down cleanly
# (RoverManual unregisters from PX4, DDS participants leave), then SIGTERM, then
# SIGKILL for anything left, including the component's child processes.
#
#   unit_stop.sh <name> [grace_seconds]
#
# The container shares the host PID namespace (pid: host), so the PID that
# in_container.sh wrote is valid here.
set -uo pipefail

name="${1:?usage: unit_stop.sh <name> [grace_seconds]}"
grace="${2:-20}"
pidfile="/dev/shm/rover_units/${name}.pid"

[[ -f "${pidfile}" ]] || exit 0
pid="$(cat "${pidfile}")"

alive() { kill -0 "$1" 2>/dev/null; }
descendants() {
    local child
    for child in $(pgrep -P "$1" 2>/dev/null); do
        echo "${child}"
        descendants "${child}"
    done
}

if alive "${pid}"; then
    # Collected first: once the parent exits, its children are reparented.
    children="$(descendants "${pid}")"
    kill -INT "${pid}" 2>/dev/null || true
    for _ in $(seq 1 $((grace * 2))); do
        alive "${pid}" || break
        sleep 0.5
    done
    if alive "${pid}"; then
        echo "[${name}] still running ${grace}s after SIGINT: SIGTERM" >&2
        kill -TERM "${pid}" 2>/dev/null || true
        sleep 3
    fi
    for p in ${pid} ${children}; do
        if alive "${p}"; then
            echo "[${name}] killing leftover pid ${p}: $(tr '\0' ' ' < "/proc/${p}/cmdline" 2>/dev/null | cut -c1-100)" >&2
            kill -KILL "${p}" 2>/dev/null || true
        fi
    done
fi
rm -f "${pidfile}"
