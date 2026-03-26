#!/usr/bin/env bash
# Stop all ROS 2 / Gazebo / PX4 processes inside the Docker container,
# then optionally kill a tmux session on the host.
#
# Usage:
#   ./scripts/stop_all.sh                          # container processes only
#   ./scripts/stop_all.sh --session px4test        # also kill tmux session
set -euo pipefail

TMUX_SESSION=""

for arg in "$@"; do
    case "${arg}" in
        -h|--help) sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"; exit 0 ;;
        --session=*) TMUX_SESSION="${arg#--session=}" ;;
        --session)   shift; TMUX_SESSION="${1:-}" ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"

DC="dcomp"

# ── Container cleanup ──────────────────────────────────────────────────────
if ! ${DC} ps 2>/dev/null | grep -q "Up"; then
    echo "Container is not running."
else
    echo "Stopping all processes in container..."

    # Stop the ROS 2 daemon first (graceful, avoids stale discovery cache)
    ${DC} exec -T ackermann_slam bash -c "
        source /opt/ros/\${ROS_DISTRO}/setup.bash 2>/dev/null
        ros2 daemon stop 2>/dev/null || true
    " 2>/dev/null || true

    # Sweep: kill by install path, excluding docker infrastructure processes.
    # NOTE: pkill -f is intentionally avoided here — with pid:host the exec
    # process's own cmdline contains the pattern strings, causing self-kill.
    # grep -v docker excludes host-side docker exec/compose processes that
    # also appear in ps output due to shared PID namespace.
    sweep() {
        ${DC} exec -T ackermann_slam bash -c "
            ps aux --no-headers \
              | grep -E '/opt/ros|/opt/microxrce|/workspace/install' \
              | grep -v 'docker\|grep' \
              | awk '{print \$2}' \
              | xargs -r kill -9 2>/dev/null || true
        " 2>/dev/null || true
    }

    sweep
    sleep 1
    sweep   # second pass catches orphans that re-parented after first kill

    # Verify
    REMAINING=$(${DC} exec -T ackermann_slam bash -c "
        ps aux --no-headers \
          | grep -E '/opt/ros|/opt/microxrce|/workspace/install' \
          | grep -v 'docker\|grep' \
          | wc -l
    " 2>/dev/null || echo "0")

    if [[ "${REMAINING}" -eq 0 ]]; then
        echo "All container processes stopped."
    else
        echo "WARNING: ${REMAINING} process(es) still running:"
        ${DC} exec -T ackermann_slam bash -c "
            ps aux --no-headers \
              | grep -E '/opt/ros|/opt/microxrce|/workspace/install' \
              | grep -v 'docker\|grep'
        " 2>/dev/null || true
    fi
fi

# ── Tmux session cleanup (host) ────────────────────────────────────────────
if [[ -n "${TMUX_SESSION}" ]]; then
    if tmux has-session -t "${TMUX_SESSION}" 2>/dev/null; then
        tmux kill-session -t "${TMUX_SESSION}"
        echo "Tmux session '${TMUX_SESSION}' killed."
    else
        echo "Tmux session '${TMUX_SESSION}' not running."
    fi
fi
