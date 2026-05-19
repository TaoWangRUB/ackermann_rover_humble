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

    # Extra sweep for long-running Python helpers started from /workspace/scripts.
    # Keep this narrow so we do not kill unrelated system Python processes.
    sweep_python_helpers() {
        ${DC} exec -T ackermann_slam bash -c "
            ps aux --no-headers \
              | grep -E 'python3 /workspace/scripts/' \
              | grep -v 'docker\|grep' \
              | awk '{print \$2}' \
              | xargs -r kill -9 2>/dev/null || true
        " 2>/dev/null || true
    }

    # Gazebo Harmonic is started by ros_gz_sim through the gz CLI, which is a
    # Ruby wrapper on many installs. Those processes do not include /opt/ros or
    # /workspace/install in their argv, so handle them explicitly. Keep the
    # match tied to Gazebo/GZ arguments so unrelated Ruby processes survive.
    sweep_gazebo() {
        ${DC} exec -T ackermann_slam bash -c '
            ps -eo pid=,comm=,args= \
              | awk '"'"'
                  $2 ~ /^(ruby|gz|gazebo|gzserver|gzclient|ign)$/ &&
                  $0 ~ /(\/usr\/bin\/gz|(^|[[:space:]])gz[[:space:]]+sim|(^|[[:space:]])ign[[:space:]]+gazebo|gazebo|gzserver|gzclient)/ {
                      print $1
                  }
                '"'"' \
              | xargs -r kill -9 2>/dev/null || true
        ' 2>/dev/null || true
    }

    # Graceful shutdown: SIGTERM PX4 mode nodes one at a time so each
    # UnregisterExtComponent message can traverse the serial XRCE link
    # to PX4 without contention. Sending all SIGTERMs at once causes
    # only ~1 of N unregister messages to arrive before the process and
    # DDS middleware exit.
    ${DC} exec -T ackermann_slam bash -c '
        PIDS=$(ps aux --no-headers \
          | grep -E "rover_(manual|speed_steering|speed_attitude|speed_rate)_mode|offboard_trajectory_mode" \
          | grep -v "docker\|grep" \
          | awk "{print \$2}")
        for p in $PIDS; do
            kill -15 $p 2>/dev/null || continue
            # Wait up to 3s for this one process to exit
            for i in $(seq 1 6); do
                kill -0 $p 2>/dev/null || break
                sleep 0.5
            done
        done
    ' 2>/dev/null || true

    sweep
    sweep_python_helpers
    sweep_gazebo
    sleep 1
    sweep   # second pass catches orphans that re-parented after first kill
    sweep_python_helpers
    sweep_gazebo

    # Verify
    REMAINING=$(${DC} exec -T ackermann_slam bash -c "
        {
          ps aux --no-headers \
            | grep -E '/opt/ros|/opt/microxrce|/workspace/install|python3 /workspace/scripts/' \
            | grep -v 'docker\|grep'
          ps -eo pid=,comm=,args= \
            | awk '
                \$2 ~ /^(ruby|gz|gazebo|gzserver|gzclient|ign)$/ &&
                \$0 ~ /(\/usr\/bin\/gz|(^|[[:space:]])gz[[:space:]]+sim|(^|[[:space:]])ign[[:space:]]+gazebo|gazebo|gzserver|gzclient)/ {
                    print
                }
              '
        } | wc -l
    " 2>/dev/null || echo "0")

    if [[ "${REMAINING}" -eq 0 ]]; then
        echo "All container processes stopped."
    else
        echo "WARNING: ${REMAINING} process(es) still running:"
        ${DC} exec -T ackermann_slam bash -c "
            ps aux --no-headers \
              | grep -E '/opt/ros|/opt/microxrce|/workspace/install|python3 /workspace/scripts/' \
              | grep -v 'docker\|grep'
            ps -eo pid=,comm=,args= \
              | awk '
                  \$2 ~ /^(ruby|gz|gazebo|gzserver|gzclient|ign)$/ &&
                  \$0 ~ /(\/usr\/bin\/gz|(^|[[:space:]])gz[[:space:]]+sim|(^|[[:space:]])ign[[:space:]]+gazebo|gazebo|gzserver|gzclient)/ {
                      print
                  }
                '
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
