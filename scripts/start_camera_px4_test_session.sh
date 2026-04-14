#!/usr/bin/env bash
# Start a tmux session with 3 panes in one window for HW development:
#   Top-left:     Micro-XRCE-DDS Agent (serial)
#   Top-right:    ROS 2 nodes (HW mode)
#   Bottom:       Odometry verification (loops every 5s)
#
# Layout:
#   ┌──────────────────┬──────────────────┐
#   │                  │   XRCE Agent     │
#   │   ROS 2 Nodes    ├──────────────────┤
#   │                  │  Verification    │
#   └──────────────────┴──────────────────┘
#
# Usage:
#   ./scripts/start_camera_px4_test_session.sh                                         # defaults
#   ./scripts/start_camera_px4_test_session.sh --depth-camera=d435i --t265 --rtabmap   # pass flags to ROS 2 nodes
#   ./scripts/start_camera_px4_test_session.sh --no-verify                             # skip verification pane
#
# The Micro-XRCE-DDS Agent starts first. ROS 2 nodes start after a short
# delay so the agent is ready. Verification starts after nodes have time
# to publish.
#
# To stop everything:
#   ./scripts/stop_all.sh
#   tmux kill-session -t rover
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- Defaults ---
SESSION="rover"
ROS2_ARGS="--hw --no-rviz"
VERIFY=true

# --- Parse arguments ---
PASSTHROUGH=()
for arg in "$@"; do
    case "${arg}" in
        --no-verify)    VERIFY=false ;;
        --session=*)    SESSION="${arg#--session=}" ;;
        -h|--help)
            sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
            exit 0 ;;
        *)              PASSTHROUGH+=("${arg}") ;;
    esac
done

# Append any extra flags (they go to start_ros2_nodes.sh)
if [[ ${#PASSTHROUGH[@]} -gt 0 ]]; then
    ROS2_ARGS+=" ${PASSTHROUGH[*]}"
fi

# Kill existing session if present
tmux kill-session -t "${SESSION}" 2>/dev/null || true

# Each pane runs commands via send-keys. Ctrl+C kills the running command
# and drops back to the host bash prompt. Scripts run on the host and use
# docker-compose exec internally to talk to the container.
#
# --- Pane 0 (left, full height): ROS 2 nodes ---
tmux new-session -d -s "${SESSION}" -n "rover"

# Enable mouse: click to select pane, scroll, resize panes (after session exists)
tmux set-option -t "${SESSION}" -g mouse on
tmux send-keys -t "${SESSION}:rover.0" \
    "sleep 3 && ${SCRIPT_DIR}/start_ros2_nodes.sh ${ROS2_ARGS}" Enter

# --- Pane 1 (right-top): Micro-XRCE-DDS Agent (serial) ---
tmux split-window -h -t "${SESSION}:rover"
tmux send-keys -t "${SESSION}:rover.1" \
    "${SCRIPT_DIR}/start_microxrce_agent.sh --serial" Enter

# --- Pane 2 (right-bottom): Verification ---
if [[ "$VERIFY" == true ]]; then
    tmux split-window -v -t "${SESSION}:rover.1" -l 12
    tmux send-keys -t "${SESSION}:rover.2" \
        "sleep 30 && ${SCRIPT_DIR}/verify_odom.sh --loop; source ${SCRIPT_DIR}/lib/dc.sh && xdcomp exec ackermann_slam bash" Enter
fi

# Select the ROS 2 nodes pane and attach
tmux select-pane -t "${SESSION}:rover.0"
tmux attach-session -t "${SESSION}"
