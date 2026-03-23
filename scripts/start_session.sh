#!/usr/bin/env bash
# Start a tmux session with 3 panes in one window for HW development:
#   Top-left:     Micro-XRCE-DDS Agent (serial)
#   Top-right:    ROS 2 nodes (HW mode)
#   Bottom:       Odometry verification (loops every 5s)
#
# Layout:
#   ┌──────────────────┬──────────────────┐
#   │   XRCE Agent     │   ROS 2 Nodes    │
#   ├──────────────────┴──────────────────┤
#   │          Verification               │
#   └─────────────────────────────────────┘
#
# Usage:
#   ./scripts/start_session.sh                                         # defaults
#   ./scripts/start_session.sh --depth-camera=d435i --t265 --rtabmap   # pass flags to ROS 2 nodes
#   ./scripts/start_session.sh --no-verify                             # skip verification pane
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
ROS2_ARGS="--hw"
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

# --- Pane 0 (top-left): Micro-XRCE-DDS Agent (serial) ---
tmux new-session -d -s "${SESSION}" -n "rover" \
    "bash -c '${SCRIPT_DIR}/start_microxrce_agent.sh --serial; echo \"[exited \$?] — press Enter to restart\"; read; exec bash'"

# Keep panes open if a command exits or crashes (must be after session exists)
tmux set-option -t "${SESSION}" remain-on-exit on

# --- Pane 1 (top-right): ROS 2 nodes ---
tmux split-window -h -t "${SESSION}:rover" \
    "bash -c 'echo \"Waiting 3s for XRCE agent...\"; sleep 3; ${SCRIPT_DIR}/start_ros2_nodes.sh ${ROS2_ARGS}; echo \"[exited \$?] — press Enter to restart\"; read; exec bash'"

# --- Pane 2 (bottom): Verification ---
if [[ "$VERIFY" == true ]]; then
    tmux split-window -v -t "${SESSION}:rover" -l 12 \
        "bash -c 'echo \"Waiting 30s for nodes to start publishing...\"; sleep 30; ${SCRIPT_DIR}/verify_odom.sh --loop; echo \"[exited \$?]\"; read; exec bash'"
fi

# Select the ROS 2 nodes pane (top-right) and attach
tmux select-pane -t "${SESSION}:rover.1"
exec tmux attach-session -t "${SESSION}"
