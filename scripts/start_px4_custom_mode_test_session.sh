#!/usr/bin/env bash
# Start a tmux session to verify the PX4 custom mode (rover_manual_mode)
# without cameras or SLAM — just XRCE agent, mock odom/TF, PX4 bridge,
# and mode activation.
#
# Layout:
#   ┌──────────────────┬──────────────────┐
#   │                  │  XRCE Agent      │
#   │  PX4 Bringup     ├──────────────────┤
#   │  (bridge + VO)   │  Mock Odom       │
#   │                  ├──────────────────┤
#   │                  │  Mock TF         │
#   └──────────────────┴──────────────────┘
#
# After all panes are up, the script activates the custom mode and arms.
#
# Usage:
#   ./scripts/start_px4_test_session.sh                  # defaults
#   ./scripts/start_px4_test_session.sh --mode-id 24     # custom mode ID
#   ./scripts/start_px4_test_session.sh --no-activate    # skip mode activation
#
# To stop everything:
#   tmux kill-session -t px4test
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- Defaults ---
SESSION="px4test"
MODE_ID="23"
ACTIVATE=true
PASSTHROUGH=()

# --- Parse arguments ---
for arg in "$@"; do
    case "${arg}" in
        --no-activate)   ACTIVATE=false ;;
        --mode-id=*)     MODE_ID="${arg#--mode-id=}" ;;
        --session=*)     SESSION="${arg#--session=}" ;;
        -h|--help)
            sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
            exit 0 ;;
        *)               PASSTHROUGH+=("${arg}") ;;
    esac
done

# Kill existing session if present
tmux kill-session -t "${SESSION}" 2>/dev/null || true

# --- Pane 0 (left-top): PX4 bringup (bridge + VO) ---
tmux new-session -d -s "${SESSION}" -n "px4test"
tmux set-option -t "${SESSION}" -g mouse on
tmux set-option -t "${SESSION}" -g pane-base-index 0

# --- Pane 1 (right-top): Micro-XRCE-DDS Agent (serial) ---
tmux split-window -h -t "${SESSION}:px4test.0"

# --- Pane 2 (right-middle): Mock odometry publisher ---
tmux split-window -v -t "${SESSION}:px4test.1"

# --- Pane 3 (right-bottom): Mock TF publisher ---
tmux split-window -v -t "${SESSION}:px4test.2" -l 8

# --- Pane 4 (left-bottom): Activation ---
tmux split-window -v -t "${SESSION}:px4test.0" -l 6

# Now send commands to each pane (all panes exist, indices are stable)
tmux send-keys -t "${SESSION}:px4test.1" \
    "${SCRIPT_DIR}/start_microxrce_agent.sh --serial" Enter

tmux send-keys -t "${SESSION}:px4test.2" \
    "sleep 3 && ${SCRIPT_DIR}/pub_odom.sh" Enter

tmux send-keys -t "${SESSION}:px4test.3" \
    "sleep 3 && ${SCRIPT_DIR}/pub_tf.sh" Enter

tmux send-keys -t "${SESSION}:px4test.0" \
    "sleep 5 && ${SCRIPT_DIR}/start_px4_bringup_vo.sh --bridge --vo-bridge ${PASSTHROUGH[*]:-}" Enter

if [[ "$ACTIVATE" == true ]]; then
    tmux send-keys -t "${SESSION}:px4test.4" \
        "echo 'Waiting 20s for PX4 mode registration...' && sleep 20 && ${SCRIPT_DIR}/activate_rover_manual.sh ${MODE_ID}" Enter
else
    tmux send-keys -t "${SESSION}:px4test.4" \
        "# Activation disabled (--no-activate). Run manually: ${SCRIPT_DIR}/activate_rover_manual.sh ${MODE_ID}" Enter
fi

# Select the PX4 bringup pane and attach
tmux select-pane -t "${SESSION}:px4test.0"
exec tmux attach-session -t "${SESSION}"
