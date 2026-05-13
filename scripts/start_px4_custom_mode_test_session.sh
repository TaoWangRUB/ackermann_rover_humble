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
#   ./scripts/start_px4_custom_mode_test_session.sh                                # defaults (/dev/ttyUSB0 @ 921600)
#   ./scripts/start_px4_custom_mode_test_session.sh --mode-id 24                   # custom mode ID
#   ./scripts/start_px4_custom_mode_test_session.sh --no-activate                  # skip mode activation
#   ./scripts/start_px4_custom_mode_test_session.sh --serial-dev=/dev/ttyTHS0      # only if Jetson 40-pin UART is actually wired
#   ./scripts/start_px4_custom_mode_test_session.sh --serial-baud=460800           # drop baud for Tegra
#
# Standard rover wiring in this repo uses /dev/ttyUSB0 for DDS. Override
# --serial-dev only when you have intentionally moved the XRCE link to a
# different device such as Jetson's 40-pin UART.
#
# IMPORTANT (hardware): pane 1 only starts the host-side MicroXRCEAgent. PX4's
# uxrce_dds_client must connect after that. If the flight controller is already
# booted, the DDS pane can stay idle until you reboot PX4 or manually restart
# uxrce_dds_client on the FMU.
#
# Any other args are passed through to start_px4_bringup_vo.sh (e.g. --mode-type).
#
# To stop everything:
#   ./scripts/stop_all.sh --session px4test
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/dc.sh"

# --- Defaults ---
SESSION="px4test"
MODE_ID="23"
ACTIVATE=true
ATTACH=true
SERIAL_DEV="/dev/ttyUSB0"
SERIAL_BAUD="921600"
PASSTHROUGH=()

# --- Parse arguments ---
for arg in "$@"; do
    case "${arg}" in
        --no-activate)    ACTIVATE=false ;;
        --no-attach)      ATTACH=false ;;
        --mode-id=*)      MODE_ID="${arg#--mode-id=}" ;;
        --session=*)      SESSION="${arg#--session=}" ;;
        --serial-dev=*)   SERIAL_DEV="${arg#--serial-dev=}" ;;
        --serial-baud=*)  SERIAL_BAUD="${arg#--serial-baud=}" ;;
        -h|--help)
            sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
            exit 0 ;;
        *)                PASSTHROUGH+=("${arg}") ;;
    esac
done

# Kill existing session if present
tmux kill-session -t "${SESSION}" 2>/dev/null || true

cat <<EOF
Starting tmux session '${SESSION}'...
XRCE serial device: ${SERIAL_DEV} @ ${SERIAL_BAUD}

Standard repo default for hardware DDS is /dev/ttyUSB0.
Note: for real hardware, the MicroXRCEAgent must be up before PX4 boots.
If PX4 is already running, reboot PX4 or restart uxrce_dds_client after pane 1 starts,
otherwise the DDS pane may appear healthy but no XRCE traffic will arrive.
EOF

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
    "${SCRIPT_DIR}/start_microxrce_agent.sh --serial ${SERIAL_DEV} ${SERIAL_BAUD}" Enter

tmux send-keys -t "${SESSION}:px4test.2" \
    "sleep 3 && ${SCRIPT_DIR}/pub_odom.sh" Enter

tmux send-keys -t "${SESSION}:px4test.3" \
    "sleep 3 && ${SCRIPT_DIR}/pub_tf.sh" Enter

tmux send-keys -t "${SESSION}:px4test.0" \
    "sleep 5 && ${SCRIPT_DIR}/start_px4_bringup_vo.sh --bridge --vo-bridge ${PASSTHROUGH[*]:-}" Enter

if [[ "$ACTIVATE" == true ]]; then
    tmux send-keys -t "${SESSION}:px4test.4" \
        "echo 'Waiting 20s for PX4 mode registration...' && sleep 20 && ${SCRIPT_DIR}/activate_rover_manual.sh ${MODE_ID}; source ${SCRIPT_DIR}/lib/dc.sh && xdcomp exec ackermann_slam bash" Enter
else
    tmux send-keys -t "${SESSION}:px4test.4" \
        "source ${SCRIPT_DIR}/lib/dc.sh && xdcomp exec ackermann_slam bash" Enter
fi

# Select the PX4 bringup pane and attach when interactive
tmux select-pane -t "${SESSION}:px4test.0"
if [[ "${ATTACH}" == true && -t 1 ]]; then
    tmux attach-session -t "${SESSION}"
fi
