#!/usr/bin/env bash
# Start a tmux session with all Jetson rover components in one place.
#
# Launches (in dependency order):
#   1. Micro XRCE-DDS Agent (serial)           — immediate
#   2. System Monitor (rover_monitor)           — immediate (independent)
#   3. ROS 2 Nodes (cameras + RTAB-Map SLAM)   — after 3s (XRCE ready)
#   4. PX4 Bringup (custom mode + VO bridge)   — after 8s (ROS nodes up)
#   5. Mode Activation (arm + custom mode)      — after 28s (PX4 registered)
#
# Layout without --with-telemetry (5 panes):
#   ┌─────────────────────────┬──────────────────────┐
#   │                         │  XRCE Agent          │
#   │  ROS 2 Nodes            ├──────────────────────┤
#   │  (cameras + SLAM)       │  System Monitor      │
#   │                         │  (rover_monitor)     │
#   ├─────────────────────────┼──────────────────────┤
#   │  PX4 Bringup            │  Mode Activation     │
#   │  (bridge + VO)          │  → container shell   │
#   └─────────────────────────┴──────────────────────┘
#
# Layout with --with-telemetry (6 panes):
#   ┌─────────────────────────┬──────────────────────┐
#   │                         │  XRCE Agent          │
#   │  ROS 2 Nodes            ├──────────────────────┤
#   │  (cameras + SLAM)       │  System Monitor      │
#   │                         │  (w/ telemetry)      │
#   ├─────────────────────────┼──────────────────────┤
#   │  PX4 Bringup            │  Mode Activation     │
#   │  (bridge + VO)          ├──────────────────────┤
#   │                         │  MQTT verify         │
#   └─────────────────────────┴──────────────────────┘
#
# Usage:
#   ./scripts/start_jetson_session.sh
#   ./scripts/start_jetson_session.sh --with-telemetry
#   ./scripts/start_jetson_session.sh --cuvslam-odom --with-telemetry
#   ./scripts/start_jetson_session.sh --vins-odom --with-telemetry
#   ./scripts/start_jetson_session.sh --t265-odom --with-telemetry
#   ./scripts/start_jetson_session.sh --nav2
#   ./scripts/start_jetson_session.sh --nav2 --with-telemetry
#   ./scripts/start_jetson_session.sh --localization
#   ./scripts/start_jetson_session.sh --keep-rtabmap-db
#   ./scripts/start_jetson_session.sh --mode-id=24
#   ./scripts/start_jetson_session.sh --mode-type=speed_steering
#   ./scripts/start_jetson_session.sh --reversible-drive
#   ./scripts/start_jetson_session.sh --no-activate
#   ./scripts/start_jetson_session.sh --record
#   ./scripts/start_jetson_session.sh --record --bag-name=kitchen_loop
#   ./scripts/start_jetson_session.sh --record --no-compress-fisheye
#   ./scripts/start_jetson_session.sh --no-attach
#   ./scripts/start_jetson_session.sh --session=mytest
#
# --record adds a new 'record' window to the same tmux session running
# scripts/record_bag.sh with the right --t265-odom/--cuvslam-odom/--vins-odom/
# --rgbd-odom flag forwarded automatically. Switch windows with Ctrl-B 0/1.
# The recorder also subscribes to /record/cmd, so the CC dashboard's Record
# button (start|stop|toggle) drives the same segments.
#
# To stop everything:
#   ./scripts/stop_all.sh --session=jetson
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"

SESSION="jetson"
ENABLE_TELEMETRY=false
ENABLE_NAV2=false
ENABLE_T265=false
T265_ODOM=false
CUVSLAM_ODOM=false
RGBD_ODOM=false
VINS_ODOM=false
LOCALIZATION=false
DELETE_DB_ON_START=""
ACTIVATE=true
MODE_ID="23"
ATTACH=true
DEPTH_CAMERA="d435i"
BROKER_HOST=""
PUBLISHER_CONFIG_FILE="/workspace/src/rover_monitor/config/publisher.yaml"
PX4_MODE_TYPE="manual"
REVERSIBLE_DRIVE=true
NAV2_CONTROLLER="mppi"
ENABLE_RECORD=false
RECORD_BAG_NAME=""
RECORD_COMPRESS_FISHEYE=""   # "" | "true" | "false"

for arg in "$@"; do
    case "${arg}" in
        --with-telemetry)  ENABLE_TELEMETRY=true ;;
        --nav2)            ENABLE_NAV2=true ;;
        --controller=*)    NAV2_CONTROLLER="${arg#--controller=}" ;;
        --t265)            ENABLE_T265=true ;;
        --t265-odom)       ENABLE_T265=true; T265_ODOM=true ;;
        --cuvslam-odom)    CUVSLAM_ODOM=true ;;
        --rgbd-odom)       RGBD_ODOM=true ;;
        --vins-odom)       VINS_ODOM=true ;;
        --localization)    LOCALIZATION=true ;;
        --wipe-rtabmap-db) DELETE_DB_ON_START="true" ;;
        --keep-rtabmap-db) DELETE_DB_ON_START="false" ;;
        --no-activate)     ACTIVATE=false ;;
        --mode-id=*)       MODE_ID="${arg#--mode-id=}" ;;
        --mode-type=*)     PX4_MODE_TYPE="${arg#--mode-type=}" ;;
        --reversible-drive) REVERSIBLE_DRIVE=true ;;
        --depth-camera=*)  DEPTH_CAMERA="${arg#--depth-camera=}" ;;
        --broker-host=*)   BROKER_HOST="${arg#--broker-host=}" ;;
        --publisher-config=*) PUBLISHER_CONFIG_FILE="${arg#--publisher-config=}" ;;
        --record)          ENABLE_RECORD=true ;;
        --bag-name=*)      RECORD_BAG_NAME="${arg#--bag-name=}" ;;
        --compress-fisheye)    RECORD_COMPRESS_FISHEYE="true" ;;
        --no-compress-fisheye) RECORD_COMPRESS_FISHEYE="false" ;;
        --no-attach)       ATTACH=false ;;
        --session=*)       SESSION="${arg#--session=}" ;;
        -h|--help)
            sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
            exit 0 ;;
    esac
done

# ── Build sub-command arguments ───────────────────────────────────────
ROS2_ARGS="--hw --no-rviz --depth-camera=${DEPTH_CAMERA}"
if [[ "${LOCALIZATION}" == true ]]; then
    ROS2_ARGS+=" --localization"
else
    ROS2_ARGS+=" --rtabmap"
fi
if [[ -n "${DELETE_DB_ON_START}" ]]; then
    if [[ "${DELETE_DB_ON_START}" == "true" ]]; then
        ROS2_ARGS+=" --wipe-rtabmap-db"
    else
        ROS2_ARGS+=" --keep-rtabmap-db"
    fi
fi
if [[ "${T265_ODOM}" == true ]]; then
    ROS2_ARGS+=" --t265-odom"
elif [[ "${ENABLE_T265}" == true ]]; then
    ROS2_ARGS+=" --t265"
fi
if [[ "${CUVSLAM_ODOM}" == true ]]; then
    ROS2_ARGS+=" --cuvslam-odom"
fi
if [[ "${RGBD_ODOM}" == true ]]; then
    ROS2_ARGS+=" --rgbd-odom"
fi
if [[ "${VINS_ODOM}" == true ]]; then
    ROS2_ARGS+=" --vins-odom"
fi
if [[ "${ENABLE_NAV2}" == true ]]; then
    ROS2_ARGS+=" --nav2 --controller=${NAV2_CONTROLLER}"
fi

# Derive canonical odom topic for readiness (same priority as rtabmap_slam.launch.py).
# Priority: cuVSLAM > cuVSLAM RGBD > VINS > T265 > /odometry/filtered (default).
# This determines which topic the PX4 pane waits for before launching.
if [[ "${CUVSLAM_ODOM}" == true ]]; then
    ODOM_READY_TOPIC="/cuvslam_odom"
elif [[ "${RGBD_ODOM}" == true ]]; then
    ODOM_READY_TOPIC="/cuvslam_rgbd_odom"
elif [[ "${VINS_ODOM}" == true ]]; then
    ODOM_READY_TOPIC="/vins_odom"
elif [[ "${T265_ODOM}" == true ]]; then
    ODOM_READY_TOPIC="/t265/odom_base"
else
    ODOM_READY_TOPIC="/odometry/filtered"
fi

ROS_SRC="source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash"
WAIT_FOR_ROS_READY="source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc '${ROS_SRC} && echo \"Waiting for ${ODOM_READY_TOPIC} topic...\" && timeout 120 bash -lc \"until ros2 topic list | grep -qx ${ODOM_READY_TOPIC}; do sleep 1; done\" && sleep 5'"
WAIT_FOR_PX4_READY="source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc '${ROS_SRC} && echo \"Waiting for PX4 DDS topics...\" && timeout 120 bash -lc \"until ros2 topic list | grep -qx /fmu/out/vehicle_status_v2; do sleep 1; done\" && sleep 10'"

TELEMETRY_ARG="enable_telemetry:=false"
PUBLISHER_CONFIG_ARG=""
BROKER_HOST_ARG=""
if [[ "${ENABLE_TELEMETRY}" == true ]]; then
    TELEMETRY_ARG="enable_telemetry:=true"
    PUBLISHER_CONFIG_ARG="publisher_config_file:=${PUBLISHER_CONFIG_FILE}"
fi

HOST_PUBLISHER_CONFIG_FILE="${PUBLISHER_CONFIG_FILE}"
if [[ "${HOST_PUBLISHER_CONFIG_FILE}" == /workspace/* ]]; then
    HOST_PUBLISHER_CONFIG_FILE="${PROJECT_DIR}/${HOST_PUBLISHER_CONFIG_FILE#/workspace/}"
fi

if [[ -z "${BROKER_HOST}" ]]; then
    if [[ -f "${HOST_PUBLISHER_CONFIG_FILE}" ]]; then
        BROKER_HOST="$(
            awk -F'"' '/broker_host:/ { print $2; exit }' \
            "${HOST_PUBLISHER_CONFIG_FILE}"
        )"
    fi
fi

BROKER_HOST="${BROKER_HOST:-localhost}"
if [[ "${ENABLE_TELEMETRY}" == true ]]; then
    BROKER_HOST_ARG="broker_host:=${BROKER_HOST}"
fi

TRACKING_EXPECT_STREAM=false
if [[ "${CUVSLAM_ODOM}" == true || "${VINS_ODOM}" == true ]]; then
    TRACKING_EXPECT_STREAM=true
fi

PX4_BRINGUP_ARGS=(--bridge --vo-bridge --mode-type "${PX4_MODE_TYPE}" --odom-topic "${ODOM_READY_TOPIC}")
if [[ "${REVERSIBLE_DRIVE}" == true ]]; then
    PX4_BRINGUP_ARGS+=(--reversible-drive)
fi

# ── Ensure Docker container is running ────────────────────────────────
if ! dcomp ps --services --filter status=running 2>/dev/null \
        | grep -q '^ackermann_slam$'; then
    echo "Container not running — starting ackermann_slam..."
    dcomp up -d ackermann_slam
fi

# ── Kill old ROS nodes before starting new ones ─────────────────────
"${SCRIPT_DIR}/stop_all.sh" --session="${SESSION}"
# Brief pause for process cleanup. Camera USB recovery is now handled
# by hardware_reset() inside the realsense_camera_node itself.
sleep 5

# ── Create tmux session ──────────────────────────────────────────────
tmux kill-session -t "${SESSION}" 2>/dev/null || true

tmux new-session -d -s "${SESSION}" -n "jetson"
tmux set-option -t "${SESSION}" -g mouse on
tmux set-option -t "${SESSION}" -g pane-base-index 0

# Create panes using pane IDs so the layout stays stable.
PANE_ROS2="$(tmux display-message -p -t "${SESSION}:jetson.0" "#{pane_id}")"
PANE_XRCE="$(tmux split-window -h -P -F "#{pane_id}" -t "${PANE_ROS2}")"
PANE_MONITOR="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_XRCE}")"
PANE_PX4="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_ROS2}" -l 15)"
PANE_ACTIVATE="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_MONITOR}")"

if [[ "${ENABLE_TELEMETRY}" == true ]]; then
    PANE_MQTT="$(tmux split-window -v -P -F "#{pane_id}" -t "${PANE_ACTIVATE}" -l 8)"
fi

# ── Pane: XRCE Agent (immediate) ─────────────────────────────────────
tmux send-keys -t "${PANE_XRCE}" \
    "${SCRIPT_DIR}/start_microxrce_agent.sh --serial" Enter

# ── Pane: System Monitor (immediate, independent) ────────────────────
tmux send-keys -t "${PANE_MONITOR}" \
    "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc '${ROS_SRC} && ros2 launch rover_monitor monitor.launch.py use_sim_time:=false depth_camera:=${DEPTH_CAMERA} tracking_expect_stream:=${TRACKING_EXPECT_STREAM} ${TELEMETRY_ARG} ${PUBLISHER_CONFIG_ARG} ${BROKER_HOST_ARG}'" Enter

# ── Pane: ROS 2 Nodes (wait for XRCE) ────────────────────────────────
tmux send-keys -t "${PANE_ROS2}" \
    "sleep 3 && ${SCRIPT_DIR}/start_ros2_nodes.sh ${ROS2_ARGS}" Enter

# ── Pane: PX4 Bringup (wait for fused odometry) ──────────────────────
tmux send-keys -t "${PANE_PX4}" \
    "sleep 8 && ${WAIT_FOR_ROS_READY} && ${SCRIPT_DIR}/start_px4_bringup_vo.sh ${PX4_BRINGUP_ARGS[*]}" Enter

# ── Pane: Mode Activation (wait for PX4 registration) ────────────────
if [[ "${ACTIVATE}" == true ]]; then
    tmux send-keys -t "${PANE_ACTIVATE}" \
        "${WAIT_FOR_PX4_READY} && ${SCRIPT_DIR}/activate_rover_manual.sh ${MODE_ID}; source ${SCRIPT_DIR}/lib/dc.sh && xdcomp exec ackermann_slam bash" Enter
else
    tmux send-keys -t "${PANE_ACTIVATE}" \
        "source ${SCRIPT_DIR}/lib/dc.sh && xdcomp exec ackermann_slam bash" Enter
fi

# ── Pane: MQTT verify (only with telemetry) ───────────────────────────
if [[ "${ENABLE_TELEMETRY}" == true ]]; then
    tmux send-keys -t "${PANE_MQTT}" \
    "source ${SCRIPT_DIR}/lib/dc.sh && dcomp exec ackermann_slam bash -lc 'set -o pipefail; if ! command -v mosquitto_sub >/dev/null 2>&1; then echo \"mosquitto_sub not installed\"; exit 1; fi; if ! python3 -c \"import google.protobuf\" >/dev/null 2>&1; then echo \"protobuf not installed\"; exit 1; fi; cd /tmp && protoc --python_out=/tmp -I /workspace/src/rover_monitor/proto /workspace/src/rover_monitor/proto/rover_health.proto >/dev/null 2>&1 || true; while true; do echo \"Subscribing to MQTT broker ${BROKER_HOST}...\"; mosquitto_sub -h ${BROKER_HOST} -t \"rover/health/#\" -F \"%x\" | python3 /workspace/scripts/decode_rover_health_mqtt.py --hex && break; status=$?; echo \"MQTT subscribe failed with status \${status}; retrying in 5s\"; sleep 5; done'" Enter
fi

tmux select-pane -t "${PANE_ROS2}"

# ── Window: Record (only with --record) ──────────────────────────────
# A separate tmux window keeps the existing pane grid untouched. The
# recorder waits for live topics (record_bag.sh has its own readiness
# gate), then idles waiting for either keystrokes (r/s) in the pane or
# 'start'/'stop'/'toggle' messages on /record/cmd from the CC button.
if [[ "${ENABLE_RECORD}" == true ]]; then
    REC_ARGS=()
    [[ -n "${RECORD_BAG_NAME}" ]]    && REC_ARGS+=("--name=${RECORD_BAG_NAME}")
    REC_ARGS+=("--depth-camera=${DEPTH_CAMERA}")
    if   [[ "${CUVSLAM_ODOM}" == true ]]; then REC_ARGS+=("--cuvslam-odom")
    elif [[ "${RGBD_ODOM}"    == true ]]; then REC_ARGS+=("--rgbd-odom")
    elif [[ "${VINS_ODOM}"    == true ]]; then REC_ARGS+=("--vins-odom")
    elif [[ "${T265_ODOM}"    == true ]]; then REC_ARGS+=("--t265-odom")
    fi
    [[ "${RECORD_COMPRESS_FISHEYE}" == "true"  ]] && REC_ARGS+=("--compress-fisheye")
    [[ "${RECORD_COMPRESS_FISHEYE}" == "false" ]] && REC_ARGS+=("--no-compress-fisheye")

    # Append after current window. -t "${SESSION}" alone is ambiguous when
    # window 0 is also named "jetson" (tmux resolves the target as a window
    # and refuses to overwrite it); -a places the new window immediately
    # after the active one.
    tmux new-window -a -t "${SESSION}:0" -n "record"
    PANE_RECORD="$(tmux display-message -p -t "${SESSION}:record.0" "#{pane_id}")"
    tmux send-keys -t "${PANE_RECORD}" \
        "${SCRIPT_DIR}/record_bag.sh ${REC_ARGS[*]}" Enter
fi

# ── Report HW mode to Control Center ─────────────────────────────────
# Jetson session = all real hardware. Report after a short delay so CC
# is ready to receive.
if [[ "${ENABLE_TELEMETRY}" == true ]]; then
    CC_URL="http://${BROKER_HOST}:8080"
    (
        sleep 10
        for _ in 1 2 3 4 5 6; do
            if curl -sf -X POST "${CC_URL}/api/hw_mode" \
                -H 'Content-Type: application/json' \
                -d '{"camera":"hw","t265":"hw","px4":"hw","jetson":"hw"}' \
                >/dev/null 2>&1; then
                exit 0
            fi
            sleep 5
        done
    ) &
fi

if [[ "${ATTACH}" == true && -t 1 ]]; then
    tmux attach-session -t "${SESSION}"
else
    echo "Tmux session '${SESSION}' created."
    echo "Attach with: tmux attach-session -t ${SESSION}"
fi
