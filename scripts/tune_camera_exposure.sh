#!/usr/bin/env bash
# Bring up the camera (or use an already-running one), sweep exposure × gain,
# and print the recommended (exposure, gain) for the current scene.
#
# Designed for environment-specific tuning (kitchen, parking lot, hallway).
# The camera should be STATIONARY pointed at a representative view of the
# scene before you run this.
#
# Usage:
#   ./scripts/tune_camera_exposure.sh                    # 'normal' preset, d435i
#   ./scripts/tune_camera_exposure.sh --preset=dark      # very dim (parking lot, night)
#   ./scripts/tune_camera_exposure.sh --preset=dim       # dim indoor (single lamp / dusk)
#   ./scripts/tune_camera_exposure.sh --preset=bright    # bright daylight (windows + sun)
#   ./scripts/tune_camera_exposure.sh --camera=l515 --preset=dim
#   ./scripts/tune_camera_exposure.sh --save-frames      # also save one frame per cell
#   ./scripts/tune_camera_exposure.sh --apply            # automatically apply the winner
#
# Flags:
#   --preset=NAME       bright | normal | dim | dark   (default: normal)
#   --camera=NAME       d435i | l515                   (default: d435i)
#   --save-frames       save a representative frame per (exp, gain) to /tmp/sweep_frames/
#   --apply             after sweep, automatically set the recommended (exp, gain)
#
# Prereqs: the camera must be running and publishing /<camera>/color/image_raw.
# Easiest way: start the rover stack first, e.g.:
#   ./scripts/start_ros2_nodes.sh --hw --no-rviz --depth-camera=d435i --t265-odom
# OR ./scripts/start_jetson_session.sh --t265-odom --with-telemetry
set -euo pipefail

PRESET="normal"
CAMERA="d435i"
SAVE_FRAMES=""
APPLY=false

for arg in "$@"; do
    case "${arg}" in
        --preset=*) PRESET="${arg#--preset=}" ;;
        --camera=*) CAMERA="${arg#--camera=}" ;;
        --save-frames) SAVE_FRAMES="--save-frames" ;;
        --apply)    APPLY=true ;;
        -h|--help)  sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"; exit 0 ;;
        *) echo "unknown arg: ${arg}" >&2; exit 2 ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"

# Sanity: container running?
if ! dcomp ps --services --filter status=running 2>/dev/null \
        | grep -q '^ackermann_slam$'; then
    echo "Container not running. Start it first:" >&2
    echo "  ./scripts/start_ros2_nodes.sh --hw --no-rviz --depth-camera=${CAMERA} --t265-odom" >&2
    echo "or" >&2
    echo "  ./scripts/start_jetson_session.sh --t265-odom --with-telemetry" >&2
    exit 1
fi

# Sanity: camera node running?
NODE_OK=$(dcomp exec -T ackermann_slam bash -lc "
    source /opt/ros/jazzy/setup.bash 2>/dev/null
    source /workspace/install/setup.bash 2>/dev/null
    ros2 node list 2>/dev/null | grep -qx '/${CAMERA}' && echo OK || echo MISSING
" 2>&1 | grep -E '^(OK|MISSING)$' | head -1)

if [[ "${NODE_OK}" != "OK" ]]; then
    echo "Camera node /${CAMERA} not running. Start the rover stack first." >&2
    echo "Then re-run this script with the camera POINTED AT A REPRESENTATIVE SCENE and STATIONARY." >&2
    exit 1
fi

echo "─────────────────────────────────────────────────────────────"
echo "  Exposure × gain sweep"
echo "  preset:   ${PRESET}"
echo "  camera:   ${CAMERA}"
echo "  save frames: $( [[ -n "${SAVE_FRAMES}" ]] && echo yes || echo no )"
echo "  apply best:  ${APPLY}"
echo "─────────────────────────────────────────────────────────────"
echo
echo "  ⚠ Camera should be STATIONARY pointed at a representative scene."
echo "    Sweep takes ~2-3 minutes (28 cells × ~5.5s each)."
echo

# Run the sweep inside the container
SWEEP_CMD="python3 /workspace/scripts/sweep_exp_gain.py --preset=${PRESET} --camera=${CAMERA} ${SAVE_FRAMES}"

if [[ "${APPLY}" == true ]]; then
    # Run the sweep, capture stdout, parse winner, apply via ros2 param set
    OUT=$(dcomp exec -T ackermann_slam bash -lc "
        source /opt/ros/jazzy/setup.bash 2>/dev/null
        source /workspace/install/setup.bash 2>/dev/null
        ${SWEEP_CMD}
    ")
    echo "${OUT}"
    # Parse the winner — first 'exp=NNNN gain=NNN' under 'Top 3 recommended'
    WIN=$(echo "${OUT}" | awk '
        /Top 3 recommended/ { capture=1; next }
        capture && /exp=/ { print; exit }
    ')
    if [[ -n "${WIN}" ]]; then
        EXP=$(echo "${WIN}" | grep -oE 'exp= *[0-9]+' | grep -oE '[0-9]+')
        GAIN=$(echo "${WIN}" | grep -oE 'gain= *[0-9]+' | grep -oE '[0-9]+')
        if [[ -n "${EXP}" && -n "${GAIN}" ]]; then
            echo
            echo "Applying winner: exposure=${EXP} gain=${GAIN}..."
            dcomp exec -T ackermann_slam bash -lc "
                source /opt/ros/jazzy/setup.bash 2>/dev/null
                source /workspace/install/setup.bash 2>/dev/null
                ros2 param set /${CAMERA} rgb_camera.enable_auto_exposure false
                ros2 param set /${CAMERA} rgb_camera.exposure ${EXP}
                ros2 param set /${CAMERA} rgb_camera.gain     ${GAIN}
            " 2>&1 | grep -v 'ROS_DDS_MODE'
            echo "Done. Camera now at exposure=${EXP} gain=${GAIN}."
        else
            echo
            echo "Could not parse winner from sweep output. Apply manually."
        fi
    else
        echo
        echo "No OK cell found. Check the sweep output above for the closest match,"
        echo "then apply manually with ros2 param set commands."
    fi
else
    dcomp exec -T ackermann_slam bash -lc "
        source /opt/ros/jazzy/setup.bash 2>/dev/null
        source /workspace/install/setup.bash 2>/dev/null
        ${SWEEP_CMD}
    "
fi
