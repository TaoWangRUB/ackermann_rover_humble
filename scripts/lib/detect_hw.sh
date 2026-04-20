#!/usr/bin/env bash
# Hardware detection helpers for auto-detecting cameras and PX4.
#
# Source this file from scripts that need auto-detection:
#   source "${SCRIPT_DIR}/lib/detect_hw.sh"
#
# Functions:
#   detect_realsense_cameras  — sets HW_HAS_D435I, HW_HAS_T265, HW_HAS_L515
#   detect_px4_dds            — sets HW_HAS_PX4_DDS
#   report_hw_mode_to_cc      — POST detection results to CC /api/hw_mode
#
# All detection runs INSIDE the Docker container via dcomp/docker exec.
# Requires dc.sh to be sourced first (for dcomp).

[[ -n "${_DETECT_HW_LOADED:-}" ]] && return 0
_DETECT_HW_LOADED=1

# ── Camera detection ──────────────────────────────────────────────────
# Uses rs-enumerate-devices inside the container to detect connected
# RealSense cameras by product line.
#
# Sets:
#   HW_HAS_D435I  — "true" if D435i detected
#   HW_HAS_T265   — "true" if T265 detected
#   HW_HAS_L515   — "true" if L515 detected
detect_realsense_cameras() {
    HW_HAS_D435I="false"
    HW_HAS_T265="false"
    HW_HAS_L515="false"

    local rs_output
    rs_output=$(dcomp exec -T ackermann_slam bash -c \
        'rs-enumerate-devices --compact 2>/dev/null' 2>/dev/null) || return 0

    if echo "${rs_output}" | grep -qi "D435I"; then
        HW_HAS_D435I="true"
    fi
    if echo "${rs_output}" | grep -qi "T265"; then
        HW_HAS_T265="true"
    fi
    if echo "${rs_output}" | grep -qi "L515"; then
        HW_HAS_L515="true"
    fi
}

# ── PX4 DDS detection ────────────────────────────────────────────────
# Checks if PX4 ROS 2 topics are available via XRCE-DDS by looking for
# the vehicle_status topic. This only works if the XRCE agent is
# already running and PX4 is connected.
#
# Sets:
#   HW_HAS_PX4_DDS  — "true" if PX4 DDS topics found
detect_px4_dds() {
    HW_HAS_PX4_DDS="false"

    local topics
    topics=$(dcomp exec -T ackermann_slam bash -c \
        'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash 2>/dev/null && timeout 3 ros2 topic list 2>/dev/null' 2>/dev/null) || return 0

    if echo "${topics}" | grep -q "/fmu/out/vehicle_status"; then
        HW_HAS_PX4_DDS="true"
    fi
}

# ── Summary printer ──────────────────────────────────────────────────
# Call after detection to print what was found.
print_hw_detection() {
    echo "Hardware detection:"
    echo "  D435i:   ${HW_HAS_D435I:-unknown}"
    echo "  T265:    ${HW_HAS_T265:-unknown}"
    echo "  L515:    ${HW_HAS_L515:-unknown}"
    echo "  PX4 DDS: ${HW_HAS_PX4_DDS:-unknown}"
    echo ""
}

# ── Report to Control Center ─────────────────────────────────────────
# POST detection results to the CC dashboard so the UI can label
# each component as HW or MOCK.
#
# Args:
#   $1 — CC base URL (default: http://localhost:8080)
report_hw_mode_to_cc() {
    local cc_url="${1:-http://localhost:8080}"
    local cam_mode="mock" px4_mode="mock"

    if [[ "${HW_HAS_D435I:-false}" == "true" || "${HW_HAS_L515:-false}" == "true" ]]; then
        cam_mode="hw"
    fi
    if [[ "${HW_HAS_PX4_DDS:-false}" == "true" ]]; then
        px4_mode="hw"
    fi

    curl -sf -X POST "${cc_url}/api/hw_mode" \
        -H 'Content-Type: application/json' \
        -d "{
            \"camera\": \"${cam_mode}\",
            \"t265\": \"$([ "${HW_HAS_T265:-false}" == "true" ] && echo hw || echo mock)\",
            \"px4\": \"${px4_mode}\",
            \"jetson\": \"mock\"
        }" >/dev/null 2>&1 || true
}
