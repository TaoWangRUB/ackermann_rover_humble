#!/usr/bin/env bash
# ============================================================================
# Upload PX4 parameters via MAVLink shell — reliable alternative to QGC file load.
#
# Usage (from host, with Cube Black connected via USB):
#   ./scripts/upload_params.sh                       # upload all params (unidirectional ESC)
#   ./scripts/upload_params.sh --reversible-drive    # upload with bidirectional ESC PWM
#   ./scripts/upload_params.sh --verify-only         # just verify current values
#
# Why use this instead of QGC "Load from file"?
#   1. QGC's .params parser silently skips lines with wrong whitespace.
#   2. SYS_AUTOSTART changes trigger a full parameter reset on reboot,
#      overwriting other params loaded in the same batch.
#   3. This script uses a two-pass approach:
#      Pass 1: Set SYS_AUTOSTART → reboot → wait for reconnect
#      Pass 2: Set all remaining params → verify each one
#
# Prerequisites:
#   - Docker container running (docker-compose up -d)
#   - Cube Black connected via USB (/dev/ttyACM0)
#   - px4_cmd.sh working (test with: ./scripts/px4_cmd.sh 'ver all')
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PX4_CMD="${SCRIPT_DIR}/px4_cmd.sh"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

VERIFY_ONLY=false
REVERSIBLE_DRIVE=false
for arg in "$@"; do
    case "${arg}" in
        --verify-only)      VERIFY_ONLY=true ;;
        --reversible-drive) REVERSIBLE_DRIVE=true ;;
    esac
done

# ── Parameter source ──────────────────────────────────────────────────────────
# Single source of truth: the QGC-compatible .params file.
# Format per line (tab-separated):  COMPID  COMPID  NAME  VALUE  TYPECODE
#   TYPECODE: 6 = int32, 9 = float
# Lines starting with # and blank lines are skipped.
# SYS_AUTOSTART is handled separately in pass 1 (requires reboot before others).

PARAMS_FILE="${PROJECT_DIR}/src/px4_bringup/config/cube_black_ackermann.params"

if [[ ! -f "${PARAMS_FILE}" ]]; then
    echo "ERROR: Parameter file not found: ${PARAMS_FILE}"
    exit 1
fi

# Parse .params file into arrays
AIRFRAME_PARAM=""
PARAMS=()

while IFS=$'\t' read -r _cid1 _cid2 name value typecode _rest; do
    # Skip comments and blank lines
    [[ -z "${name}" || "${_cid1}" == \#* ]] && continue

    # Map QGC type code to int/float
    local_type="int"
    if [[ "${typecode}" == "9" ]]; then
        local_type="float"
    fi

    if [[ "${name}" == "SYS_AUTOSTART" ]]; then
        AIRFRAME_PARAM="${name} ${value} ${local_type}"
    else
        PARAMS+=("${name} ${value} ${local_type}")
    fi
done < "${PARAMS_FILE}"

if [[ -z "${AIRFRAME_PARAM}" ]]; then
    echo "ERROR: SYS_AUTOSTART not found in ${PARAMS_FILE}"
    exit 1
fi

# ── ESC-dependent PWM values ──────────────────────────────────────────────────
# PX4 "Throttle" maps [-1, 1] → [PWM_MIN, PWM_MAX].
# Unidirectional: MIN=1000 (=DIS) so throttle -1 → ESC stop, +1 → full fwd
# Bidirectional:  MIN=1100, MAX=1900, center 1500 = ESC stop
if [[ "${REVERSIBLE_DRIVE}" == true ]]; then
    PARAMS+=("PWM_MAIN_MIN1 1100 int")
    PARAMS+=("PWM_MAIN_MAX1 1900 int")
    echo "ESC mode: bidirectional (PWM_MIN=1100, PWM_MAX=1900, center=1500=stop)"
else
    PARAMS+=("PWM_MAIN_MIN1 1000 int")
    PARAMS+=("PWM_MAIN_MAX1 2000 int")
    echo "ESC mode: unidirectional (PWM_MIN=1000=DIS=stop, PWM_MAX=1900)"
fi

echo "Loaded ${#PARAMS[@]} parameters (+1 airframe) from ${PARAMS_FILE##*/}"

# ── Helper functions ─────────────────────────────────────────────────────────

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m'  # No Color

print_connectivity_help() {
        cat <<EOF
ERROR: Cannot reach PX4 over MAVLink shell on /dev/ttyACM0.

What this usually means:
    1. PX4 USB CDC is present, but MAVLink is not responding yet.
    2. On a fresh/factory-reset Cube Black, free RAM is often too low for reliable USB MAVLink.
    3. After a reboot, the USB CDC endpoint may need a physical USB replug to recover.

Recommended recovery:
    1. Replug the Cube Black USB cable and wait for /dev/ttyACM0 to reappear.
    2. If this is a fresh board or recent factory reset, use QGC for the first two-pass load of:
         src/px4_bringup/config/cube_black_ackermann.params
      Make sure SYS_USB_AUTO=2 and USB_MAV_MODE=2 are loaded as well.
    3. Reboot, replug USB, then rerun:
         ./scripts/upload_params.sh --verify-only

Expected once bootstrapped:
    ./scripts/px4_cmd.sh 'free' 8
    # expect at least ~28 KB free RAM
EOF
}

px4_param_set() {
    local name="$1" value="$2"
    "${PX4_CMD}" "param set ${name} ${value}" 8 2>/dev/null
}

px4_param_show() {
    local name="$1"
    "${PX4_CMD}" "param show ${name}" 8 2>/dev/null
}

verify_param() {
    local name="$1" expected="$2" type="$3"
    local output
    output=$(px4_param_show "${name}" 2>/dev/null) || true

    # Extract the current value from `param show` output.
    # Output format: "x + SYS_AUTOSTART [711,1434] : 51000"
    local current
    current=$(echo "${output}" | grep -oP ':\s+\K[-0-9.e+]+' | tail -1) || true

    if [[ -z "${current}" ]]; then
        printf "  ${RED}FAIL${NC}  %-25s  expected=%-10s  actual=<not found>\n" "${name}" "${expected}"
        return 1
    fi

    local match=false
    if [[ "${type}" == "int" ]]; then
        # Integer comparison (strip decimals from both sides)
        local exp_int cur_int
        exp_int=$(printf "%.0f" "${expected}" 2>/dev/null || echo "${expected}")
        cur_int=$(printf "%.0f" "${current}" 2>/dev/null || echo "${current}")
        [[ "${exp_int}" == "${cur_int}" ]] && match=true
    else
        # Float comparison with tolerance
        local diff
        diff=$(awk "BEGIN { d = ${current} - ${expected}; print (d < 0 ? -d : d) }")
        (( $(awk "BEGIN { print (${diff} < 0.001) }") )) && match=true
    fi

    if ${match}; then
        printf "  ${GREEN}OK${NC}    %-25s  value=%s\n" "${name}" "${current}"
        return 0
    else
        printf "  ${RED}FAIL${NC}  %-25s  expected=%-10s  actual=%s\n" "${name}" "${expected}" "${current}"
        return 1
    fi
}

wait_for_heartbeat() {
    local max_wait="${1:-60}"
    echo "Waiting for PX4 heartbeat (up to ${max_wait}s)..."
    for i in $(seq 1 "${max_wait}"); do
        if "${PX4_CMD}" "ver hwcmp" 4 >/dev/null 2>&1; then
            echo "PX4 reconnected after ${i}s."
            return 0
        fi
        sleep 1
    done
    echo "ERROR: PX4 did not reconnect within ${max_wait}s"
    return 1
}

# ── Main ─────────────────────────────────────────────────────────────────────

echo "============================================"
echo "  PX4 Parameter Upload — Cube Black"
echo "============================================"
echo ""

# Connectivity check
echo "Checking PX4 connectivity..."
if ! "${PX4_CMD}" "ver hwcmp" 5 >/dev/null 2>&1; then
    print_connectivity_help
    exit 1
fi
echo "PX4 connected."
echo ""

if ${VERIFY_ONLY}; then
    echo "=== Verify-only mode ==="
    echo ""
    FAILURES=0

    # Verify airframe
    read -r name value type <<< "${AIRFRAME_PARAM}"
    verify_param "${name}" "${value}" "${type}" || ((FAILURES++)) || true

    # Verify all params
    for entry in "${PARAMS[@]}"; do
        read -r name value type <<< "${entry}"
        verify_param "${name}" "${value}" "${type}" || ((FAILURES++)) || true
    done

    echo ""
    if [[ ${FAILURES} -eq 0 ]]; then
        printf "${GREEN}All parameters verified OK.${NC}\n"
    else
        printf "${RED}${FAILURES} parameter(s) do not match expected values.${NC}\n"
        exit 1
    fi
    exit 0
fi

# ── Pass 1: Airframe ─────────────────────────────────────────────────────────
echo "=== Pass 1: Airframe (SYS_AUTOSTART) ==="

read -r name value type <<< "${AIRFRAME_PARAM}"
current_airframe=$(px4_param_show "${name}" 2>/dev/null | grep -oP ':\s+\K[-0-9.e+]+' | tail -1) || true

if [[ "${current_airframe}" == "${value}" ]]; then
    echo "SYS_AUTOSTART already set to ${value}, skipping reboot."
else
    echo "Setting SYS_AUTOSTART=${value} (current: ${current_airframe})"
    px4_param_set "${name}" "${value}"
    echo "Rebooting PX4 to apply airframe change..."
    "${PX4_CMD}" "reboot" 3 || true
    sleep 5
    wait_for_heartbeat 60
fi
echo ""

# ── Pass 2: All remaining parameters ─────────────────────────────────────────
echo "=== Pass 2: Setting all parameters ==="

REBOOT_NEEDED=false

for entry in "${PARAMS[@]}"; do
    read -r name value type <<< "${entry}"
    echo -n "  Setting ${name}=${value} ... "
    px4_param_set "${name}" "${value}"

    # Flag reboot-required params
    case "${name}" in
        UXRCE_DDS_CFG|SER_TEL2_BAUD)
            REBOOT_NEEDED=true
            ;;
    esac

    echo "done"
done
echo ""

if ${REBOOT_NEEDED}; then
    echo "Reboot-required parameters changed (UXRCE_DDS_CFG, SER_TEL2_BAUD)."
    echo "Rebooting PX4..."
    "${PX4_CMD}" "reboot" 3 || true
    sleep 5
    wait_for_heartbeat 60
    echo ""
fi

# ── Pass 3: Verification ─────────────────────────────────────────────────────
echo "=== Verification ==="
echo ""
FAILURES=0

read -r name value type <<< "${AIRFRAME_PARAM}"
verify_param "${name}" "${value}" "${type}" || ((FAILURES++)) || true

for entry in "${PARAMS[@]}"; do
    read -r name value type <<< "${entry}"
    verify_param "${name}" "${value}" "${type}" || ((FAILURES++)) || true
done

echo ""
if [[ ${FAILURES} -eq 0 ]]; then
    printf "${GREEN}All ${#PARAMS[@]} parameters verified OK.${NC}\n"
else
    printf "${RED}${FAILURES} parameter(s) failed verification!${NC}\n"
    echo "Run again or set failed params manually via:"
    echo "  ./scripts/px4_cmd.sh 'param set <NAME> <VALUE>'"
    exit 1
fi

echo ""
echo "Done. Parameters are persistent across reboots."
