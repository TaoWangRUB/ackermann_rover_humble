#!/usr/bin/env bash
# Test the Control Center stack end-to-end.
#
# Verifies telemetry flow, sends commands, checks ACK tracking,
# and confirms telemetry continues after commands.
#
# Prerequisites:
#   - Control Center stack running (docker compose up)
#   - rover_monitor running with --with-telemetry
#
# Usage:
#   ./scripts/test_control_center.sh
#   CC_URL=http://192.168.1.10:8080 ./scripts/test_control_center.sh
set -uo pipefail

for arg in "$@"; do
    case "${arg}" in
        -h|--help)
            sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
            exit 0 ;;
    esac
done

CC_URL="${CC_URL:-http://localhost:8080}"
PASS=0
FAIL=0

green()  { printf '\033[0;32m%s\033[0m\n' "$*"; }
red()    { printf '\033[0;31m%s\033[0m\n' "$*"; }
header() { printf '\n\033[1;34m── %s ──\033[0m\n' "$*"; }

check() {
    local desc="$1" ok="$2"
    if [[ "${ok}" == "true" ]]; then
        green "  ✓ ${desc}"
        PASS=$((PASS + 1))
    else
        red "  ✗ ${desc}"
        FAIL=$((FAIL + 1))
    fi
}

# ── 1. Telemetry flow ─────────────────────────────────────────────────
header "Telemetry Flow"

STATUS=$(curl -sf "${CC_URL}/api/status" 2>/dev/null || echo "{}")
for key in health cam px4 jetson; do
    stale=$(echo "${STATUS}" | python3 -c "
import sys, json
d = json.load(sys.stdin)
e = d.get('${key}', {})
print(str(e.get('stale', True)).lower())
" 2>/dev/null || echo "true")
    if [[ "${stale}" == "false" ]]; then
        check "${key}: receiving data (stale=false)" "true"
    else
        check "${key}: receiving data (stale=false)" "false"
    fi
done

# ── 2. Send commands ──────────────────────────────────────────────────
header "Command Dispatch"

send_cmd() {
    local cmd_type="$1" expect_ok="${2:-true}"
    local resp
    resp=$(curl -sf -X POST "${CC_URL}/api/command" \
        -H 'Content-Type: application/json' \
        -d "{\"cmd_type\": \"${cmd_type}\", \"params\": {}}" 2>/dev/null || echo "{}")

    local status
    status=$(echo "${resp}" | python3 -c "
import sys, json
d = json.load(sys.stdin)
print(d.get('status', 'error'))
" 2>/dev/null || echo "error")

    if [[ "${expect_ok}" == "true" ]]; then
        if [[ "${status}" == "sent" || "${status}" == "ok" ]]; then
            check "${cmd_type}: accepted" "true"
        else
            check "${cmd_type}: accepted (got: ${status})" "false"
        fi
    else
        if [[ "${status}" == "rejected" || "${status}" == "error" ]]; then
            check "${cmd_type}: correctly rejected" "true"
        else
            check "${cmd_type}: correctly rejected (got: ${status})" "false"
        fi
    fi
}

send_cmd "estop" true
send_cmd "disarm" true
send_cmd "cancel_goal" true
send_cmd "arm" false  # mock PX4 battery is 10% < 20% threshold

sleep 1

# ── 3. Command history + ACK ─────────────────────────────────────────
header "Command History & ACK"

HISTORY=$(curl -sf "${CC_URL}/api/command/history" 2>/dev/null || echo "[]")
CMD_COUNT=$(echo "${HISTORY}" | python3 -c "
import sys, json
d = json.load(sys.stdin)
print(len(d) if isinstance(d, list) else 0)
" 2>/dev/null || echo "0")

if [[ "${CMD_COUNT}" -ge 3 ]]; then
    check "command history has ${CMD_COUNT} entries (≥3)" "true"
else
    check "command history has ${CMD_COUNT} entries (≥3)" "false"
fi

# Check that at least one ACK was received
ACK_COUNT=$(echo "${HISTORY}" | python3 -c "
import sys, json
d = json.load(sys.stdin)
count = sum(1 for c in d if c.get('ack_status') and c['ack_status'] != 'pending')
print(count)
" 2>/dev/null || echo "0")

if [[ "${ACK_COUNT}" -ge 1 ]]; then
    check "ACK received for ${ACK_COUNT} command(s)" "true"
else
    check "ACK received (may need more time)" "false"
fi

# ── 4. Telemetry continues after commands ─────────────────────────────
header "Post-Command Telemetry"

sleep 2
STATUS2=$(curl -sf "${CC_URL}/api/status" 2>/dev/null || echo "{}")
health_stale=$(echo "${STATUS2}" | python3 -c "
import sys, json
d = json.load(sys.stdin)
print(str(d.get('health', {}).get('stale', True)).lower())
" 2>/dev/null || echo "true")

if [[ "${health_stale}" == "false" ]]; then
    check "telemetry still flowing after commands" "true"
else
    check "telemetry still flowing after commands" "false"
fi

# ── Summary ───────────────────────────────────────────────────────────
header "Summary"
echo "  Passed: ${PASS}  Failed: ${FAIL}"

if [[ "${FAIL}" -gt 0 ]]; then
    red "  Some tests failed."
    exit 1
else
    green "  All tests passed."
fi
