#!/usr/bin/env bash
# ============================================================================
# Build PX4 Rover Firmware for Cube Black (FMUv3)
# ============================================================================
#
# This script:
#   1. Backs up PX4's default dds_topics.yaml
#   2. Copies the workspace's trimmed dds_topics.yaml into the PX4 source tree
#   3. Builds the firmware with:  make px4_fmu-v3_rover
#   4. Reports the binary size vs. 2 MB flash limit
#   5. Restores the original dds_topics.yaml on exit (even on failure)
#
# Usage (from host):
#   ./scripts/build_px4_rover_fw.sh              # build firmware
#   ./scripts/build_px4_rover_fw.sh --no-dds     # build without replacing dds_topics.yaml
#
# The resulting firmware (.px4) can be found at:
#   /px4/build/px4_fmu-v3_rover/px4_fmu-v3_rover.px4
#
# Flash via QGroundControl: Vehicle Setup → Firmware → Advanced → Custom firmware file
#
# Prerequisites:
#   - Docker container running
#   - PX4 source mounted at /px4 inside Docker
# ============================================================================
set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/dc.sh"

SKIP_DDS="${1:-}"
PX4_DDS_PATH="src/modules/uxrce_dds_client/dds_topics.yaml"
WORKSPACE_DDS="src/px4_bringup/config/dds_topics.yaml"
FLASH_SIZE_BYTES=$((2 * 1024 * 1024))   # 2 MB

echo "╔════════════════════════════════════════════════════════════╗"
echo "║  PX4 Rover Firmware Build — Cube Black (FMUv3, 2 MB)     ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

xdcomp exec ackermann_slam bash -c "
set -euo pipefail

git config --global --add safe.directory '*'
cd /px4

PX4_DDS=\"${PX4_DDS_PATH}\"
WS_DDS=\"/workspace/${WORKSPACE_DDS}\"
BACKUP_DDS=\"\${PX4_DDS}.bak\"

# ── Cleanup trap: always restore original dds_topics.yaml ────────────────
cleanup() {
    if [[ -f \"\${BACKUP_DDS}\" ]]; then
        echo ''
        echo '→ Restoring original dds_topics.yaml...'
        mv \"\${BACKUP_DDS}\" \"\${PX4_DDS}\"
        echo '  Done.'
    fi
}
trap cleanup EXIT INT TERM

# ── Step 1: Replace dds_topics.yaml (unless --no-dds) ────────────────────
if [[ '${SKIP_DDS}' != '--no-dds' ]]; then
    if [[ ! -f \"\${WS_DDS}\" ]]; then
        echo 'ERROR: Workspace dds_topics.yaml not found at:' \"\${WS_DDS}\"
        exit 1
    fi
    echo '→ Backing up PX4 dds_topics.yaml...'
    cp \"\${PX4_DDS}\" \"\${BACKUP_DDS}\"
    echo '→ Copying trimmed dds_topics.yaml into PX4 source tree...'
    cp \"\${WS_DDS}\" \"\${PX4_DDS}\"
    echo '  Source: ' \"\${WS_DDS}\"
    echo '  Target: ' \"\${PX4_DDS}\"
    echo ''
else
    echo '→ Skipping dds_topics.yaml replacement (--no-dds flag set)'
    echo ''
fi

# ── Step 2: Build firmware ────────────────────────────────────────────────
echo '→ Building firmware: make px4_fmu-v3_rover'
echo '  (this may take several minutes on first build...)'
echo ''

export GZ_DISTRO=harmonic
make px4_fmu-v3_rover

# ── Step 3: Report binary size ────────────────────────────────────────────
echo ''
echo '════════════════════════════════════════════════════════════'
FW_FILE='build/px4_fmu-v3_rover/px4_fmu-v3_rover.px4'
if [[ -f \"\${FW_FILE}\" ]]; then
    FW_SIZE=\$(stat -c%s \"\${FW_FILE}\")
    FW_SIZE_KB=\$(( FW_SIZE / 1024 ))
    FLASH_KB=\$(( ${FLASH_SIZE_BYTES} / 1024 ))
    PCT=\$(( FW_SIZE * 100 / ${FLASH_SIZE_BYTES} ))
    echo \"Firmware: \${FW_FILE}\"
    echo \"Size:     \${FW_SIZE_KB} KB / \${FLASH_KB} KB  (\${PCT}% of flash)\"

    if [[ \${FW_SIZE} -gt ${FLASH_SIZE_BYTES} ]]; then
        echo ''
        echo 'WARNING: Firmware exceeds 2 MB flash! Reduce modules or disable features.'
        exit 1
    else
        echo 'Status:   OK — fits in Cube Black 2 MB flash'
    fi
else
    echo 'WARNING: Firmware file not found at expected path.'
    echo 'Check build output above for errors.'
fi
echo '════════════════════════════════════════════════════════════'
echo ''
echo 'Flash via QGC: Vehicle Setup → Firmware → Advanced → Custom firmware file'
echo \"  File: /px4/\${FW_FILE}\"
"
