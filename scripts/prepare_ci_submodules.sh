#!/usr/bin/env bash
set -euo pipefail

if [[ "${CI:-}" != "true" ]]; then
    echo "scripts/prepare_ci_submodules.sh is intended for CI runners only." >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VINS_DIR="${REPO_ROOT}/src/VINS-Fusion"
VINS_URL="https://github.com/zinuok/VINS-Fusion-ROS2.git"
VINS_REF="main"

cd "${REPO_ROOT}"

# The root repo currently records a local-only VINS-Fusion gitlink. In CI we
# reconstruct the intended tree from a fetchable upstream ref plus the tracked
# compatibility patch.
git submodule sync --recursive
git submodule update --init --depth 1 src/px4-ros2-interface-lib src/px4_msgs

rm -rf "${VINS_DIR}"
git clone --depth 1 --branch "${VINS_REF}" "${VINS_URL}" "${VINS_DIR}"

bash scripts/apply_vins_fusion_patch.sh
