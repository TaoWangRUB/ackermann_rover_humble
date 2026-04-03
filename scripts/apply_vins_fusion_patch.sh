#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SUBMODULE_DIR="${REPO_ROOT}/src/VINS-Fusion"
PATCH_FILE="${REPO_ROOT}/patches/vins-fusion-ros2-jazzy.patch"

if [[ ! -d "${SUBMODULE_DIR}/.git" && ! -f "${SUBMODULE_DIR}/.git" ]]; then
    echo "VINS-Fusion submodule is missing at ${SUBMODULE_DIR}" >&2
    exit 1
fi

if [[ ! -f "${PATCH_FILE}" ]]; then
    echo "Patch file is missing at ${PATCH_FILE}" >&2
    exit 1
fi

if git -C "${SUBMODULE_DIR}" apply --check "${PATCH_FILE}" >/dev/null 2>&1; then
    git -C "${SUBMODULE_DIR}" apply "${PATCH_FILE}"
    echo "Applied VINS-Fusion ROS 2 Jazzy patch."
    exit 0
fi

if git -C "${SUBMODULE_DIR}" apply --reverse --check "${PATCH_FILE}" >/dev/null 2>&1; then
    echo "VINS-Fusion ROS 2 Jazzy patch is already applied."
    exit 0
fi

echo "VINS-Fusion patch could not be applied cleanly. Inspect ${SUBMODULE_DIR} for local conflicts." >&2
exit 1
