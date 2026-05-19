#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SUBMODULE_DIR="${REPO_ROOT}/src/px4-ros2-interface-lib"

# Patches applied in order. Each patch must be independently apply-able and
# apply-reverse-able (idempotent).
PATCH_FILES=(
    "${REPO_ROOT}/patches/px4-ros2-interface-lib-colcon-ignore.patch"
    "${REPO_ROOT}/patches/px4-ros2-interface-lib-arming-check-watchdog.patch"
)

if [[ ! -d "${SUBMODULE_DIR}/.git" && ! -f "${SUBMODULE_DIR}/.git" ]]; then
    echo "px4-ros2-interface-lib submodule is missing at ${SUBMODULE_DIR}" >&2
    exit 1
fi

apply_one() {
    local patch_file="$1"
    local label
    label="$(basename "${patch_file}")"

    if [[ ! -f "${patch_file}" ]]; then
        echo "Patch file is missing at ${patch_file}" >&2
        return 1
    fi

    if git -C "${SUBMODULE_DIR}" apply --check "${patch_file}" >/dev/null 2>&1; then
        git -C "${SUBMODULE_DIR}" apply "${patch_file}"
        echo "Applied ${label}."
        return 0
    fi

    if git -C "${SUBMODULE_DIR}" apply --reverse --check "${patch_file}" >/dev/null 2>&1; then
        echo "${label} is already applied."
        return 0
    fi

    echo "${label} could not be applied cleanly. Inspect ${SUBMODULE_DIR} for local conflicts." >&2
    return 1
}

for p in "${PATCH_FILES[@]}"; do
    apply_one "${p}" || exit 1
done
