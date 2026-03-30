#!/usr/bin/env bash
# Shared docker-compose helper — compatible with v1.25+ and v2.x.
#
# Source this file from any script under scripts/:
#   SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
#   source "${SCRIPT_DIR}/lib/dc.sh"   # or "${SCRIPT_DIR}/../scripts/lib/dc.sh"
#
# After sourcing you get:
#   PROJECT_DIR   — repo root
#   COMPOSE_FILE  — path to docker/docker-compose.yml
#   dcomp()          — drop-in replacement for "docker-compose --env-file … -f …"
#
# Why: docker-compose v1.25 does NOT support --env-file (added in v1.28).
#      v2 loads .env from the compose-file directory, not CWD.
#      Exporting the vars into the shell works for every version.

# Guard against double-sourcing
[[ -n "${_DC_LOADED:-}" ]] && return 0
_DC_LOADED=1

# ── resolve paths ──────────────────────────────────────────────────────
# Walk up from this file (scripts/lib/dc.sh) to the repo root.
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
COMPOSE_FILE="${PROJECT_DIR}/docker/docker-compose.yml"

# ── export .env into the current shell ─────────────────────────────────
_DC_ENV_FILE="${PROJECT_DIR}/.env"
if [[ -f "${_DC_ENV_FILE}" ]]; then
    set -a
    # shellcheck source=/dev/null
    source "${_DC_ENV_FILE}"
    set +a
fi

# ── runtime defaults (not in .env) ─────────────────────────────────────
export ARCH="${ARCH:-$(uname -m)}"
export USERNAME="${USERNAME:-$(id -un)}"
export USER_UID="${USER_UID:-$(id -u)}"
export USER_GID="${USER_GID:-$(id -g)}"

# ── wrapper command selection ──────────────────────────────────────────
# Support both Compose v2 (`docker compose`) and Compose v1 (`docker-compose`).
# Some snap-based Docker installs ship a broken `/snap/bin/docker-compose`
# wrapper for non-interactive shells while the real v2 plugin binary still
# exists on disk, so keep a direct-plugin fallback as well.
if docker compose version >/dev/null 2>&1; then
    DC_CMD=(docker compose)
elif command -v docker-compose >/dev/null 2>&1 && docker-compose version >/dev/null 2>&1; then
    DC_CMD=(docker-compose)
elif [[ -x /snap/docker/current/usr/libexec/docker/cli-plugins/docker-compose ]] \
    && /snap/docker/current/usr/libexec/docker/cli-plugins/docker-compose version >/dev/null 2>&1; then
    DC_CMD=(/snap/docker/current/usr/libexec/docker/cli-plugins/docker-compose)
else
    echo "No working Compose command found (checked 'docker compose', 'docker-compose', and snap plugin path)." >&2
    return 1 2>/dev/null || exit 1
fi

# ── wrappers ───────────────────────────────────────────────────────────
# dcomp  — run docker compose (returns to caller; safe for multi-call scripts)
# xdcomp — exec docker compose (replaces current process; use as last command)
#           Needed because `exec` only works with external commands, not functions.
dcomp() {
    "${DC_CMD[@]}" -f "${COMPOSE_FILE}" "$@"
}
xdcomp() {
    exec "${DC_CMD[@]}" -f "${COMPOSE_FILE}" "$@"
}
