#!/usr/bin/env bash
set -euo pipefail

# Export docs context for AI workflows.
# Creates artifacts/ai/context.tar.gz containing docs/, OWNERS.yaml, TEAMS.yaml, and README.md

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")"/../.. && pwd)"
ARTIFACTS_DIR="$ROOT_DIR/artifacts/ai"
mkdir -p "$ARTIFACTS_DIR"

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

copy_safe() {
	local src="$1"; local dest="$2"
	if [[ -e "$src" ]]; then
		mkdir -p "$dest"
		cp -R "$src" "$dest/"
	fi
}

copy_safe "$ROOT_DIR/docs" "$TMP_DIR"
copy_safe "$ROOT_DIR/README.md" "$TMP_DIR"

tar -czf "$ARTIFACTS_DIR/context.tar.gz" -C "$TMP_DIR" .
echo "Exported context to $ARTIFACTS_DIR/context.tar.gz"
