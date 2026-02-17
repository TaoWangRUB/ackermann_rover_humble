#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")"/../.. && pwd)"
ARTIFACTS_DIR="$ROOT_DIR/artifacts/ai"
mkdir -p "$ARTIFACTS_DIR"

PROMPT_FILE="${PROMPT_FILE:-$ROOT_DIR/docs/prompts/safety.md}"

read_frontmatter() {
	awk '/^---/{flag=!flag; next} flag {print}' "$PROMPT_FILE" || true
}

ROLE=$(read_frontmatter | awk -F': ' '/^role:/ {print $2}' || true)
ASSIGNED_TO=$(read_frontmatter | awk -F': ' '/^assigned_to:/ {print $2}' || true)
DUE_DATE_FM=$(read_frontmatter | awk -F': ' '/^due_date:/ {print $2}' || true)
TRACKING_ISSUE_FM=$(read_frontmatter | awk -F': ' '/^tracking_issue:/ {print $2}' || true)
DUE_DATE=${DUE_DATE:-$DUE_DATE_FM}
TRACKING_ISSUE=${TRACKING_ISSUE:-$TRACKING_ISSUE_FM}

REPORT="$ARTIFACTS_DIR/safety_report.md"

S_REQ_FILE="$ROOT_DIR/docs/requirements/safety.md"
REQS=$(grep -E '^[SP][0-9]+:' "$S_REQ_FILE" | sed 's/^/- /' || true)

WATCHDOG="$ROOT_DIR/src/safety/src/watchdog_node.cpp"
PARAMS="$ROOT_DIR/src/ackermann_control/params/ackermann.yaml"

{
	echo "# Safety Report"
	echo "Prompt: ${PROMPT_FILE#$ROOT_DIR/}"
	echo "Role: ${ROLE:-Safety Reviewer}"
	echo "Assigned To: ${ASSIGNED_TO:-Copilot}"
	[[ -n "$DUE_DATE" ]] && echo "Due Date: $DUE_DATE"
	[[ -n "$TRACKING_ISSUE" ]] && echo "Tracking Issue: $TRACKING_ISSUE"
	echo
	echo "## Requirements"
	echo "$REQS"
	echo
	echo "## File Presence"
	for f in "$WATCHDOG" "$PARAMS"; do
		rel=${f#$ROOT_DIR/}
		if [[ -e "$f" ]]; then echo "- Present: $rel"; else echo "- MISSING: $rel"; fi
	done
	echo
	echo "## Suggested Safety Checks"
	echo "- Add fault injection tests verifying watchdog triggers safe stop"
	echo "- Validate parameter bounds: wheelbase, max_steering_angle, max_speed"
	echo "- Document failure modes and mitigations in docs/architecture/failure_modes.md"
} > "$REPORT"

echo "Safety report written to ${REPORT#$ROOT_DIR/}"
