#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")"/../.. && pwd)"
ARTIFACTS_DIR="$ROOT_DIR/artifacts/ai"
mkdir -p "$ARTIFACTS_DIR"

PROMPT_FILE="${PROMPT_FILE:-$ROOT_DIR/docs/prompts/planner.md}"

read_frontmatter() {
	awk '/^---/{flag=!flag; next} flag {print}' "$PROMPT_FILE" || true
}

ROLE=$(read_frontmatter | awk -F': ' '/^role:/ {print $2}' || true)
ASSIGNED_TO=$(read_frontmatter | awk -F': ' '/^assigned_to:/ {print $2}' || true)
DUE_DATE_FM=$(read_frontmatter | awk -F': ' '/^due_date:/ {print $2}' || true)
TRACKING_ISSUE_FM=$(read_frontmatter | awk -F': ' '/^tracking_issue:/ {print $2}' || true)
DUE_DATE=${DUE_DATE:-$DUE_DATE_FM}
TRACKING_ISSUE=${TRACKING_ISSUE:-$TRACKING_ISSUE_FM}

REPORT="$ARTIFACTS_DIR/planner_report.md"
{
	echo "# Planner Report"
	echo "Prompt: ${PROMPT_FILE#$ROOT_DIR/}"
	echo "Role: ${ROLE:-Planner}"
	echo "Assigned To: ${ASSIGNED_TO:-Copilot}"
	[[ -n "$DUE_DATE" ]] && echo "Due Date: $DUE_DATE"
	[[ -n "$TRACKING_ISSUE" ]] && echo "Tracking Issue: $TRACKING_ISSUE"
	echo
	echo "## Checks"
	for f in "$ROOT_DIR/docs/architecture/overview.md" "$ROOT_DIR/docs/architecture/node_graph.md" "$ROOT_DIR/docs/architecture/interfaces.md" "$ROOT_DIR/docs/architecture/failure_modes.md"; do
		rel=${f#$ROOT_DIR/}
		if [[ -s "$f" ]]; then echo "- Present: $rel"; else echo "- MISSING: $rel"; fi
	done
	echo
	echo "## Suggested Updates"
	echo "- Ensure topics/services and message types are explicit in interfaces.md"
	echo "- Document lifecycle transitions and startup order in overview.md"
	echo "- Add failure scenarios and mitigations in failure_modes.md"
	echo
	echo "## Links"
	echo "- Nav2 config: src/navigation/config/nav2_ackermann.yaml"
	echo "- Controller params: src/ackermann_control/params/ackermann.yaml"
} > "$REPORT"

echo "Planner report written to ${REPORT#$ROOT_DIR/}"
