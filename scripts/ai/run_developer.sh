#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")"/../.. && pwd)"
ARTIFACTS_DIR="$ROOT_DIR/artifacts/ai"
mkdir -p "$ARTIFACTS_DIR"

PROMPT_FILE="${PROMPT_FILE:-$ROOT_DIR/docs/prompts/developer.md}"

read_frontmatter() {
  awk '/^---/{flag=!flag; next} flag {print}' "$PROMPT_FILE" || true
}

ROLE=$(read_frontmatter | awk -F': ' '/^role:/ {print $2}' || true)
ASSIGNED_TO=$(read_frontmatter | awk -F': ' '/^assigned_to:/ {print $2}' || true)
DUE_DATE_FM=$(read_frontmatter | awk -F': ' '/^due_date:/ {print $2}' || true)
TRACKING_ISSUE_FM=$(read_frontmatter | awk -F': ' '/^tracking_issue:/ {print $2}' || true)
DUE_DATE=${DUE_DATE:-$DUE_DATE_FM}
TRACKING_ISSUE=${TRACKING_ISSUE:-$TRACKING_ISSUE_FM}

# Optional comma-separated requirement IDs to focus on, e.g., R1,S1,P1
REQ_IDS_ENV=${REQUIREMENTS:-}

list_requirement_ids() {
  grep -hE '^(R|S|P)[0-9]+:' "$ROOT_DIR"/docs/requirements/*.md | sed 's/^/- /' || true
}

REPORT="$ARTIFACTS_DIR/developer_report.md"

{
  echo "# Developer Report"
  echo "Prompt: ${PROMPT_FILE#$ROOT_DIR/}"
  echo "Role: ${ROLE:-Developer}"
  echo "Assigned To: ${ASSIGNED_TO:-Copilot}"
  [[ -n "$DUE_DATE" ]] && echo "Due Date: $DUE_DATE"
  [[ -n "$TRACKING_ISSUE" ]] && echo "Tracking Issue: $TRACKING_ISSUE"
  echo
  echo "## Plan & Trace"
  if [[ -n "$REQ_IDS_ENV" ]]; then
    echo "- Focus Requirements: $REQ_IDS_ENV"
  else
    echo "- Requirement IDs present:"
    list_requirement_ids
  fi
  echo "- ADRs: see docs/decisions/"
  echo
  echo "## Code & Config Targets"
  echo "- Controller: src/ackermann_control/src/ackermann_controller.cpp"
  echo "- Safety: src/safety/src/watchdog_node.cpp"
  echo "- Nav2 config: src/navigation/config/nav2_ackermann.yaml"
  echo "- Params: src/ackermann_control/params/ackermann.yaml"
  echo
  echo "## Checks"
  echo "- Ensure interfaces unchanged or create ADR for changes"
  echo "- Link code changes to requirement IDs in commit/PR"
  echo "- Update docs/architecture and docs/requirements as needed"
  echo
  echo "## Test Commands"
  echo '```bash'
  echo 'colcon build --symlink-install'
  echo 'colcon test --event-handlers console_direct+ --packages-select ackermann_control safety'
  echo 'colcon test-result --verbose'
  echo '```'
  echo
  echo "## Suggested Next Steps"
  echo "- Implement feature per docs/prompts/developer.md checklist"
  echo "- Run safety and reviewer scripts; attach reports in PR"
} > "$REPORT"

echo "Developer report written to ${REPORT#$ROOT_DIR/}"