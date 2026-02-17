#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")"/../.. && pwd)"
ARTIFACTS_DIR="$ROOT_DIR/artifacts/ai"
mkdir -p "$ARTIFACTS_DIR"

PROMPT_FILE="${PROMPT_FILE:-$ROOT_DIR/docs/prompts/reviewer.md}"

read_frontmatter() {
	awk '/^---/{flag=!flag; next} flag {print}' "$PROMPT_FILE" || true
}

ROLE=$(read_frontmatter | awk -F': ' '/^role:/ {print $2}' || true)
ASSIGNED_TO=$(read_frontmatter | awk -F': ' '/^assigned_to:/ {print $2}' || true)
DUE_DATE_FM=$(read_frontmatter | awk -F': ' '/^due_date:/ {print $2}' || true)
TRACKING_ISSUE_FM=$(read_frontmatter | awk -F': ' '/^tracking_issue:/ {print $2}' || true)
DUE_DATE=${DUE_DATE:-$DUE_DATE_FM}
TRACKING_ISSUE=${TRACKING_ISSUE:-$TRACKING_ISSUE_FM}

REPORT="$ARTIFACTS_DIR/reviewer_report.md"

md_links_check() {
	local issues=0
	while IFS= read -r file; do
		while IFS= read -r link; do
			# Extract path inside ](path)
			path=$(echo "$link" | sed -n 's/.*](\(.*\)).*/\1/p')
			# Only check relative repo paths
			if [[ "$path" == ../* || "$path" == ../../* ]]; then
				# Normalize to repo root
				norm="$ROOT_DIR/${path#../}"
				if [[ ! -e "$norm" ]]; then
					echo "- Broken link in ${file#$ROOT_DIR/}: $path" >> "$REPORT"
					issues=$((issues+1))
				fi
			fi
		done < <(grep -o ']([^)]*)' "$file" || true)
	done < <(find "$ROOT_DIR/docs" -name '*.md')
	return $issues
}

{
	echo "# Reviewer Report"
	echo "Prompt: ${PROMPT_FILE#$ROOT_DIR/}"
	echo "Role: ${ROLE:-Reviewer}"
	echo "Assigned To: ${ASSIGNED_TO:-Copilot}"
	[[ -n "$DUE_DATE" ]] && echo "Due Date: $DUE_DATE"
	[[ -n "$TRACKING_ISSUE" ]] && echo "Tracking Issue: $TRACKING_ISSUE"
	echo
	echo "## Checks"
	echo "- Markdown lint: (skipped if markdownlint not installed)"
} > "$REPORT"

# Optional markdownlint
if command -v markdownlint >/dev/null 2>&1; then
	markdownlint "$ROOT_DIR/docs"/**/*.md >> "$REPORT" || true
fi

echo "## Link Checks" >> "$REPORT"
md_links_check || true

echo "## Requirement IDs" >> "$REPORT"
grep -hE '^(R|S|P)[0-9]+:' "$ROOT_DIR"/docs/requirements/*.md | sed 's/^/- /' >> "$REPORT" || true

echo "Reviewer report written to ${REPORT#$ROOT_DIR/}"
