#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")"/../.. && pwd)"
cd "$WORKSPACE_DIR"

ARTIFACTS_DIR="$WORKSPACE_DIR/artifacts/ai"
REPORT="$ARTIFACTS_DIR/test_report.md"
mkdir -p "$ARTIFACTS_DIR"

QUALITY_GATE_THRESHOLDS=${QUALITY_GATE_THRESHOLDS:-"legal/copyright=0,whitespace/comments=0,whitespace/line_length=0,whitespace/tab=0"}
QUALITY_GATE_DEFAULT_THRESHOLD=${QUALITY_GATE_DEFAULT_THRESHOLD:-0}
QUALITY_GATE_IGNORED=${QUALITY_GATE_IGNORED:-""}

declare -A LINT_THRESHOLDS=()
declare -A LINT_COUNTS=()
declare -a QUALITY_GATE_FAILURES=()
declare -a QUALITY_GATE_IGNORED_LIST=()

trim() {
	local var="$1"
	var="${var#${var%%[![:space:]]*}}"
	var="${var%${var##*[![:space:]]}}"
	printf '%s' "$var"
}

parse_thresholds() {
	if [[ -z "$QUALITY_GATE_THRESHOLDS" ]]; then
		return
	fi
	IFS=',' read -ra pairs <<< "$QUALITY_GATE_THRESHOLDS"
	for pair in "${pairs[@]}"; do
		local cleaned
		cleaned=$(trim "$pair")
		[[ -z "$cleaned" ]] && continue
		local key="${cleaned%%=*}"
		local value="${cleaned#*=}"
		key=$(trim "$key")
		value=$(trim "$value")
		[[ -z "$key" || -z "$value" ]] && continue
		LINT_THRESHOLDS["$key"]="$value"
	done
}

parse_ignored_categories() {
	if [[ -z "$QUALITY_GATE_IGNORED" ]]; then
		return
	fi
	IFS=',' read -ra raw <<< "$QUALITY_GATE_IGNORED"
	for item in "${raw[@]}"; do
		local trimmed
		trimmed=$(trim "$item")
		[[ -z "$trimmed" ]] && continue
		QUALITY_GATE_IGNORED_LIST+=("$trimmed")
	done
}

append_report() {
	echo "$1" >> "$REPORT"
}

append_code_block() {
	local file="$1"
	append_report '```'
	cat "$file" >> "$REPORT"
	append_report '```'
}

is_ignored_category() {
	local category="$1"
	for ignored in "${QUALITY_GATE_IGNORED_LIST[@]}"; do
		if [[ "$ignored" == "$category" ]]; then
			return 0
		fi
	done
	return 1
}

record_lint_categories() {
	local file="$1"
	while IFS= read -r line; do
		if [[ $line =~ Category\ \'([^\']+)\'\ errors\ found:\ ([0-9]+) ]]; then
			local category="${BASH_REMATCH[1]}"
			local count="${BASH_REMATCH[2]}"
			LINT_COUNTS["$category"]="$count"
		fi
	done < "$file"
}

evaluate_quality_gate() {
	append_report "## Quality Gate"
	append_report ""
	if [[ $CPPLINT_AVAILABLE -ne 1 ]]; then
		append_report "- Skipped (ament_cpplint not installed)."
		return
	fi
	if [[ ${#LINT_COUNTS[@]} -eq 0 ]]; then
		append_report "- No cpplint categories reported."
		return
	fi
	mapfile -t categories < <(printf '%s\n' "${!LINT_COUNTS[@]}" | sort)
	append_report "| Category | Count | Threshold | Ignored | Status |"
	append_report "| :-- | ---: | ---: | :-----: | :----: |"
	local failures=0
	for category in "${categories[@]}"; do
		local count="${LINT_COUNTS[$category]}"
		local threshold="${LINT_THRESHOLDS[$category]:-$QUALITY_GATE_DEFAULT_THRESHOLD}"
		local ignored="no"
		local status="pass"
		if is_ignored_category "$category"; then
			ignored="yes"
			status="ignored"
		else
			if (( count > threshold )); then
				status="fail"
				failures=1
				QUALITY_GATE_FAILURES+=("$category (count=$count, threshold=$threshold)")
			fi
		fi
		append_report "| $category | $count | $threshold | $ignored | $status |"
	done
	append_report ""
	if (( failures )); then
		QUALITY_GATE_FAILED=1
		append_report "Quality gate result: **FAIL**"
	else
		append_report "Quality gate result: **PASS**"
	fi
	append_report ""
}

init_report() {
	cat <<EOF > "$REPORT"
# Test Report
Generated: $(date -u +"%Y-%m-%d %H:%M:%SZ")

Quality Gate thresholds: ${QUALITY_GATE_THRESHOLDS:-"<default:${QUALITY_GATE_DEFAULT_THRESHOLD}>"}
Ignored categories: ${QUALITY_GATE_IGNORED:-"<none>"}

EOF
}

run_cpplint() {
	append_report "## cpplint (ament_cpplint)"
	if command -v ament_cpplint >/dev/null 2>&1; then
		CPPLINT_AVAILABLE=1
		local tmp
		tmp=$(mktemp)
		set +e
		ament_cpplint src/ackermann_control src/safety 2>&1 | tee "$tmp"
		local status=${PIPESTATUS[0]}
		set -e
		append_code_block "$tmp"
		record_lint_categories "$tmp"
		CPPLINT_STATUS=$status
		rm -f "$tmp"
	else
		append_report "- Skipped (ament_cpplint not installed)."
	fi
}

run_cppcheck() {
	append_report ""
	append_report "## cppcheck"
	if command -v cppcheck >/dev/null 2>&1; then
		local tmp
		tmp=$(mktemp)
		set +e
		cppcheck --enable=warning,performance,style --std=c++17 --error-exitcode=1 \
			src/ackermann_control/src \
			src/safety/src 2>&1 | tee "$tmp"
		local status=${PIPESTATUS[0]}
		set -e
		append_code_block "$tmp"
		CPPCHECK_STATUS=$status
		rm -f "$tmp"
	else
		append_report "- Skipped (cppcheck not installed)."
	fi
}

parse_thresholds
parse_ignored_categories
init_report

CPPLINT_AVAILABLE=0
CPPLINT_STATUS=0
CPPCHECK_STATUS=0
QUALITY_GATE_FAILED=0

# Run lint before build to catch style/static issues locally.
run_cpplint
run_cppcheck
evaluate_quality_gate

if (( QUALITY_GATE_FAILED )); then
	echo "[run_tests] Quality gate failed:" >&2
	printf ' - %s\n' "${QUALITY_GATE_FAILURES[@]}" >&2
	exit 1
fi

if (( CPPCHECK_STATUS != 0 )); then
	echo "[run_tests] cppcheck reported issues." >&2
	exit "$CPPCHECK_STATUS"
fi

if (( CPPLINT_AVAILABLE == 1 && CPPLINT_STATUS != 0 && ${#LINT_COUNTS[@]} == 0 )); then
	echo "[run_tests] cpplint exited with status $CPPLINT_STATUS and produced no category summary." >&2
	exit "$CPPLINT_STATUS"
fi

append_report ""
append_report "## Build & Test"

colcon build --symlink-install | tee -a "$REPORT"
colcon test --event-handlers console_direct+ --packages-select ackermann_control safety | tee -a "$REPORT"
colcon test-result --verbose | tee -a "$REPORT"
