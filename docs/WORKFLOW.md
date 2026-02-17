---
title: Development Workflow
status: Draft
owner: system_team
agent: Copilot
last_updated: 2026-02-17
doc_type: context
ros_distro: humble
---

## Purpose
Standardize how features are implemented according to requirements, tested, and committed to the repo with clear traceability and review.

## 1) Plan & Trace
- Identify requirement IDs in [docs/requirements](./requirements) (e.g., R1, S1, P1) and related ADRs in [docs/decisions](./decisions).
- Select the relevant role prompt in [docs/prompts](./prompts) and fill Inputs (IDs, files).

## 2) Implement
- Write code aligned with architecture ([docs/architecture](./architecture)) and constraints ([docs/context](./context)).
- Keep interfaces stable or record changes via a new ADR.

## 3) Test
- Build and run tests locally before committing.
- Example ROS 2 workspace commands:

```bash
colcon build --symlink-install
colcon test --event-handlers console_direct+ --packages-select ackermann_control safety
colcon test-result --verbose
```

> Tip: `scripts/ai/run_developer.sh` summarizes requirements and checklists but intentionally does not run these commands. Always execute the build/test sequence yourself (or via CI automation) before moving on.

Shortcut: use [scripts/ai/run_tests.sh](../scripts/ai/run_tests.sh) to run the same sequence:

```bash
bash scripts/ai/run_tests.sh
```

## 4) Update Docs
- For changed behavior or interfaces, update architecture pages and requirements traceability.
- Ensure links to code/config are correct.

## 5) Safety Review
- Run safety checklist using [scripts/ai/run_safety_check.sh](../scripts/ai/run_safety_check.sh) and attach the report.

```bash
PROMPT_FILE=docs/prompts/safety.md bash scripts/ai/run_safety_check.sh
```

## 6) Review
- Run reviewer checklist using [scripts/ai/run_reviewer.sh](../scripts/ai/run_reviewer.sh); fix broken links and missing traceability.

```bash
PROMPT_FILE=docs/prompts/reviewer.md bash scripts/ai/run_reviewer.sh
```

## 7) Commit & PR
- Commit message template (reference requirement IDs and ADRs):

```
feat(ackermann_control): implement R1 path following

Refs: R1, ADR-002
Docs: updated docs/architecture/interfaces.md, docs/requirements/system.md
Tests: added/updated controller tests
```

- Open a PR using the template in .github; include artifacts from scripts in `artifacts/ai/`.

## Enforcement & CI (optional)
- Add CI jobs to run link checks and safety/reviewer scripts on PRs.
- Consider a commit lint to require requirement IDs in commit bodies.
