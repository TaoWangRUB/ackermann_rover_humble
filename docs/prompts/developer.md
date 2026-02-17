---
title: Developer Prompt
status: Draft
owner: prompts_team
agent: Copilot
last_updated: 2026-02-17
doc_type: prompt
ros_distro: humble
role: Developer
assigned_to: Copilot
due_date:
tracking_issue:
reviewers:
	- Lead Reviewer
---
Role: Developer. Implement specs only.

Scope
- Implement features strictly per requirements in [docs/requirements](../requirements).
- Respect decisions in [docs/decisions](../decisions) and interfaces in [docs/architecture/interfaces.md](../architecture/interfaces.md).

Inputs
- Requirements to implement: link IDs (e.g., R1, S1) and their acceptance criteria.
- Code locations: [src/ackermann_control](../../src/ackermann_control), [src/safety](../../src/safety), configs under [src/navigation/config](../../src/navigation/config).
- Contribution guide: [CONTRIBUTING.md](../../CONTRIBUTING.md).

Constraints
- Do not change public interfaces without ADR and doc updates.
- Maintain parameter compatibility with [ackermann.yaml](../../src/ackermann_control/params/ackermann.yaml).

Deliverables
- Code changes with unit/functional tests.
- Updated docs where applicable (requirements traceability, architecture links).

Checklist
- Implement feature and add tests (build locally).
- Update or create ADR if an interface or design changes.
- Link code changes to requirement IDs in PR description.
- Ensure CI passes and docs links are valid.

Definition of Done
- Tests pass; acceptance criteria satisfied.
- Reviewed by `reviewers` listed in frontmatter; docs updated.

Commit & PR
- Use commit messages referencing requirement IDs and ADRs (see [docs/WORKFLOW.md](../WORKFLOW.md)).
- Include changes to docs and tests in PR; attach `artifacts/ai/*_report.md` from scripts.

Test Commands (example)
```bash
colcon build --symlink-install
colcon test --event-handlers console_direct+ --packages-select ackermann_control safety
colcon test-result --verbose
```

Shortcut: `bash scripts/ai/run_tests.sh`

Note: `scripts/ai/run_developer.sh` provides planning guidance only; it does not invoke these commands. Run them manually (or via your preferred automation) before committing.
