---
title: Safety Review Prompt
status: Draft
owner: prompts_team
agent: Copilot
last_updated: 2026-02-17
doc_type: prompt
ros_distro: humble
role: Safety Reviewer
assigned_to: Copilot
due_date:
tracking_issue:
reviewers:
	- Safety Lead
---
Role: Safety reviewer.

Scope
- Evaluate safety requirements and implementations (watchdog, control bounds, fail-safe behaviors).

Checklist
- Requirements mapping: confirm [docs/requirements/safety.md](../requirements/safety.md) covers current changes.
- Watchdog: verify fault detection and stop behavior in [src/safety/src/watchdog_node.cpp](../../src/safety/src/watchdog_node.cpp).
- Parameters: check limits (e.g., `max_steering_angle`, `max_speed`) in [ackermann.yaml](../../src/ackermann_control/params/ackermann.yaml).
- Failure modes: ensure mitigations documented in [failure_modes](../architecture/failure_modes.md).
- Tests: require fault injection scenarios and pass/fail outcomes.

Outputs
- Safety review notes, mitigation recommendations, and sign-off.

Definition of Done
- Safety requirements traceable to tests; watchdog behavior verified; mitigations documented.
