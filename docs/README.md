# Documentation Guide

Purpose: establish how Markdown files in `docs/` are created, maintained, and linked to code so they remain actionable for engineers and CI.

## Structure
- **Architecture:** design overviews, node graph, interfaces.
- **Context:** assumptions, constraints, project context.
- **Decisions (ADRs):** recorded architectural decisions.
- **Requirements:** system, performance, safety requirements.
- **Prompts:** AI assistant prompts used by scripts.

## Architecture Docs
- [docs/architecture/overview.md](architecture/overview.md)
- [docs/architecture/interfaces.md](architecture/interfaces.md)
- [docs/architecture/px4_overview.md](architecture/px4_overview.md)
- [docs/architecture/px4_ros2_interface_lib.md](architecture/px4_ros2_interface_lib.md)

## Frontmatter (required)
Add a YAML frontmatter block at the top of every `.md` file to standardize metadata and enable tooling.

```
---
title: <Short Title>
status: Draft | Accepted | Superseded
owner: <Name or Team>
last_updated: 2026-02-17
doc_type: requirement | ADR | architecture | prompt | context
ros_distro: humble
---
```

## Conventions
- **Link to code/config:** reference exact paths, e.g., [src/ackermann_control/src/ackermann_controller.cpp](../src/ackermann_control/src/ackermann_controller.cpp) and [src/navigation/config/nav2_ackermann.yaml](../src/navigation/config/nav2_ackermann.yaml).
- **Keep docs concise:** prefer checklists, tables, and explicit acceptance criteria.
- **Version awareness:** note applicable ROS distro and constraints when relevant.
- **Ownership:** each doc has a clear owner responsible for updates.

## Templates (copy as needed)

### ADR (decisions/*)
```
---
title: ADR-XXX: <Decision>
status: Accepted
owner: <Name>
last_updated: 2026-02-17
doc_type: ADR
---

Context
Why is this decision needed? What constraints apply?

Decision
State the decision clearly.

Alternatives Considered
- Option A: pros/cons
- Option B: pros/cons

Consequences
- Positive and negative outcomes, operational impact.

Links
- Implementation: ../src/safety/src/watchdog_node.cpp
- Config: ../src/navigation/config/nav2_ackermann.yaml
```

### Requirement (requirements/*)
```
---
title: R-XXX: <Requirement>
status: Draft
owner: <Name>
last_updated: 2026-02-17
doc_type: requirement
---

Statement
Clear, testable requirement (e.g., "Vehicle shall stop on fault").

Acceptance Criteria
- Observable condition(s) and pass/fail thresholds.

Verification
- Test procedure(s), tools, and expected artifacts.

Traceability
- Code: ../src/safety/src/watchdog_node.cpp
- Params: ../src/ackermann_control/params/ackermann.yaml
```

### Architecture (architecture/*)
```
---
title: Node Graph
status: Draft
owner: <Name>
last_updated: 2026-02-17
doc_type: architecture
---

Components
- ackermann_controller: subscribes to path/odometry, publishes ackermann_msgs.
- safety_watchdog: monitors heartbeats, commands safe stop.
- nav2 stack: planning and control plugins.

Topics/Services
- /cmd_ackermann (ackermann_msgs/AckermannDrive)
- /odom (nav_msgs/Odometry)

Links
- Controller: ../src/ackermann_control/src/ackermann_controller.cpp
- Safety: ../src/safety/src/watchdog_node.cpp
```

### Prompt (prompts/*)
```
---
title: Safety Review Prompt
status: Draft
owner: <Name>
last_updated: 2026-02-17
doc_type: prompt
---

Role
Safety reviewer.

Checklist
- Verify watchdog triggers on fault and commands stop.
- Confirm requirements coverage and tests exist.
- Check interfaces and parameter bounds.

Outputs
- Review notes, required changes, and sign-off.
```

## Editing Workflow
1. Create or update the doc with frontmatter and the relevant template.
2. Cross-link to code/config using relative paths under `../src/...`.
3. Add acceptance criteria (requirements) or explicit consequences (ADRs).
4. Request review from the doc owner and a relevant code owner.
5. Keep `last_updated` current upon merge.

## Quality Checks (recommended)
- **Lint:** run markdown linting and link checking locally or in CI.
- **Links:** ensure all referenced files exist in the repo.
- **Consistency:** align terminology with code (topic names, message types).

### Optional local checks
Install markdownlint-cli and run checks:

```
npm install -g markdownlint-cli
markdownlint docs/**/*.md
```

## Examples
- Requirement linking to code: [watchdog implementation](../src/safety/src/watchdog_node.cpp).
- Architecture linking to params: [controller params](../src/ackermann_control/params/ackermann.yaml).

## Maintenance
- Review docs when changing code interfaces or parameters.
- Supersede ADRs with a new ADR when decisions change; keep status updated.
- Use owners and reviews to prevent stale documentation.

## Folder-by-Folder Usage

**Architecture (docs/architecture/)**
- **Purpose:** Describe how the system is structured (components, topics/services, data flow) so engineers can build and troubleshoot.
- **Modify & Extend:** Add components, topic names, message types, lifecycle states; include diagrams and links to implementations like [src/ackermann_control/src/ackermann_controller.cpp](../src/ackermann_control/src/ackermann_controller.cpp) and configs like [src/navigation/config/nav2_ackermann.yaml](../src/navigation/config/nav2_ackermann.yaml).
- **Best Practices:** Keep topic/service contracts explicit; add example payloads; document startup order and lifecycle transitions.

**Requirements (docs/requirements/)**
- **Purpose:** Capture testable statements the system must satisfy (safety, performance, system behavior) with traceability to code/tests.
- **Modify & Extend:** For each requirement, add acceptance criteria and verification steps; link directly to code (e.g., [src/safety/src/watchdog_node.cpp](../src/safety/src/watchdog_node.cpp)) and parameter files (e.g., [src/ackermann_control/params/ackermann.yaml](../src/ackermann_control/params/ackermann.yaml)).
- **Best Practices:** Make requirements measurable; include pass/fail thresholds; maintain IDs and status; ensure coverage in tests.

**Decisions / ADRs (docs/decisions/)**
- **Purpose:** Record architectural decisions with context, alternatives, and consequences to guide future changes.
- **Modify & Extend:** Use the ADR template; add links to implementation and configs; update status (Accepted/Superseded) when decisions evolve.
- **Best Practices:** Include rationale and trade-offs; list alternatives considered; note impact areas and migration steps.

**Context (docs/context/)**
- **Purpose:** State assumptions, constraints, and project background that shape requirements and designs.
- **Modify & Extend:** Expand assumptions with justification and impact; map constraints to ADRs and requirements; link to relevant code/config.
- **Best Practices:** Keep environment and hardware constraints explicit; note what happens if assumptions are violated; timestamp and assign owners.

**Prompts (docs/prompts/)**
- **Purpose:** Provide reusable, machine-consumable prompts for AI workflows (development, review, planning, safety checks).
- **Modify & Extend:** Add role, scope, checklist, and expected outputs; reference docs and code paths; ensure consistent metadata in frontmatter.
- **Best Practices:** Keep prompts actionable and specific; align with contribution guidelines; validate in `scripts/ai/` flows.
