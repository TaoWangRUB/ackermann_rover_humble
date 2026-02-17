# Developer Report
Prompt: docs/prompts/developer.md
Role: Developer
Assigned To: Copilot

## Plan & Trace
- Requirement IDs present:
- P1: Max speed 2.0 m/s.
- S1: Vehicle shall stop on fault.
- R1: Autonomous navigation from A to B.
- ADRs: see docs/decisions/

## Code & Config Targets
- Controller: src/ackermann_control/src/ackermann_controller.cpp
- Safety: src/safety/src/watchdog_node.cpp
- Nav2 config: src/navigation/config/nav2_ackermann.yaml
- Params: src/ackermann_control/params/ackermann.yaml

## Checks
- Ensure interfaces unchanged or create ADR for changes
- Link code changes to requirement IDs in commit/PR
- Update docs/architecture and docs/requirements as needed

## Test Commands
```bash
colcon build --symlink-install
colcon test --event-handlers console_direct+ --packages-select ackermann_control safety
colcon test-result --verbose
```

## Suggested Next Steps
- Implement feature per docs/prompts/developer.md checklist
- Run safety and reviewer scripts; attach reports in PR
