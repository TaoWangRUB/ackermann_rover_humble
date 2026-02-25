---
description: How to develop a new feature in the Ackermann Rover repo
---
# Feature Development Workflow

When tasked with creating or modifying a feature, verify you comply with the architecture before modifying code.

1. Ensure Docker is running.
// turbo
2. Verify the base build.
```bash
docker compose -f docker/docker-compose.yml exec ackermann_slam bash -c "colcon build --symlink-install"
```
3. Implement the feature. Follow `/home/taowang/workspace/ackermann_rover_humble/.agent/instructions.md`.
4. Add unit and functional tests to the respective package.
5. If public interfaces or parameters change, draft an ADR in `docs/decisions/` and update `docs/architecture/`.
6. Run the rigorous verification pipeline:
// turbo
```bash
bash scripts/verify_agent_work.sh
```
7. Only if `verify_agent_work.sh` passes, are you permitted to conclude the task.
