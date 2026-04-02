---
name: ackermann-rover-stack
description: Use when working in the ackermann_rover_humble repository or changing its Docker, Gazebo, RTAB-Map, Nav2, ros2_control, RealSense, PX4, rover monitoring, Control Center, tmux session, or verification flows. Provides the repo-specific subsystem map, launch/script entrypoints, architecture references, and mandatory validation workflow.
---

# Ackermann Rover Stack

Use this skill for any task in this repository. Start by reading [references/project_memory.md](references/project_memory.md). Open the relevant ADR or architecture doc before changing public behavior, topic names, frames, or launch sequencing.

## Workflow

1. On the host, run `source ~/.bashrc` before any Docker command. The compose stack depends on exported `ARCH`, `USERNAME`, `USER_UID`, and `USER_GID`.
   In non-interactive automation, `.bashrc` may return before exporting them; if Compose still shows blank variables, load `.env` and export `ARCH`, `USERNAME`, `USER_UID`, and `USER_GID` explicitly before invoking Compose.
2. Manage the environment with `docker-compose -f docker/docker-compose.yml ...` as described in `AGENTS.md`. In practice, many day-to-day workflows are wrapped by `scripts/lib/dc.sh` and the `scripts/start_*.sh` helpers, so inspect the matching script before reproducing a launch sequence manually.
3. Run ROS build, test, launch, and debugging commands inside `ackermann_slam`. Treat the host-side `control_center/` stack separately from the in-container rover stack.
4. Before declaring a task done, follow the repo validation contract in `AGENTS.md`, `.agent/instructions.md`, and `scripts/verify_agent_work.sh`. Always shut down launches and containers you started.

## Mental Model

- `robot_bringup` is the top-level composition layer. It selects simulation vs hardware, depth camera choice, optional RTAB-Map, optional Nav2, optional PX4 SITL, RViz, and rover monitoring.
- `description_robot` owns the URDF, Gazebo world, ROS-Gazebo bridges, and ros2_control/PX4-SITL joint control plumbing.
- `rtabmap_bringup` owns RGB-D sync, IMU frame conditioning, VIO or ICP odometry, EKF fusion, RTAB-Map SLAM/localization, depth-to-scan, and obstacle point cloud generation.
- `ackermann_nav2_bringup` owns the Nav2 stack with MPPI + Smac Hybrid and remaps odom consumption to `/odometry/filtered`.
- `px4_bringup` owns the PX4 DDS/MAVLink bridges and the custom `px4_ros2_interface_lib` modes.
- `realsense_camera_bringup` owns D435i/L515/T265 startup and T265 odom relay behavior for hardware mode.
- `rover_monitor` is the health/telemetry monitoring subsystem.
- `control_center` is the host-side base-station subsystem. It consumes `rover_monitor` telemetry over MQTT/Protobuf, serves a FastAPI + React dashboard, stores history in InfluxDB, gates outbound commands, and tracks command ACKs.
- `scripts/` is a first-class operational layer, not just miscellaneous helpers. `scripts/lib/dc.sh` normalizes Compose usage, and the `start_*`, `stop_all.sh`, `verify_*`, and test scripts encode the canonical launch, tmux-session, and smoke-test workflows.
- `ackermann_control` and `safety` are small focused nodes with their own parameters and tests.

## Read These First For Common Task Types

- Launch orchestration, topics, TF, startup ordering: `docs/architecture/overview.md`, `docs/architecture/interfaces.md`, `docs/architecture/node_graph.md`
- PX4 mode or odometry bridge work: `docs/architecture/px4_overview.md`, `docs/decisions/ADR-004-rover-manual-mode.md`, `docs/decisions/ADR-005-rover-speed-steering-mode.md`, `docs/decisions/ADR-006-rover-speed-attitude-mode.md`, `docs/decisions/ADR-008-px4-odometry-frames.md`
- Gazebo / ros2_control joint behavior: `docs/decisions/ADR-007-steering-joint-control.md`
- Nav2 behavior and tuning: `docs/architecture/nav2_overview.md`, `docs/decisions/ADR-002-nav2.md`
- Host-side monitoring and command flows: `docs/architecture/control_center.md`, `docs/architecture/system_monitor.md`
- Operational entrypoints and smoke tests: `scripts/lib/dc.sh`, `scripts/start_ros2_nodes.sh`, `scripts/start_px4_sitl.sh`, `scripts/start_microxrce_agent.sh`, `scripts/start_px4_bringup_vo.sh`, `scripts/start_host_session.sh`, `scripts/start_system_monitor_session.sh`, `scripts/test_control_center.sh`, `scripts/verify_agent_work.sh`
- Safety expectations: `docs/requirements/safety.md`, `docs/decisions/ADR-003-safety.md`

## Debugging Defaults

- ROS graph: `ros2 topic list`, `ros2 topic hz`, `ros2 node list`, `ros2 lifecycle get`
- TF: `ros2 run tf2_ros tf2_echo ...`, `ros2 run tf2_tools view_frames`
- Gazebo transport: `gz topic -l`, `gz topic -e -t <topic>`
- Controller state: `ros2 control list_controllers`, `ros2 control list_hardware_interfaces`

Keep changes aligned with existing docs. If you intentionally change interfaces, launch semantics, frames, or requirements, update the architecture docs and ADRs in the same task.
