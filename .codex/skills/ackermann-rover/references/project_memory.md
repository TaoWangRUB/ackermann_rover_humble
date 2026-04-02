# Project Memory

## Stack Summary

- Primary stack: Gazebo Harmonic + ROS 2 Jazzy + RTAB-Map + Nav2 + ros2_control + optional PX4 SITL or hardware bridge.
- Default development environment: Docker service `ackermann_slam` from `docker/docker-compose.yml`.
- Host-side operations are a real part of the architecture: the repo includes a standalone `control_center/` service and a large `scripts/` layer that encodes canonical compose, tmux, launch, and smoke-test flows.
- Important shell prerequisite on every host terminal: `source ~/.bashrc`.
- Automation gotcha: in non-interactive shells, `.bashrc` can return before exporting the Compose variables. If Compose reports blank `ARCH`, `IMAGE_NAME`, or user IDs, explicitly load `.env` and export `ARCH`, `USERNAME`, `USER_UID`, and `USER_GID` before running `docker compose`/`docker-compose`.
- Verification baseline: `AGENTS.md` plus `scripts/verify_agent_work.sh`.

## Subsystem Map

- `src/robot_bringup`
  - Top-level launch entrypoint: `launch/robot_bringup.launch.py`
  - Composes Gazebo or hardware sensors, RTAB-Map, Nav2, RViz, and rover monitor.
  - In hardware mode publishes synthetic `/joint_states` from `/cmd_vel` via `scripts/cmd_vel_joint_relay.py` for TF and RViz continuity.
- `src/description_robot`
  - Owns the robot URDF/Xacro tree, worlds, Gazebo launch, ROS-Gazebo bridges, and controller spawning.
  - `launch/gazebo_bringup.launch.py` sets resource paths, starts Gazebo, bridges camera/IMU/LiDAR/odom/clock topics, spawns the robot, and starts controller spawners unless PX4 SITL is driving joints.
  - `config/ackermann_controller.yaml` configures `joint_state_broadcaster` and `ackermann_steering_controller`.
- `src/rtabmap_bringup`
  - `launch/rtabmap_slam.launch.py` wires:
    - `rtabmap_sync/rgbd_sync`
    - `imu_transformer` + `imu_filter_madgwick`
    - `rtabmap_odom` in RGB-D or ICP mode
    - `robot_localization` EKF
    - `rtabmap_slam` and optional `rtabmap_viz`
    - `depthimage_to_laserscan`
    - `rtabmap_util/point_cloud_xyz` and `obstacles_detection`
  - Primary frames: `map -> odom -> ackermann/base_link`.
- `src/ackermann_nav2_bringup`
  - `launch/nav2_bringup.launch.py` inlines Nav2 navigation launch behavior.
  - Remaps Nav2 odom to `/odometry/filtered`.
  - Uses MPPI controller with `motion_model: Ackermann` and Smac Hybrid planner.
  - `reversible_drive` switches between forward-only and bidirectional planning/controller limits.
- `src/px4_bringup`
  - `launch/px4_bringup.launch.py` starts custom PX4 mode nodes and optional odometry bridges.
  - Supports `trajectory`, `speed_steering`, `speed_attitude`, and `manual` modes.
  - Uses `src/px4-ros2-interface-lib` and `px4_msgs`.
- `src/realsense_camera_bringup`
  - Starts D435i, L515, and optional T265.
  - Includes `odom_tf_relay` for T265 odometry re-expression into `ackermann/base_link`.
- `src/rover_monitor`
  - Monitoring stack for telemetry, camera, PX4, and Jetson probes plus aggregation/publishing.
- `control_center`
  - Host-side base-station stack, documented in `docs/architecture/control_center.md`.
  - `main.py` wires `EventBus`, `MQTTReceiver`, `TelemetryCache`, `InfluxDBWriter`, `AlertNotifier`, `DashboardServer`, `SafetyGate`, `CommandGateway`, and `AckTracker`.
  - `docker-compose.yaml` starts Mosquitto, InfluxDB, and the Control Center app on the host network.
  - `ui/src/App.jsx` is the React dashboard for health panels, alerts, drive controls, and command history.
  - Operational contract: it pairs with `src/rover_monitor` over MQTT/Protobuf topics such as `rover/health/#`, `rover/alerts`, and `rover/cmd/ack`.
- `src/ackermann_control`
  - Converts `/cmd_vel` into `ackermann_msgs/AckermannDriveStamped`.
- `src/safety`
  - Hosts the safety watchdog node and its tests.

## Scripts Layer

- `scripts/lib/dc.sh`
  - Canonical Compose wrapper for the repo.
  - Loads `.env`, fills runtime defaults, and chooses a working Compose implementation (`docker compose`, `docker-compose`, or snap plugin fallback).
- `scripts/start_docker.sh`
  - Starts the dev container if needed and opens an interactive shell with ROS and workspace overlays sourced.
- `scripts/start_ros2_nodes.sh`
  - Main host wrapper for `robot_bringup`.
  - Encodes the normal matrix for sim vs hardware, depth camera selection, optional T265, RTAB-Map, Nav2, PX4 SITL, PX4 mode node, VO bridge, odom-topic selection, reversible drive, and optional pre-builds.
- `scripts/start_px4_sitl.sh`, `scripts/start_microxrce_agent.sh`, `scripts/start_px4_bringup_vo.sh`
  - Canonical PX4 bringup helpers for SITL, XRCE transport, and ROS-side mode/odometry bridging.
- `scripts/start_cameras.sh`
  - RealSense launcher wrapper for hardware workflows.
- `scripts/start_host_session.sh`
  - Host/base-station tmux session for Control Center startup, logs, MQTT monitoring, and Control Center tests.
- `scripts/start_system_monitor_session.sh`
  - Tmux session for `rover_monitor`, mock publishers or live topics, optional telemetry publishing, and optional Control Center stack startup.
- `scripts/start_jetson_session.sh`, `scripts/start_camera_px4_test_session.sh`, `scripts/start_px4_custom_mode_test_session.sh`
  - Higher-level tmux orchestrators for hardware or integration-test workflows.
- `scripts/stop_all.sh`
  - Cleanup helper for ROS, PX4, MicroXRCE, helper scripts, and tmux sessions.
- `scripts/verify_agent_work.sh`, `scripts/verify_odom.sh`, `scripts/verify_cmd_vel_chain.sh`, `scripts/motion_test.py`, `scripts/test_control_center.sh`
  - Verification and smoke-test surface for launch health, odometry plumbing, command-velocity chains, motion response, and Control Center telemetry/ACK behavior.
- `scripts/publish_mock_camera.py`, `scripts/publish_mock_px4.py`, `scripts/decode_rover_health_mqtt.py`, `scripts/pub_*`, `scripts/px4_cmd*`, `scripts/upload_params.sh`, `scripts/build_px4_rover_fw.sh`
  - Utility surface for mocks, telemetry decoding, command publishing, PX4 interaction, firmware/param workflows, and ad hoc diagnostics.

## Launch Matrix

- Full sim + SLAM + Nav2:
  - `ros2 launch robot_bringup robot_bringup.launch.py rtabmap:=true nav2:=true`
- Gazebo only:
  - `ros2 launch robot_bringup robot_bringup.launch.py rtabmap:=false nav2:=false`
- Gazebo + RTAB-Map:
  - `ros2 launch robot_bringup robot_bringup.launch.py rtabmap:=true nav2:=false`
- Hardware path:
  - `use_gazebo:=false use_sim_time:=false`
  - Depth camera selected by `depth_camera:=l515|d435i`
  - T265 optional with `hw_enable_t265:=true`
- PX4 SITL path:
  - `enable_px4_sitl:=true` switches joint actuation away from ros2_control and into Gazebo PX4 joint plugins.

## Operational Entry Points

- Container lifecycle:
  - `scripts/lib/dc.sh` is the durable entrypoint for repo scripts that need Compose.
  - `scripts/start_docker.sh` is the quickest way to get into the dev container with the right sourcing.
- Rover stack launch:
  - `scripts/start_ros2_nodes.sh` is the highest-signal host wrapper and is often a better “what actually runs” reference than a raw launch file alone.
- PX4 support processes:
  - `scripts/start_microxrce_agent.sh`
  - `scripts/start_px4_sitl.sh`
  - `scripts/start_px4_bringup_vo.sh`
- Host telemetry and dashboard workflows:
  - `scripts/start_system_monitor_session.sh --with-telemetry`
  - `scripts/start_host_session.sh`
  - `scripts/test_control_center.sh`

## Topic And Frame Expectations

- RTAB-Map camera topics are derived from `depth_camera`.
  - Simulation: `/<depth_camera>/image`, `/<depth_camera>/depth_image`, `/<depth_camera>/camera_info`, `/<depth_camera>/imu/raw`
  - Hardware: `<depth_camera>/color/image_raw`, `<depth_camera>/aligned_depth_to_color/image_raw`, related camera info, and `<depth_camera>/imu`
- EKF output for the rest of the stack: `/odometry/filtered`
- RTAB-Map inputs use `/vo_odom` or `/icp_odom` plus `/imu/data`
- Nav2 consumes `/odometry/filtered`, `/tf`, `/map`, and scan/costmap topics
- ros2_control controller publishes `/ackermann_steering_controller/odometry` and joint states; TF base frame stays `ackermann/base_link`
- Control Center / rover monitor MQTT topics:
  - `rover/health/#`
  - `rover/alerts`
  - `rover/cmd/ack`
  - outbound command topics such as `rover/cmd/goal`, `rover/cmd/arm`, `rover/cmd/disarm`, `rover/cmd/mode`, `rover/cmd/estop`, and `rover/cmd/cancel_goal`

## Important Design Decisions

- `ADR-007`: three joint-control scenarios exist and they change who owns joint state and actuation:
  - Gazebo + ros2_control
  - Gazebo + PX4 SITL
  - Real hardware without ROS feedback, using `cmd_vel_joint_relay.py` for visualization only
- `ADR-008`: PX4 odometry bridging must respect ROS ENU/FLU vs PX4 NED/FRD conversions. Twist is body-frame data, not world-frame data.
- `ADR-004`, `ADR-005`, `ADR-006`: PX4 custom modes are first-class design choices, not temporary hacks.

## Verification Rules To Remember

- Host-side Docker lifecycle:
  - Prefer `docker-compose -f docker/docker-compose.yml ...` because that is what `AGENTS.md` prescribes.
- Build/test expectations:
  - `colcon build --symlink-install`
  - `colcon test --event-handlers console_direct+`
  - `colcon test-result --verbose`
- The repo also expects simulation validation, topic/TF checks, Nav2 lifecycle readiness, ros2_control controller activity, and clean shutdown after validation.
- `scripts/verify_agent_work.sh` is the automation baseline and currently performs a staged build, limited package tests, a timed launch smoke test, and teardown.
- Full-workspace `colcon test` currently also exercises vendored `px4_ros2_cpp` and `px4-ros2-interface-lib` example-package lint/integration suites, which are stricter than the repo verification script and include FMU-dependent cases.
- Control Center validation is partly outside the ROS workspace:
  - `scripts/test_control_center.sh` checks dashboard telemetry freshness, command dispatch, ACK history, and post-command telemetry continuity.

## Known Repo Conventions

- The repo documentation is already structured for durable context:
  - `docs/context/` for assumptions and constraints
  - `docs/architecture/` for nodes, topics, and subsystem behavior
  - `docs/decisions/` for ADRs that should be read before changing interfaces
- Many practical workflows live in shell scripts and tmux sessions rather than only in ROS launch files, so repo understanding is incomplete unless `scripts/` is inspected alongside `src/` and `docs/`.
- If a change alters public topics, frames, parameters, or launch behavior, update docs in the same task.
- Keep container and simulation cleanup explicit. Background launches left running will interfere with later work.
