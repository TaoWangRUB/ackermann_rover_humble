---
title: Project Context
status: Draft
owner: system_team
agent: Copilot
last_updated: 2026-02-19
doc_type: context
ros_distro: humble
---
## Overview

Ackermann autonomous rover using ROS 2 Humble.

## Reference Projects

- **Gazebo Ackermann RC Simulation** (local workspace: `gazebo_ackermann_rc_sim`)
	- Purpose: baseline Gazebo simulation and controller setup for an Ackermann RC platform.
	- Usage: informs world setup, vehicle dynamics, and control topics reused or adapted in this project.
	- Scope differences: this project adds Nav2 integration, safety watchdog, and RTAB-Map SLAM on top of the reference.

- **PX4-Autopilot** (local workspace: `/home/taowang/workspace/PX4-Autopilot`)
	- Version: v1.17.0-alpha1 (main branch, commit `14e3a2da03`)
	- Purpose: open-source autopilot firmware used as the low-level flight controller for the rover. Provides offboard control, state estimation, and actuator management.
	- ROS 2 integration: via `uxrce_dds_client` module which bridges PX4 uORB topics to DDS/ROS 2 topics.
	- Key modules used by this project:
		- `rover_ackermann` — PX4's native Ackermann rover controller (AckermannActControl, AckermannPosControl, AckermannRateControl, AckermannSpeedControl, AckermannDriveModes).
		- `uxrce_dds_client` — Micro-XRCE-DDS bridge exposing PX4 topics to ROS 2 (topic list in `dds_topics.yaml`).
		- `gz_bridge` — Gazebo↔PX4 simulation bridge; **authoritative reference for coordinate frame conversions** (ENU/FLU ↔ NED/FRD).
	- Key message types (PX4 `.msg` definitions):
		- `OffboardControlMode` — selects which setpoint fields PX4 should obey (position, velocity, acceleration, attitude, body_rate).
		- `TrajectorySetpoint6dof` — velocity/position setpoint in NED frame.
		- `VehicleAcceleration`, `VehicleImu`, `VehicleLocalPositionSetpoint` — state estimation outputs.
	- Coordinate frame conventions (from `gz_bridge/GZBridge.cpp`):
		- **World position**: ENU `(x,y,z)` → NED `(y, x, -z)`
		- **Orientation**: FLU→ENU quaternion → FRD→NED quaternion via `q_ENU→NED * q_FLU→ENU * inv(q_FLU→FRD)` where `q_FLU→FRD = (0,1,0,0)` and `q_ENU→NED = (0, √2/2, √2/2, 0)`
		- **Body linear/angular velocity**: FLU `(x,y,z)` → FRD `(x, -y, -z)`
	- Simulation: PX4 SITL with Gazebo via `gz_bridge`, `gz_plugins`, and `gz_msgs`.
	- Source path: `/home/taowang/workspace/PX4-Autopilot/src/`

- **px4-ros2-interface-lib** (git submodule: `src/px4-ros2-interface-lib`)
	- Repository: [Auterion/px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib)
	- Version: v2.0.0 (main branch)
	- Purpose: C++ library for registering custom PX4 flight modes via ROS 2. Replaces manual offboard heartbeat/command publishing with a proper mode lifecycle API.
	- Key APIs used:
		- `px4_ros2::ModeBase` — base class for custom flight modes with `onActivate()`, `onDeactivate()`, `updateSetpoint(dt)` lifecycle
		- `px4_ros2::NodeWithMode<T>` — template node that registers a mode and spins
		- `px4_ros2::TrajectorySetpointType` — NED velocity/position setpoint (used by offboard trajectory mode)
		- `px4_ros2::RoverSpeedSteeringSetpointType` — body speed + normalized steering (used by speed steering mode)
		- `px4_ros2::RoverSpeedAttitudeSetpointType` — body speed + NED yaw heading (used by speed attitude mode)
		- `px4_ros2::OdometryLocalPosition` — read current vehicle heading from PX4 telemetry
	- Depends on: `px4_msgs` (must match PX4 firmware version)
	- Build: `find_package(px4_ros2_cpp REQUIRED)`, link `px4_ros2_cpp::px4_ros2_cpp`

Add additional reference projects here as they are adopted (e.g., upstream Nav2 demos, RTAB-Map tutorials).
