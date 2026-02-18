---
title: Architecture Overview
status: Draft
owner: architecture_team
agent: Copilot
last_updated: 2026-02-17
doc_type: architecture
ros_distro: humble
---
## Mission Profile

Autonomous Ackermann rover with UAV-grade autonomy stack. Primary workflow:

1. **Simulation-first**: Gazebo world publishes ground-truth pose plus virtual camera, GPS, and IMU feeds. Vehicle software validates behaviors in simulation before hardware.
2. **Perception & Localization**: RTAB-Map performs visual-inertial odometry (VIO) and loop-closure SLAM using camera + IMU + (optionally) simulated GPS for global alignment.
3. **Planning & Control**: Nav2 planners consume RTAB-Map localization and map layers to generate paths from point A→B. Ackermann controller converts twist commands to steering/speed profiles.
4. **Vehicle Interface**: Custom DDS bridge (px4-offboard) relays high-level velocity/attitude commands to PX4 firmware, which enforces safety limits and actuates the rover drivetrain.

## Core Subsystems

| Layer | Responsibilities | Key Nodes/Tools |
| --- | --- | --- |
| Simulation Inputs | Synthetic sensors, environment | Gazebo, sensor plugins, map assets |
| Localization & Mapping | VIO, SLAM, TF publishing | RTAB-Map, TF tree manager |
| Navigation & Control | Global + local planners, Ackermann control | Nav2 stack, ackermann_control package |
| Vehicle Interface | Command translation to PX4 | px4-offboard DDS bridge, PX4 autopilot |

## Data Flow Summary

- Gazebo → `/camera/*`, `/imu/data`, `/gps/fix`, `/ground_truth/pose`.
- Sensor bridge nodes republish to RTAB-Map in ROS 2 friendly QoS.
- RTAB-Map outputs `/rtabmap/odom`, `/rtabmap/mapData`, TF frames (`map`, `odom`, `base_link`).
- Nav2 consumes RTAB-Map odometry and costmaps to output `/cmd_vel_nav`.
- Ackermann controller converts `/cmd_vel_nav` to `/cmd_ackermann` (speed + steering).
- px4-offboard DDS bridge reads `/cmd_ackermann`, packages setpoints for PX4 over custom DDS topics.
- PX4 status (heartbeat, actuator feedback) loops back via DDS for health monitoring.

## Simulation → Hardware Parity

- **Sensors**: Gazebo plugins mimic VIO sensors; hardware swaps in real camera + IMU via same topics.
- **Localization**: RTAB-Map configuration shared between sim and field, with parameter overrides for lens, IMU bias, etc.
- **Vehicle Interface**: DDS bridge targets PX4 SITL in Gazebo initially, then PX4 hardware by switching DDS endpoints.
- **Safety**: Watchdog monitors PX4 status topics; on anomalies, Ackermann controller drops to zero-speed command.

This overview drives the detailed node graph, interface contracts, and failure mode definitions in the following documents.
