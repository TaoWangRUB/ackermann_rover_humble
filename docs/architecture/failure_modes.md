---
title: Failure Modes
status: Draft
owner: architecture_team
agent: Copilot
last_updated: 2026-02-17
doc_type: architecture
ros_distro: humble
---
## Visual-Inertial Odometry (RTAB-Map)

| Failure | Symptoms | Mitigation |
| --- | --- | --- |
| Feature-poor environment | RTAB-Map confidence drops, odom drift | Blend IMU-only integration temporarily; slow vehicle and request re-localization |
| Loop closure divergence | Sudden map jump affecting Nav2 | Freeze new goals, reset RTAB-Map pose using recent GPS fix or operator input |

## Sensor / Simulation Faults

| Failure | Symptoms | Mitigation |
| --- | --- | --- |
| Gazebo sensor plugin stalls | No camera/IMU updates | Watchdog detects stale timestamps, pauses mission, restarts plugin |
| Hardware sensor misalignment | TF inconsistencies | Calibration check step before mission; fall back to IMU-only odom |

## DDS Bridge / PX4 Interface

| Failure | Symptoms | Mitigation |
| --- | --- | --- |
| DDS bridge disconnect | `/px4/status` heartbeat missing | Safety node commands zero speed and instructs operator to re-establish link |
| PX4 failsafe triggered | PX4 reports manual takeover/failsafe | Halt mission, hand control to operator, log event |
| Command saturation | ackermann_controller exceeds PX4 limits | Bridge clamps to PX4-configured speed/yaw rates, reports warning |

## Mission Control

| Failure | Symptoms | Mitigation |
| --- | --- | --- |
| Nav2 goal unreachable (map gap) | Planner oscillates | Switch to teleop/manual, refine map |
| Safety watchdog trip | `/safety/fault` true | px4-offboard sends brake command, Ackermann controller holds zero speed until cleared |
