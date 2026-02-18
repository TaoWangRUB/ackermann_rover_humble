---
title: Interfaces
status: Draft
owner: architecture_team
agent: Copilot
last_updated: 2026-02-17
doc_type: architecture
ros_distro: humble
---
## Sensor Interfaces

| Source | Topic | Type | Notes |
| --- | --- | --- | --- |
| Gazebo camera plugin | `/sim/camera/image(raw)` | sensor_msgs/msg/Image | Stereo/RGB-D, `frame_id`=`camera_link` |
| Gazebo IMU | `/sim/imu` | sensor_msgs/msg/Imu | Includes orientation + angular velocity |
| Gazebo GPS | `/sim/gps/fix` | sensor_msgs/msg/NavSatFix | Used to anchor map frame, optional |
| Ground truth | `/sim/ground_truth/pose` | geometry_msgs/msg/PoseStamped | Debug only; not consumed in production |

Sensor bridge remaps these to canonical topics (`/camera/front/image`, `/imu/data`, `/gps/fix`) with ROS 2 QoS overrides.

## RTAB-Map Interfaces

| Topic | Direction | Type | Purpose |
| --- | --- | --- | --- |
| `/rtabmap/odom` | out | nav_msgs/msg/Odometry | High-rate VIO solution for Nav2/TF |
| `/rtabmap/mapData` | out | rtabmap_msgs/msg/MapData | Loop closures, graph data |
| `/tf`, `/tf_static` | out | tf2_msgs/msg/TFMessage | Frames: `map`, `odom`, `base_link` |
| `/rtabmap/goal` | in | geometry_msgs/msg/PoseStamped | Optional localization goal reset |

## Navigation & Control

| Topic | Producer → Consumer | Type | Notes |
| --- | --- | --- | --- |
| `/cmd_vel_nav` | Nav2 → ackermann_control | geometry_msgs/msg/Twist | Nav2 computed velocity commands |
| `/cmd_ackermann` | ackermann_control → px4-offboard bridge | ackermann_msgs/msg/AckermannDriveStamped | Contains target speed + steering angle |
| `/safety/fault` | Watchdog → px4-offboard bridge | std_msgs/msg/Bool | Forces zero-speed when true |

## PX4 DDS Bridge

Topics/services exposed by https://github.com/TaoWangRUB/px4-offboard (simplified until APIs stabilize):

| Interface | Direction | Description |
| --- | --- | --- |
| `/px4/setpoint/ackermann` | in | DDS bridge consumes Ackermann setpoints and converts to PX4 offboard commands |
| `/px4/status` | out | Heartbeat, mode, failsafe flags |
| `/px4/actuator_feedback` | out | Telemetry used by watchdog/logging |
| `/px4/arm` | service (future) | Arm/disarm request (placeholder) |

Bridge parameters: `dds_domain_id`, `px4_uav_id`, `command_rate_hz`. Defaults target PX4 SITL; switching to hardware updates host/ports only.

## Simulation ↔ Hardware Switches

- `use_sim_time` parameter toggled globally.
- Sensor bridge packages support plugin overrides vs hardware drivers.
- DDS bridge selects SITL vs hardware endpoints via parameter file under `config/px4_bridge.yaml` (to be added).
