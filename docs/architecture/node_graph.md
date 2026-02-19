---
title: Node Graph
status: Draft
owner: architecture_team
agent: Copilot
last_updated: 2026-02-18
doc_type: architecture
ros_distro: humble
---
## High-Level Graph

![Current ROS graph](../../rosgraph.png)
![TF snapshot](../../frames_2026-02-19_13.42.08.pdf)

```
[Gazebo Sensors]
	│
	▼
[ros_gz_bridge parameter_bridge] ──► `/ackermann/depth_camera/*`, `/l515/imu/raw`, `/rplidar/scan`, `/ackermann/odom`, `/clock`
	│
	▼
[rgbd_sync]
	│
	├─► [rtabmap_odom (RGB-D)] ──► `/vo_odom`
	│
	└─► [depthimage_to_laserscan] ──► `/scan`

[imu_transformer] ─► [imu_filter_madgwick] ──► `/imu/data`

`/vo_odom` or `/icp_odom`, `/imu/data`, `/ackermann/odom`
	│
	▼
[robot_localization EKF] ──► `/odometry/filtered` + TF (`odom → ackermann/base_footprint`)
	│
	▼
[rtabmap_slam (SLAM/localization)] ──► `/rtabmap/*`, `map → odom` TF
	│
	▼
[Nav2 stack (planned)] ──► `/cmd_vel_nav`
	│
	▼
[ackermann_steering_controller] ──► `/ackermann/cmd_vel`
	│
	▼
[Ackermann hardware / Gazebo vehicle]
```

### Components

- **Gazebo Sensors**: Depth camera, IMU, and 2D LiDAR plugins inject the simulated sensor feeds that show up at the top of `rosgraph.png`.
- **ros_gz_bridge `parameter_bridge`**: Bridges those Gazebo Transport streams into ROS topics and propagates `/clock` so everything under `robot_bringup` runs with `use_sim_time=true`.
- **RGB-D Sync + Depth-to-Scan**: `rgbd_sync` time-aligns RGB + depth + camera info; `depthimage_to_laserscan` optionally produces `/scan` when ICP odom is enabled.
- **IMU Conditioning**: `imu_transformer` rewrites IMU frames into `ackermann/base_footprint`, and `imu_filter_madgwick` outputs `/imu/data` for downstream fusion.
- **Localization Stack**: `rtabmap_odom` (RGB-D or ICP), `robot_localization` EKF, and `rtabmap_slam` generate `/odometry/filtered`, `/rtabmap/*`, and TF (`map → odom → ackermann/base_footprint`). The EKF intentionally keeps `/odometry/filtered` as Nav2's odometry source, leaving `/odom` for raw sensor fusion output.
- **Nav2 Stack (planned)**: Consumes `/tf`, `/odometry/filtered`, `/rtabmap/map*`, and `/scan` once its launch file is fully wired, issuing `/cmd_vel_nav` for the controller.
- **Ackermann Controller**: `ackermann_steering_controller` consumes `/ackermann/cmd_vel` (or remapped `/cmd_vel_nav`) and drives the Gazebo hardware via `gz_ros2_control`.

### Notable Links

- `/clock` from ros_gz_bridge remains mandatory for the entire launch stack; every node in `rosgraph.png` listed above was running with `use_sim_time=true`.
- RTAB-Map currently publishes `/map`, `/rtabmap/odom`, `/rtabmap/mapData`, `/rtabmap/mapPath`, and TF between `map`, `odom`, and `ackermann/base_footprint`.
- Nav2 is staged to consume those topics plus `/odometry/filtered` (instead of the raw `/odom` topic) and `/scan`; once enabled it will produce `/cmd_vel_nav` for the Ackermann controller.

Future revisions will capture the Nav2 behavior tree nodes once their configuration lands in this repository.

Future revisions will include a detailed diagram once the DDS topic schema stabilizes.
