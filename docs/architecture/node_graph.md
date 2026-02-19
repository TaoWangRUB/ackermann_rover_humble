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

```
[Gazebo Sensors]
	│
	▼
[ros_gz_bridge parameter_bridge] ──► `/ackermann/depth_camera/*`, `/l515/imu/raw`, `/rplidar/scan`, `/ackermann/odom`, `/clock`
	│
	▼
[rgbd_sync] ─► [rtabmap_odom (RGB-D/ICP)] ─► `/vo_odom` or `/icp_odom`
	│                    │
	│                    └──────────────┐
	▼                                   ▼
[depthimage_to_laserscan]            [robot_localization EKF] ─► `/odometry/filtered`
	│                                   │
	▼                                   ▼
[scan topic]                        [rtabmap_slam or localization] ─► `/rtabmap/odom`, `/rtabmap/mapData`, `/tf`
								│
								▼
							 [Nav2 stack]
								│
								▼
						  `/cmd_vel_nav` (Twist)
								│
								▼
						  [Ackermann Controller]
								│
								▼
						  `/cmd_ackermann`
								│
								▼
						  [px4-offboard DDS bridge]
								│
								▼
							   [PX4]
								▲
								│
				    `/px4/status`, `/px4/actuator_feedback`
								│
								▼
						    [Safety Watchdog]
```

### Components

- **Gazebo Sensors**: Depth camera, IMU, and 2D LiDAR plugins provide the authoritative synthetic sensor feeds.
- **ros_gz_bridge `parameter_bridge`**: Converts Gazebo Transport streams into ROS 2 topics and provides `/clock` so downstream nodes stay time-synchronized.
- **RGB-D Sync + Depth-to-Scan**: `rgbd_sync` keeps RGB + depth + camera info latched, and `depthimage_to_laserscan` provides `/scan` when RTAB-Map runs in ICP mode or for redundancy.
- **IMU Conditioning**: `imu_transformer` enforces `ackermann/base_footprint` frames, while `imu_filter_madgwick` publishes `/imu/data` for the EKF.
- **RTAB-Map + Robot Localization**: `rtabmap_odom` (RGB-D or ICP), `robot_localization` EKF, and `rtabmap_slam` (SLAM/localization modes) together provide odom + map frames, occupancy data, and TF.
- **Nav2 Stack**: Uses `/tf`, `/odometry/filtered`, and RTAB-Map map data to plan and control along mission goals.
- **Ackermann Controller**: Converts Nav2 Twist commands into AckermannDriveStamped commands within configured limits.
- **px4-offboard DDS Bridge**: Forwards `/cmd_ackermann` to PX4 setpoints and republishes telemetry topics into ROS 2.
- **Safety Watchdog**: Subscribes to `/px4/status`, EKF diagnostics, and RTAB-Map health metrics; asserts `/safety/fault` back toward control nodes when needed.

### Notable Links

- `/clock` from the ros_gz bridge is mandatory for all nodes launched through `robot_bringup`; the launch wires `use_sim_time=true` by default.
- RTAB-Map publishes `/map`, `/rtabmap/odom`, `/rtabmap/mapData`, `/rtabmap/mapPath`, and TF between `map`, `odom`, and `ackermann/base_footprint`.
- Nav2 consumes the RTAB-Map map server plus `/odometry/filtered` and goal topics from mission tooling.
- px4-offboard exposes `/px4/status`, `/px4/actuator_feedback`, and (future) `/px4/arm` so the watchdog + operator tools can assess health without cracking open DDS traces.

Future revisions will capture the Nav2 behavior tree nodes once their configuration lands in this repository.

Future revisions will include a detailed diagram once the DDS topic schema stabilizes.
