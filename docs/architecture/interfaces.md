---
title: Interfaces
status: Draft
owner: architecture_team
agent: Copilot
last_updated: 2026-02-18
doc_type: architecture
ros_distro: humble
---
## Gazebo ↔ ROS 2 Bridge

`robot_description/launch/gazebo_bringup.launch.py` launches `ros_gz_bridge parameter_bridge` so Gazebo topics land directly on the ROS graph. All entities are prefixed with `ackmann` to simplify multi-robot simulation.

| Gazebo Topic | ROS 2 Topic | ROS Type | Notes |
| --- | --- | --- | --- |
| `/world/default/clock` | `/clock` | rosgraph_msgs/msg/Clock | Drives `use_sim_time` for every node in `robot_bringup` |
| `/ackmann/depth_camera/image` | `/ackmann/depth_camera/image` | sensor_msgs/msg/Image | RGB stream processed by `rgbd_sync` |
| `/ackmann/depth_camera/depth_image` | `/ackmann/depth_camera/depth_image` | sensor_msgs/msg/Image | Registered depth image |
| `/ackmann/depth_camera/camera_info` | `/ackmann/depth_camera/camera_info` | sensor_msgs/msg/CameraInfo | Shared by RGB, depth, and laserscan nodes |
| `/l515/imu/raw` | `/l515/imu/raw` | sensor_msgs/msg/Imu | Raw IMU prior to frame alignment |
| `/rplidar/scan` | `/rplidar/scan` | sensor_msgs/msg/LaserScan | Alternative/backup planar scan |
| `/ackmann/odom` | `/ackmann/odom` | nav_msgs/msg/Odometry | Ground-truth odom used for validation only |

Resource paths (`GZ_SIM_RESOURCE_PATH`, etc.) are extended so Gazebo can discover the robot meshes and external sensor descriptions (e.g., `realsense2_description`).

## Sensor Conditioning Interfaces

| Node | Input Topic(s) | Output Topic(s) | Type(s) | Purpose |
| --- | --- | --- | --- | --- |
| `rgbd_sync` (`rtabmap_sync`) | `/ackmann/depth_camera/image`, `/ackmann/depth_camera/depth_image`, `/ackmann/depth_camera/camera_info` | `/rgb/image`, `/depth/image`, `/rgb/camera_info`, `/depth/camera_info` | sensor_msgs/msg/Image, sensor_msgs/msg/CameraInfo | Provides tightly time-synchronized RGB-D streams |
| `depthimage_to_laserscan` | `/ackmann/depth_camera/depth_image`, `/ackmann/depth_camera/camera_info` | `/scan` | sensor_msgs/msg/LaserScan | Generates planar scan for ICP odom or redundant sensing |
| `imu_transformer` | `/l515/imu/raw` | `/l515/imu/raw_transformed` | sensor_msgs/msg/Imu | Aligns IMU data with `ackmann/base_footprint` |
| `imu_filter_madgwick` | `/l515/imu/raw_transformed` | `/imu/data` | sensor_msgs/msg/Imu | Filters and outputs orientation + angular velocity for EKF |

## RTAB-Map & State Estimation Interfaces

| Topic | Direction | Type | Producer | Notes |
| --- | --- | --- | --- | --- |
| `/vo_odom` | out | nav_msgs/msg/Odometry | `rtabmap_odom` (RGB-D) | Selected when `vision=true` |
| `/icp_odom` | out | nav_msgs/msg/Odometry | `rtabmap_odom` (ICP) | Selected when `vision=false` |
| `/odometry/filtered` | out | nav_msgs/msg/Odometry | `robot_localization` | Primary odom for Nav2 and RTAB-Map |
| `/rtabmap/odom` | out | nav_msgs/msg/Odometry | `rtabmap_slam` | Pose in `map` frame (SLAM mode) |
| `/rtabmap/mapData`, `/rtabmap/mapGraph`, `/rtabmap/mapPath` | out | rtabmap_msgs/msg/Map* | `rtabmap_slam` | Graph + occupancy data |
| `/tf`, `/tf_static` | out | tf2_msgs/msg/TFMessage | `robot_state_publisher`, RTAB-Map | TF tree linking `map` → `odom` → `ackmann/base_footprint` |
| `/rtabmap/goal` | in | geometry_msgs/msg/PoseStamped | Mission manager | Optional localization goal reset |
| `/odom` | in | nav_msgs/msg/Odometry | RTAB-Map | Remaps to `/odometry/filtered` during SLAM |
| `/imu/data` | in | sensor_msgs/msg/Imu | RTAB-Map + EKF | Orientation constraints |
| `/scan` | in | sensor_msgs/msg/LaserScan | RTAB-Map | Only used when in ICP mode |

Launch arguments:

- `vision` toggles between `/vo_odom` (RGB-D) and `/icp_odom` (ICP).
- `localization` switches RTAB-Map to localization-only mode (no new nodes in the map graph).
- `rtabmap_viz` gates the visualization client for debugging.

## Navigation & Control Interfaces

| Topic | Producer → Consumer | Type | Notes |
| --- | --- | --- | --- |
| `/cmd_vel` | Nav2 → ackermann_control | geometry_msgs/msg/Twist | Launch remaps this to `/cmd_vel_nav` for clarity |
| `/cmd_vel_nav` | Nav2 → ackermann_control | geometry_msgs/msg/Twist | Explicit alias used inside ackermann_control params |
| `/cmd_ackermann` | ackermann_control → px4-offboard bridge | ackermann_msgs/msg/AckermannDriveStamped | Speed + steering commands, constrained by wheelbase + limits |
| `/safety/fault` | Watchdog → ackermann_control & px4-offboard | std_msgs/msg/Bool | When true, controller publishes zero speed and bridge drops setpoints |

`ackermann_control` parameters expose `input_cmd_vel_topic` and `output_ackermann_topic` so hardware deployments can rename topics without touching Nav2 config.

## PX4 DDS Bridge Interfaces

Topics/services exposed by https://github.com/TaoWangRUB/px4-offboard (simplified until APIs stabilize):

| Interface | Direction | Description |
| --- | --- | --- |
| `/px4/setpoint/ackermann` | in | DDS bridge consumes Ackermann setpoints and converts to PX4 offboard commands |
| `/px4/status` | out | Heartbeat, mode, failsafe flags |
| `/px4/actuator_feedback` | out | Telemetry used by watchdog/logging |
| `/px4/arm` | service (future) | Arm/disarm request (placeholder) |

Bridge parameters: `dds_domain_id`, `px4_uav_id`, `command_rate_hz`. Defaults target PX4 SITL; switching to hardware updates host/ports only.

## Simulation ↔ Hardware Switches

- `use_sim_time` parameter toggled globally by `robot_bringup`.
- Sensor interfaces remain identical between Gazebo and hardware; replace Gazebo plugins with hardware drivers that publish on the same `/ackmann/*`, `/l515/imu/raw`, `/scan` topics.
- DDS bridge selects SITL vs hardware endpoints via the upcoming `config/px4_bridge.yaml` parameter file.
