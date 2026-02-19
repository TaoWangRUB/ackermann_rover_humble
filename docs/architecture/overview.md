---
title: Architecture Overview
status: Draft
owner: architecture_team
agent: Copilot
last_updated: 2026-02-18
doc_type: architecture
ros_distro: humble
---
## Mission Profile

Autonomous Ackermann rover with UAV-grade autonomy stack. Primary workflow:

1. **Simulation-first**: `robot_bringup.launch.py` boots Gazebo via `description_robot` and keeps ROS ↔ Gazebo resources discoverable through `GZ_SIM_RESOURCE_PATH`. The ros_gz parameter bridge exports depth camera, IMU, LiDAR, odometry, and `/clock` directly into ROS 2 topics the rest of the stack consumes.
2. **Perception & Localization**: `rtabmap_bringup` orchestrates RGB-D synchronization, IMU preprocessing, VIO/ICP odometry, robot_localization fusion, and RTAB-Map SLAM or localization-only mode. Outputs include TF (`map`→`odom`→`ackermann/base_footprint`), `/rtabmap/odom`, `/odometry/filtered`, and map data for Nav2.
3. **Planning & Control**: Nav2 planners consume RTAB-Map localization and costmaps to generate `/cmd_vel` (aliased to `/cmd_vel_nav`). The Ackermann controller converts twist commands to steering/speed profiles on `/cmd_ackermann`.
4. **Vehicle Interface**: The px4-offboard DDS bridge relays high-level Ackermann commands to PX4 (SITL or hardware), enforces heartbeat-based failsafes, and reports telemetry the watchdog consumes.

## Core Subsystems

| Layer | Responsibilities | Key Nodes/Tools |
| --- | --- | --- |
| Simulation & Bridge | Gazebo physics, sensor plugins, ros_gz parameter bridge, TF priming | `gazebo_bringup.launch.py`, ros_gz_sim, ros_gz_bridge, joint_state_publisher, robot_state_publisher |
| Sensor Conditioning | Sync RGB-D, convert depth→scan, align IMU frames, filter IMU | `rgbd_sync`, `depthimage_to_laserscan`, `imu_transformer`, `imu_filter_madgwick` |
| Localization & Mapping | VIO/ICP odom, EKF fusion, loop-closure SLAM, TF publishing | `rtabmap_odom`, `robot_localization`, `rtabmap_slam`, `rtabmap_viz` |
| Navigation & Control | Global + local planners, Ackermann conversion | Nav2 stack, `ackermann_control` package |
| Safety & Vehicle Interface | Command gating, DDS bridge to PX4, telemetry | Safety watchdog, `px4-offboard` DDS bridge, PX4 firmware |

## Data Flow Summary

- ros_gz parameter bridge exports `/ackermann/depth_camera/image`, `/ackermann/depth_camera/depth_image`, `/ackermann/depth_camera/camera_info`, `/l515/imu/raw`, `/rplidar/scan`, `/ackermann/odom`, and `/clock` into ROS 2.
- `rgbd_sync` aligns the RGB-D stream, and `depthimage_to_laserscan` produces `/scan` so RTAB-Map can switch between vision or ICP pipelines.
- `imu_transformer` re-frames `/l515/imu/raw` into `ackermann/base_footprint`, while `imu_filter_madgwick` provides `/imu/data` for EKF fusion.
- `rtabmap_odom` (RGB-D or ICP) publishes `/vo_odom` or `/icp_odom`; `robot_localization` fuses them into `/odometry/filtered`, and RTAB-Map SLAM emits `/rtabmap/odom`, `/rtabmap/mapData`, and TF.
- Nav2 consumes `/tf`, `/odometry/filtered`, and the map topics to produce `/cmd_vel` (namespaced `/cmd_vel_nav`).
- `ackermann_control` maps `/cmd_vel_nav` to `/cmd_ackermann`, which the px4-offboard DDS bridge translates to PX4 setpoints and relays telemetry (`/px4/status`, `/px4/actuator_feedback`).
- Safety watchdog subscribes to PX4 telemetry and RTAB-Map health to assert `/safety/fault`, forcing zero-speed commands when set.

## Simulation → Hardware Parity

- **Sensors**: Gazebo depth camera + IMU + LiDAR topics mirror hardware drivers. Swapping to real sensors preserves the `/ackermann/*`, `/l515/imu/raw`, and `/scan` contracts so RTAB-Map continues to function.
- **Localization**: `rtabmap_bringup` exposes parameters for localization-only vs SLAM mode, RGB-D vs ICP odometry, and EKF tuning. Field deployments reuse the same launch with overrides for camera intrinsics, IMU bias, and scan settings.
- **Vehicle Interface**: The DDS bridge hosts SITL endpoints by default; deploying to hardware updates `dds_domain_id` and transport endpoints only. `/cmd_ackermann` remains the canonical control contract.
- **Safety**: Watchdog monitors `/px4/status` plus EKF/RTAB-Map diagnostics. Any stale heartbeat or low-confidence localization toggles `/safety/fault`, which `ackermann_control` currently interprets by halting the vehicle (future: gating before px4-offboard).

## Gazebo ↔ RTAB-Map Integration

- `robot_bringup.launch.py` composes `gazebo_bringup` and `rtabmap_slam.launch.py`, ensuring shared launch arguments (`use_sim_time`, pose, namespace) stay consistent across subsystems.
- The ros_gz parameter bridge topics feed directly into the remaps defined inside `rtabmap_bringup`, so no intermediate republishers are required. Topic names intentionally follow the `ackermann/*` prefix to minimize collisions in multi-robot scenarios.
- State estimation runs in three tiers: raw VIO/ICP odom, EKF-smoothed `/odometry/filtered`, and RTAB-Map's global map frame. Nav2 and downstream planners consume `/odometry/filtered` plus the RTAB-Map map server outputs.
- Interfaces to Nav2, safety watchdog, and px4-offboard are now centralized in documentation tables below to keep integrators aligned when topics or QoS policies change.

This overview drives the detailed node graph, interface contracts, and failure mode definitions in the following documents.
