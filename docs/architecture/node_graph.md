---
title: Node Graph
status: Draft
owner: architecture_team
agent: Copilot
last_updated: 2026-02-17
doc_type: architecture
ros_distro: humble
---
## High-Level Graph

```
Gazebo Sensors ──► Sensor Bridge ──► RTAB-Map ──► Nav2 ──► Ackermann Controller ──► px4-offboard DDS ──► PX4
		   │             │                │             │                   │                           │
		   └────► PX4 telemetry ◄─────────┴─────────────┴────────────────────┴───────────── Safety Watchdog
```

### Components

- **Gazebo Sensors**: Camera, IMU, GPS, ground-truth pose plugins publish raw data.
- **Sensor Bridge**: Ensures consistent QoS, frame IDs, and clock sync before feeding RTAB-Map and PX4 state estimation.
- **RTAB-Map**: Consumes camera + IMU to produce VIO (`/rtabmap/odom`), loop closures, and map data. Publishes TF between `map`, `odom`, `base_link`.
- **Nav2 Stack**: Global planner + local controllers read TF, costmaps, and mission goals to emit `/cmd_vel_nav`.
- **Ackermann Controller**: Converts `/cmd_vel_nav` (Twist) into AckermannDriveStamped for speed/steering control.
- **px4-offboard DDS Bridge**: Packages Ackermann commands into PX4-compatible DDS topics (from https://github.com/TaoWangRUB/px4-offboard). Maintains heartbeat/status topics.
- **PX4 Firmware**: Runs in SITL (Gazebo) or real hardware, enforcing dynamics and safety limits.
- **Safety Watchdog**: Monitors PX4 heartbeat, fault topics, and RTAB-Map confidence to trigger stop commands.

### Notable Links

- Gazebo time synchronizes all topics; RTAB-Map and PX4 subscribe to `/clock` in simulation.
- RTAB-Map publishes occupancy grid/costmap layers consumed by Nav2 via map server interfaces.
- DDS bridge exposes telemetry (`/px4/status`, `/px4/battery` etc.) back to ROS 2 for monitoring and safety policies.

Future revisions will include a detailed diagram once the DDS topic schema stabilizes.
