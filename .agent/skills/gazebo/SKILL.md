---
name: Gazebo Simulation
description: Instructions for checking, managing, and debugging the Gazebo Harmonic (gz) simulator.
---

# Gazebo Simulation Skill

Gazebo is used to simulate the physics and sensors of the robot. This project uses **Gazebo Harmonic** (the modern `gz` CLI, not the classic `gazebo` / `roslaunch` commands).

## 1. Launching Simulation

Gazebo is always started through the ROS 2 launch system — do not run `gz sim` directly unless isolating a specific failure.

```bash
# Minimal Gazebo + ros2_control only (no SLAM, no Nav2)
ros2 launch robot_bringup robot_bringup.launch.py rtabmap:=false nav2:=false

# Disable RViz when debugging to reduce overhead
ros2 launch robot_bringup robot_bringup.launch.py rtabmap:=false nav2:=false rviz:=false

# Full simulation stack
ros2 launch robot_bringup robot_bringup.launch.py rtabmap:=true nav2:=true

# From host (preferred):
./scripts/start_ros2_nodes.sh
./scripts/start_ros2_nodes.sh --rtabmap --nav2
```

Always wait **5–10 seconds** after launch before validating TF trees or sensor topics — Gazebo takes time to fully load the world and spawn the URDF.

## 2. Validating Simulation Physics
When Gazebo is launched, the `ros_gz_bridge` connects Gazebo transport topics to ROS 2 topics.
If sensor data or robot movement fails, introspect Gazebo directly.

```bash
# List all active Gazebo topics
gz topic -l

# Echo a Gazebo topic (1 message)
gz topic -e -t "/model/ackermann/odometry_with_covariance" -n 1

# Check IMU sensor data (z should be ≈ -9.81 or +9.81 depending on convention)
gz topic -e -t "/world/warehouse/model/ackermann/link/base_link/sensor/imu_sensor/imu" -n 1

# Check motor command arriving at Gazebo
gz topic -e -t /model/ackermann/command/motor_speed -n 3

# Check steering servo
gz topic -e -t /model/ackermann/servo_0 -n 3

# Check rover position (confirm movement)
gz topic -e -t /model/ackermann/odometry_with_covariance -n 1 | grep -A3 'position {'

# List Gazebo resources
gz resource list
```

## 3. ROS ↔ Gazebo Bridge Topics
The `ros_gz_bridge` exposes these Gazebo sensors as ROS 2 topics:

| ROS 2 Topic | Gazebo Source | Type |
|-------------|---------------|------|
| `/ackermann/depth_camera/image` | gz depth camera RGB | `sensor_msgs/Image` |
| `/ackermann/depth_camera/depth_image` | gz depth camera depth | `sensor_msgs/Image` |
| `/ackermann/depth_camera/camera_info` | gz camera info | `sensor_msgs/CameraInfo` |
| `/l515/imu/raw` | gz IMU sensor | `sensor_msgs/Imu` |
| `/rplidar/scan` | gz LiDAR | `sensor_msgs/LaserScan` |
| `/ackermann/odom` | gz odometry | `nav_msgs/Odometry` |
| `/clock` | gz simulation clock | `rosgraph_msgs/Clock` |

## 4. Common Issues & Debugging

**Robot Not Moving:**
- Check `ros2 topic echo /ackermann/cmd_vel` shows commands.
- Check `ros2 node list | grep bridge` — ros_gz_bridge must be running.
- Check Gazebo side: `gz topic -e -t /model/ackermann/cmd_vel`.

**No Lidar/Camera Data:**
- Same bridging issue. Check `gz topic -e -t /lidar/points`.

**Gazebo Crashes:**
- Check startup logs heavily. Look for plugin loading errors (`[Err] [Plugin...]`) or missing mesh files.
- Check for duplicate `/clock` publishers: `ros2 topic info /clock`.

**RViz Resets / TF Time Jump:**
- Symptom: `Detected jump back in time. Resetting RViz.`
- Cause: Multiple conflicting simulation stacks (multiple `/clock` publishers).
- Fix:
  ```bash
  pkill -f "ros2 launch robot_bringup" || true
  pkill -f "rviz2" || true
  pkill -f "gz sim" || true
  ```
  Then start a single clean bringup instance.

## 5. PX4 SITL with Gazebo

When `enable_px4_sitl:=true`, PX4 SITL co-simulates inside Gazebo — the `gz_bridge` module links PX4 directly to Gazebo without a ROS bridge.

**Startup order is critical:**
1. Start Gazebo: `ros2 launch robot_bringup robot_bringup.launch.py enable_px4_sitl:=true`
2. Start MicroXRCEAgent: `./scripts/start_microxrce_agent.sh`
3. Start PX4 SITL: `./scripts/start_px4_sitl.sh` (or `--vio` for VIO-only EKF2)
4. Start PX4 bridge: `ros2 launch px4_bringup px4_bringup.launch.py mode_type:=speed_steering`

**Or from host with a single command:**
```bash
./scripts/start_ros2_nodes.sh --px4
./scripts/start_ros2_nodes.sh --px4 --rtabmap --nav2
```

**PX4 Airframe:** 51000 (`gz_rover_ackermann`)
**World:** `warehouse.sdf` (contains `<spherical_coordinates>` for GPS reference)

**Key PX4 ↔ Gazebo sensor topic paths (for PX4's gz_bridge):**
```
/world/warehouse/model/ackermann/link/base_link/sensor/imu_sensor/imu
/world/warehouse/model/ackermann/link/base_link/sensor/air_pressure_sensor/air_pressure
/world/warehouse/model/ackermann/link/base_link/sensor/magnetometer_sensor/magnetometer
/world/warehouse/model/ackermann/link/base_link/sensor/navsat_sensor/navsat
```

**PX4 actuator topic paths (from gz_bridge):**
- Motor: `/model/ackermann/command/motor_speed`
- Steering: `/model/ackermann/servo_0`

## 6. T265 Fisheye Simulation Bridges
In simulation, the T265 fisheye cameras and IMU are bridged via `gazebo_bringup.launch.py` when `enable_t265:=true` or `use_cuvslam_odom:=true`:

```bash
# Ros 2 topics produced by sim T265 bridges:
/t265/fisheye1/image_raw
/t265/fisheye2/image_raw
/t265/fisheye1/camera_info
/t265/fisheye2/camera_info
/t265/imu
```

## 7. Stopping All Simulation Processes
```bash
# From host — kills all ROS 2, Gazebo, PX4, and MicroXRCE processes inside the container:
docker compose -f docker/docker-compose.yml exec ackermann_slam bash -c \
  "pkill -9 -f 'ros2|rviz2|gz|ruby|px4|MicroXRCE'"
```

> **CRITICAL:** Always terminate the simulation after completing any task. Leaving Gazebo or ROS nodes running causes `/clock` conflicts and stale process issues for future runs.
