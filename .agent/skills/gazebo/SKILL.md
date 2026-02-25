---
name: Gazebo Simulation
description: Instructions for checking, managing, and debugging the Gazebo Harmonic (gz) simulator.
---

# Gazebo Simulation Skill

Gazebo is used to simulate the physics and sensors of the robot. This project uses the modern Gazebo, which prefixes its commands with `gz` (not the classic `gazebo` commands).

## 1. Validating Simulation Physics
When Gazebo is launched, the `ros_gz_bridge` connects Gazebo transport topics to ROS 2 topics. 
If sensor data or robot movement fails, you often need to introspect Gazebo directly.

- **Check Gazebo Topics list**:
  ```bash
  gz topic -l
  ```
- **Echo a Gazebo Topic**:
  ```bash
  gz topic -e -t "/gazebo/topic_name"
  ```
- **Check Gazebo World info**:
  ```bash
  gz resource list
  ```

## 2. Common Issues & Debugging
- **Robot Not Moving**: If `ros2 topic echo /cmd_vel` shows commands, but the robot doesn't move, check if the `ros_gz_bridge` is running (`ros2 node list | grep bridge`). If the bridge is up, check if `gz topic -e -t /model/ackermann/cmd_vel` is receiving data.
- **No Lidar/Camera Data**: Same bridging issue; check if the Gazebo sensors are actively publishing (`gz topic -e -t /lidar/points`).
- **Gazebo Crashes**: Check the startup logs heavily. Look for plugin loading errors (`[Err] [Plugin...]`) or missing mesh files (URDF/SDF errors).

## 3. Gazebo Launching Best Practices
In this project, Gazebo is typically launched via a ROS 2 python launch file (`ros_gz_sim`). You do not normally run `gz sim` manually as an agent unless isolating a failure.

If you need to test just the simulation without Nav2/RTAB-Map:
```bash
ros2 launch robot_bringup robot_bringup.launch.py rtabmap:=false nav2:=false
```

Always give Gazebo ~5-10 seconds to fully load the world and spawn the URDF before validating TF trees or sensor topics.
