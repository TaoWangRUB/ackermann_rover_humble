---
name: ROS 2 Development
description: Essential commands and workflows for building, testing, and debugging ROS 2 workspaces.
---

# ROS 2 Development Skill

This skill provides instructions on how to interact with, build, test, and debug ROS 2 code in the **ackermann_rover_humble** project. All commands should be run inside the Docker container unless stated otherwise.

## 1. Environment Setup
Before running ROS 2 commands, always source the workspace:
```bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
```
The Docker entrypoint does this automatically when you `exec` into the container. If sourcing fails because the workspace isn't built yet, run `colcon build --symlink-install` first.

## 2. Building the Workspace
ROS 2 uses `colcon` as its build tool.

- **Build all packages**:
  ```bash
  colcon build --symlink-install
  ```
- **Build specific packages** (faster during iterative development):
  ```bash
  colcon build --symlink-install --packages-select <package_name1> <package_name2>
  ```
- **Build packages up to a specific package** (builds dependencies too):
  ```bash
  colcon build --symlink-install --packages-up-to <package_name>
  ```
- **Via host launcher** (preferred — handles workspace sourcing):
  ```bash
  ./scripts/start_ros2_nodes.sh --build-only
  ./scripts/start_ros2_nodes.sh --build-only=pkg1,pkg2
  ./scripts/start_ros2_nodes.sh --build=<pkg>   # build then launch
  ```

## 3. Testing
ROS 2 uses `colcon test` to run tests (gtest/pytest).

- **Run all tests**:
  ```bash
  colcon test --event-handlers console_direct+
  ```
- **Run tests for a specific package**:
  ```bash
  colcon test --event-handlers console_direct+ --packages-select <package_name>
  ```
- **Check test results** — MUST do this after running tests:
  ```bash
  colcon test-result --verbose
  ```
- **Run the full agent verification suite** (required before marking any task Done):
  ```bash
  bash scripts/verify_agent_work.sh
  ```

## 4. Debugging & Introspection
- **Topics**:
  - `ros2 topic list`
  - `ros2 topic info /topic_name`
  - `ros2 topic echo /topic_name --once`
  - `ros2 topic hz /topic_name`
- **Nodes**:
  - `ros2 node list`
  - `ros2 node info /node_name`
- **Parameters**:
  - `ros2 param list`
  - `ros2 param get /node_name param_name`
  - `ros2 param set /node_name param_name value`
- **Services/Actions**:
  - `ros2 service list` & `ros2 service call ...`
  - `ros2 action list` & `ros2 action send_goal ...`
- **GUI Tools** (requires `xhost +local:root`):
  - `rqt_graph` — visualize node/topic graph
  - `rqt_tf_tree` — visualize TF frames
- **TF2 Introspection**:
  - `ros2 run tf2_ros tf2_echo <source_frame> <target_frame>`
  - `ros2 run tf2_tools view_frames` — generates `frames.pdf`
  - `ros2 run tf2_ros tf2_monitor`

## 5. Lifecycle Nodes
Nav2 and hardware interfaces use Managed (Lifecycle) nodes.

```bash
ros2 lifecycle get /node_name
ros2 lifecycle set /node_name configure
ros2 lifecycle set /node_name activate
```

## 6. Launch Files
Standard pattern:
```bash
ros2 launch <package_name> <launch_file> [arg:=value ...]
```

### Key launch files in this project:

| Command | Effect |
|---------|--------|
| `ros2 launch robot_bringup robot_bringup.launch.py` | Top-level Gazebo bringup |
| `ros2 launch robot_bringup robot_bringup.launch.py rtabmap:=true nav2:=true` | + SLAM + Nav2 |
| `ros2 launch robot_bringup robot_bringup.launch.py use_cuvslam_odom:=true rtabmap:=true` | + cuVSLAM odometry |
| `ros2 launch robot_bringup robot_bringup.launch.py enable_px4_sitl:=true` | PX4 SITL mode |
| `ros2 launch px4_bringup px4_bringup.launch.py` | PX4 ROS 2 bridge (manual mode) |
| `ros2 launch px4_bringup px4_bringup.launch.py mode_type:=speed_steering` | PX4 speed-steering mode |
| `ros2 launch px4_bringup px4_bringup.launch.py enable_vo_bridge:=true` | PX4 VO bridge |

### robot_bringup.launch.py key arguments:
| Argument | Default | Effect |
|----------|---------|--------|
| `use_gazebo` | `true` | Simulation vs hardware |
| `enable_px4_sitl` | `false` | PX4 SITL mode (disables ros2_control) |
| `rtabmap` | `false` | Launch RTAB-Map SLAM |
| `nav2` | `false` | Launch Nav2 |
| `rviz` | `true` | Launch RViz |
| `use_cuvslam_odom` | `false` | cuVSLAM as primary odom (auto-enables T265 fisheye) |
| `use_vins_odom` | `false` | VINS-Fusion as primary odom |
| `use_t265_odom` | `false` | T265 built-in odom |
| `depth_camera` | `d435i` | Select depth camera (`d435i` or `l515`) |

## 7. Host Launcher (preferred entry point)
The script `scripts/start_ros2_nodes.sh` is the recommended way to start any combination:

```bash
./scripts/start_ros2_nodes.sh                                 # Gazebo only
./scripts/start_ros2_nodes.sh --rtabmap                       # + RTAB-Map
./scripts/start_ros2_nodes.sh --rtabmap --nav2                # + Nav2
./scripts/start_ros2_nodes.sh --rtabmap --nav2 --no-rviz      # + no RViz
./scripts/start_ros2_nodes.sh --rtabmap --vo-bridge           # + PX4 VO bridge
./scripts/start_ros2_nodes.sh --px4                           # PX4 SITL full stack
./scripts/start_ros2_nodes.sh --hw --depth-camera=d435i --t265 --rtabmap  # Hardware
./scripts/start_ros2_nodes.sh --hw --cuvslam-odom --rtabmap   # Hardware + cuVSLAM
./scripts/start_ros2_nodes.sh --cuvslam-odom --rtabmap        # Sim + cuVSLAM
```

## 8. VIO / Camera Debug Probe
While the stack is running, use the VIO diagnostic script to inspect all camera/odom topics:
```bash
./scripts/debug_vio.sh

# Shorter probe window
HZ_WINDOW=5 ECHO_TIMEOUT=3 ./scripts/debug_vio.sh
```
This auto-detects D435i, L515, T265, VINS-Fusion, cuVSLAM, IMU, VO, EKF, and map topics, prints timestamp samples and per-topic stats.

## 9. Driving the Robot

**ros2_control mode** (default, `enable_px4_sitl:=false`):
```bash
ros2 topic pub -r 1 /ackermann/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'ackermann/base_link'}, twist: {linear: {x: 1.0}, angular: {z: 0.5}}}"
```

**PX4 mode** (cmd_vel → PX4 bridge):
```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 1.0}, angular: {z: 0.0}}'
```

## 10. TF Frame Contract
The expected TF tree for this project is:
```
map → odom → ackermann/base_link
```
- `map → odom`: published by RTAB-Map SLAM
- `odom → ackermann/base_link`: published by each odometry source (RTAB-Map VO / EKF / T265 relay / VINS relay / cuVSLAM relay)

## 11. Coordinate Frame Conventions
All ROS nodes in this project use ENU/FLU. The PX4 bridge nodes handle conversion to NED/FRD.

| Data | ROS Frame | PX4 Frame | Conversion Rule |
|------|-----------|-----------|-----------------|
| World position | ENU `(x,y,z)` | NED | `(y, x, -z)` |
| Body linear vel | FLU `(x,y,z)` | FRD | `(x, -y, -z)` |
| Body angular vel | FLU `(x,y,z)` | FRD | `(x, -y, -z)` |
| Orientation quat | FLU→ENU | FRD→NED | `q_ENU→NED · q · inv(q_FLU→FRD)` |

See `docs/architecture/overview.md` for the full derivation.
