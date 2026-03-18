# Autonomous Rover (ROS 2 Humble, Ackermann)

Production-grade template for an Ackermann-steered autonomous rover.
Includes Nav2, localization, safety, CI, and AI-assisted workflows.

## Docker Image Usage

This repo includes a Dockerfile and compose stack under `docker/` for a ROS 2 + Gazebo (gz) dev image. Defaults target ROS 2 Jazzy with Gazebo Harmonic, and all variants are configured via `docker/.env`.

1. Edit `docker/.env` if you need different distributions. Set `UBUNTU_VERSION` to the matching base image, `ROS_UBUNTU_CODENAME` to the apt codename for that Ubuntu release, and adjust `ROS_DISTRO` / `GZ_DISTRO` accordingly. (Example: for ROS Iron on Ubuntu 22.04, set `UBUNTU_VERSION=22.04`, `ROS_UBUNTU_CODENAME=jammy`, `ROS_DISTRO=iron`, `GZ_DISTRO=fortress`.)
2. Build the image using the compose file in `docker/`:
	- `docker-compose -f docker/docker-compose.yml build ackermann_slam`

> Note: The Dockerfile is tuned for ROS 2 Jazzy on Ubuntu 24.04 (Noble). Other distros may require additional tweaks (base image / apt repo codename) and are not guaranteed to work out of the box.

Run interactively (recommended during development):

- `xhost +local:root` (host, once per session)
- `docker-compose -f docker/docker-compose.yml run --rm ackermann_slam`

Or keep the container running in the background and exec in:

- `docker-compose -f docker/docker-compose.yml up -d ackermann_slam`
- `docker-compose -f docker/docker-compose.yml exec ackermann_slam bash`

When you're done, stop and clean up the stack with:

- `docker-compose -f docker/docker-compose.yml down`

Inside the container, the workspace is mounted at `/workspace` and you can run `colcon build` and `ros2 launch robot_bringup robot_bringup.launch.py ...` as usual.

## Troubleshooting: RViz Crash / TF Time Jump

If RViz repeatedly resets with `Detected jump back in time` or crashes during bringup, the most common cause is multiple simulation stacks running at once (multiple `/clock` publishers).

### Symptoms

- RViz logs: `Detected jump back in time. Resetting RViz.`
- TF logs: `Detected jump back in time. Clearing TF buffer.`
- RViz may exit with `exit code -11`

### Root Cause

When more than one `ros2 launch robot_bringup ...` (or `gz sim`) process is active, simulation time can jump backward between publishers. RViz/TF consumers then continuously reset.

### Fix

1. Stop all running launch/sim processes in the container:

```bash
pkill -f "ros2 launch robot_bringup" || true
pkill -f "rviz2" || true
pkill -f "gz sim" || true
```

2. Verify no stale processes remain:

```bash
pgrep -af "ros2 launch robot_bringup|rviz2|gz sim"
```

3. Start one clean bringup instance only:

```bash
ros2 launch robot_bringup robot_bringup.launch.py \
	enable_d435i:=false \
	enable_t265:=false \
	enable_cubepilot:=true \
	rtabmap:=false nav2:=false
```

### Prevention

- Always stop previous bringup runs (`Ctrl+C`) before starting a new one.
- Avoid launching the same stack from multiple terminals at the same time.
- If debugging, run `rviz:=false` first, confirm stability, then launch RViz in a separate step.

## Stopping All ROS 2 Nodes

To stop all running ROS 2 nodes, Gazebo, and related processes inside the Docker container:

```bash
docker-compose -f docker/docker-compose.yml exec ackermann_slam bash -c "pkill -9 -f 'ros2|gz|ruby'"
```

This kills:
- `ros2` — all ROS 2 nodes and launch processes
- `gz` — Gazebo simulator
- `ruby` — Gazebo's internal Ruby processes

Always run this before starting a new simulation session to avoid stale processes or `/clock` conflicts.

## PX4 Bridge (`px4_bringup`)

The `px4_bringup` package provides C++ bridges between the ROS 2 navigation stack and PX4 autopilot. Four C++ custom flight modes are implemented using the [`px4_ros2_interface_lib`](https://github.com/Auterion/px4-ros2-interface-lib) (added as a git submodule). Two Python odometry bridges are available:

- **`px4_vision_odom.py`** — converts `nav_msgs/Odometry` (ENU/FLU) to `VehicleOdometry` (NED/FRD) and publishes to `/fmu/in/vehicle_visual_odometry` via Micro-XRCE-DDS. The `odom_topic` argument selects the input odometry source.
- **`px4_vehicle_odometry.py`** — subscribes to `/fmu/out/vehicle_odometry` (PX4 EKF2 output, NED/FRD) and converts to ENU/FLU for ROS 2, publishing on `/px4_vehicle_odom` and `/px4_vehicle_odom_base` for comparison with EKF odometry.

Both nodes are launched together by `--vo-bridge`.

### Modes

| Executable                  | PX4 Mode Type                          | `/cmd_vel` Mapping                                       | Use Case                          |
| --------------------------- | -------------------------------------- | -------------------------------------------------------- | --------------------------------- |
| `offboard_trajectory_mode`  | Offboard (TrajectorySetpoint)          | TF2 body→odom (ENU→NED) velocity                         | Generic offboard velocity control |
| `rover_speed_steering_mode` | Custom registered (RoverSpeedSteering) | `linear.x` → speed, `angular.z` → normalized steering    | **Recommended for Ackermann**     |
| `rover_speed_attitude_mode` | Custom registered (RoverSpeedAttitude) | `linear.x` → speed, `angular.z` integrated → yaw heading | Heading-hold driving              |
| `rover_manual_mode`         | Custom registered (RoverManual)        | Pass-through throttle + steering                          | Manual teleoperation              |

### Launch

```bash
# Mode node only (default: manual)
ros2 launch px4_bringup px4_bringup.launch.py

# Specific mode
ros2 launch px4_bringup px4_bringup.launch.py mode_type:=speed_steering

# VO bridge only (no mode node)
ros2 launch px4_bringup px4_bringup.launch.py enable_mode_node:=false enable_vo_bridge:=true

# Mode + VO bridge
ros2 launch px4_bringup px4_bringup.launch.py mode_type:=speed_steering enable_vo_bridge:=true

# From host via helper script:
./scripts/start_px4_bringup_vo.sh --bridge --mode-type speed_steering
./scripts/start_px4_bringup_vo.sh --vo-bridge
./scripts/start_px4_bringup_vo.sh --bridge --vo-bridge
```

### Odometry Bridge

`px4_vision_odom.py` converts `nav_msgs/Odometry` (ENU/FLU) to PX4 `VehicleOdometry` (NED/FRD). `px4_vehicle_odometry.py` does the inverse — PX4 EKF2 output back to ROS 2 frames for diagnostics. See [Architecture Overview](docs/architecture/overview.md) for the full coordinate conversion reference.

## Architecture Blueprints

The full system architecture (Gazebo ↔ RTAB-Map interfaces, topic contracts, and node graph) lives in the `docs/architecture` directory:

- [Architecture Overview](docs/architecture/overview.md)
- [Node Graph](docs/architecture/node_graph.md)
- [Interfaces Reference](docs/architecture/interfaces.md)

These documents stay current with the `robot_bringup` launch composition, ros_gz bridge topics, and RTAB-Map + Nav2 integrations—start there when wiring new hardware or extending the stack.
