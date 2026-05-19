# Autonomous Rover (ROS 2 Humble, Ackermann)

Production-grade template for an Ackermann-steered autonomous rover.
Includes Nav2, localization, safety, CI, and AI-assisted workflows.

## Docker Image Usage

This repo includes a Dockerfile and compose stack under `docker/` for a ROS 2 + Gazebo (gz) dev image. Defaults target ROS 2 Jazzy with Gazebo Harmonic on Ubuntu 24.04. The same `docker/docker-compose.yml` works on both x86_64 and Jetson (aarch64) — architecture is detected automatically.

### Prerequisites (one-time host setup)

Add these exports to `~/.bashrc` so docker-compose always picks up the correct values:

```bash
export ARCH=$(uname -m)
export USERNAME=$(id -un)
export USER_UID=$(id -u)
export USER_GID=$(id -g)
```

Then reload: `source ~/.bashrc`

> In each new terminal, run `source ~/.bashrc` (or open a fresh terminal) before running docker-compose commands.

### Configuration

Edit `.env` at the project root to change ROS / Gazebo / Ubuntu versions. For example, for ROS Iron on Ubuntu 22.04: set `UBUNTU_VERSION=22.04`, `ROS_UBUNTU_CODENAME=jammy`, `ROS_DISTRO=iron`, `GZ_DISTRO=fortress`.

### Build

```bash
docker-compose -f docker/docker-compose.yml build ackermann_slam
```

> The Dockerfile is tuned for ROS 2 Jazzy on Ubuntu 24.04 (Noble). Other distros may require additional tweaks and are not guaranteed to work out of the box.

### Run

Allow X11 forwarding (once per host session):

```bash
xhost +local:
```

Keep the container running in the background and exec in:

```bash
docker-compose -f docker/docker-compose.yml up -d ackermann_slam
docker-compose -f docker/docker-compose.yml exec ackermann_slam bash
```

Or run interactively (removed on exit):

```bash
docker-compose -f docker/docker-compose.yml run --rm ackermann_slam
```

Stop and clean up:

```bash
docker-compose -f docker/docker-compose.yml down
```

Inside the container the workspace is mounted at `/workspace`. The container runs as your host user (not root), so all files created inside are owned by you on the host.

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

### PX4 Shell And Parameter Workflow

The default hardware workflow for `px4_cmd.sh` and `upload_params.sh` now uses a host-side MAVProxy bridge on USB and talks to PX4 over `udpin:127.0.0.1:14550`. This avoids reopening the flaky USB CDC endpoint from Python on every command.

Run the bridge once and reuse it for later commands:

```bash
cd /home/taowang/workspace/ackermann_rover_humble
./scripts/start_px4_mavproxy_bridge.sh
```

If PX4 is in the dead USB state, reconnect the Cube Black USB cable once while MAVProxy is already running. After that, reuse the same bridge for repeated commands and uploads:

```bash
./scripts/px4_cmd.sh 'ver all' 20
./scripts/upload_params.sh --reversible-drive
./scripts/upload_params.sh --verify-only --reversible-drive
./scripts/start_px4_mavproxy_bridge.sh --status
./scripts/start_px4_mavproxy_bridge.sh --stop
```

To bypass the bridge for a single run and go back to direct USB:

```bash
PX4_USE_MAVPROXY=0 ./scripts/px4_cmd.sh 'ver all' 20
./scripts/upload_params.sh --direct-usb --reversible-drive
```

See [PX4 Overview](docs/architecture/px4_overview.md) for the rationale and operator notes.

## System Monitor & Control Center

Real-time health monitoring and remote command interface for the rover. The `rover_monitor` ROS 2 package runs on the Jetson, aggregating camera/PX4/platform metrics. The Control Center runs on a host machine providing a web dashboard, InfluxDB storage, and bidirectional MQTT command gateway.

See [System Monitor Architecture](docs/architecture/system_monitor.md) and [Control Center Architecture](docs/architecture/control_center.md) for full details.

### Quick Start — Jetson (rover)

```bash
# Build rover_monitor (first time or after changes)
./scripts/start_ros2_nodes.sh --build-only=rover_monitor

# Launch everything: XRCE agent + cameras + SLAM + PX4 + monitor
# (Docker container is auto-started if not already running)
./scripts/start_jetson_session.sh --depth-camera=d435i --cuvslam-odom --with-telemetry

# With VINS-Fusion odometry instead of cuVSLAM:
./scripts/start_jetson_session.sh --depth-camera=d435i --vins-odom --with-telemetry

# With T265 tracking camera (no VIO):
./scripts/start_jetson_session.sh --depth-camera=d435i --t265 --with-telemetry

# With T265 as odometry source (T265 internal VIO):
./scripts/start_jetson_session.sh --depth-camera=d435i --t265-odom --with-telemetry

# With Nav2:
./scripts/start_jetson_session.sh --depth-camera=d435i --cuvslam-odom --nav2 --with-telemetry
```

### Hardware Bringup Shortcuts

```bash
# D435i + cuVSLAM odometry + RTAB-Map
./scripts/start_ros2_nodes.sh --hw --depth-camera=d435i --cuvslam-odom --rtabmap

# D435i + VINS-Fusion odometry + RTAB-Map
./scripts/start_ros2_nodes.sh --hw --depth-camera=d435i --vins-odom --rtabmap

# D435i + T265 tracking + RTAB-Map (T265 internal VIO)
./scripts/start_ros2_nodes.sh --hw --depth-camera=d435i --t265 --rtabmap

# T265 as the odometry source for EKF (no external VIO)
./scripts/start_ros2_nodes.sh --hw --depth-camera=d435i --t265-odom --rtabmap

# cuVSLAM odometry only (T265 fisheye, no depth camera)
./scripts/start_ros2_nodes.sh --hw --cuvslam-odom

# VINS-Fusion odometry only (T265 fisheye, no depth camera)
./scripts/start_ros2_nodes.sh --hw --vins-odom

# Probe the live VIO / RTAB-Map pipeline from the host
HZ_WINDOW=5 ECHO_TIMEOUT=3 ./scripts/debug_vio.sh
```

**Odometry source priority:** cuVSLAM > VINS-Fusion > T265 > RGB-D VO > ICP.
Both `--cuvslam-odom` and `--vins-odom` automatically enable the T265 fisheye
streams (required for stereo VIO). When no VIO flag is given, RTAB-Map uses its
built-in RGB-D visual odometry.

With `--t265-odom`, EKF and RTAB-Map subscribe to `/t265/odom_base`, while
`rgbd_odometry` stays independent on `/vo_odom` for debugging and comparison.
The relayed T265 base odometry publishes in the standard `odom ->
ackermann/base_link` frame chain.

### Quick Start — Host (base station)

```bash
# Launch Control Center stack (Mosquitto + InfluxDB + dashboard)
./scripts/start_host_session.sh

# Run e2e verification tests
./scripts/test_control_center.sh
```

- Dashboard: `http://localhost:8080`
- InfluxDB: `http://localhost:8086` (rover / rover-password)

### x86 Development (no hardware)

```bash
# Auto-detect mode (uses real cameras if connected, mocks otherwise):
./scripts/start_system_monitor_session.sh --with-telemetry

# Force mock mode with localhost broker:
./scripts/start_system_monitor_session.sh --mock --with-telemetry
```

### Stop

```bash
# Jetson:
./scripts/stop_all.sh --session=jetson

# Host:
docker compose -f control_center/docker-compose.yaml down
tmux kill-session -t host
```

> **Note:** Edit `src/rover_monitor/config/publisher.yaml` to set `broker_host` to the host machine IP before deploying to Jetson.

## Architecture Blueprints

The full system architecture (Gazebo ↔ RTAB-Map interfaces, topic contracts, and node graph) lives in the `docs/architecture` directory:

- [Architecture Overview](docs/architecture/overview.md)
- [Node Graph](docs/architecture/node_graph.md)
- [Interfaces Reference](docs/architecture/interfaces.md)

These documents stay current with the `robot_bringup` launch composition, ros_gz bridge topics, and RTAB-Map + Nav2 integrations—start there when wiring new hardware or extending the stack.
