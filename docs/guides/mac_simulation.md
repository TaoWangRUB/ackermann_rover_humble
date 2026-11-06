---
title: Gazebo Simulation on an Apple-Silicon Mac
status: Draft
owner: Tao Wang
last_updated: 2026-11-06
doc_type: guide
ros_distro: jazzy
---
# Gazebo Simulation on an Apple-Silicon Mac

How to run this repo's simulation (Gazebo sensors, ros2_control, RTAB-Map, Nav2, PX4 SITL) in Docker Desktop on an Apple-silicon Mac. Everything below was run on 2026-11-06 on a Mac with Docker Desktop (engine 29.8, 14 GB RAM, 6 CPUs), no X server, no NVIDIA GPU.

## What differs from Linux / Jetson

| Difference | Why | What the repo does |
|------------|-----|--------------------|
| No RViz, no Gazebo window | macOS has no X server | Gazebo runs server-only with software rendering (`gz sim -s -r --headless-rendering`). Camera, depth and lidar sensors still render. View data in the Foxglove app. |
| Container ports are not reachable with `--network host` | Docker Desktop's host network is inside its Linux VM, not the Mac (checked with curl) | The Mac override gives the container its own bridge network and publishes the Foxglove port. |
| No NVIDIA runtime, no CUDA | Apple silicon | Runtime `runc`; `/usr/local/cuda` is an empty read-only stub. `cuvslam_bringup` builds "metadata only" (warning, expected); `realsense_camera_bringup` builds a stub node. |
| `rosdep install` in the base entrypoint fails silently | Only root had a rosdep cache; the entrypoint runs as the container user | The Mac layer runs `rosdep update` as the user and installs from the package manifests. |
| Gazebo model downloads | Fuel cache would be lost on container recreation | `gazebo_cache/` on the host is mounted at `/home/<user>/.gz`; the warehouse world (about 9 MB) downloads once. |

Not available on the Mac: cuVSLAM, VINS GPU mode, real cameras. Available: Gazebo sensors, `ros2_control`, RTAB-Map, Nav2, PX4 SITL.

Files: [Dockerfile.mac](../../docker/Dockerfile.mac), [docker-compose.mac.yml](../../docker/docker-compose.mac.yml), [dc.sh](../../scripts/lib/dc.sh), [start_ros2_nodes.sh](../../scripts/start_ros2_nodes.sh), [start_foxglove_bridge.sh](../../scripts/start_foxglove_bridge.sh).

Note for Linux users: the base image's `rosdep install` failure shows up as the log line `rosdep install encountered errors`. Then `gz_ros2_control`, `behaviortree_cpp_v3`, `imu_transformer`, `irobot_create_nodes` and `turtlebot4_simulator` are never installed unless you add them by hand. [Dockerfile.mac](../../docker/Dockerfile.mac) shows the fix.

## Prerequisites

- Docker Desktop with at least 14 GB RAM (the base image build runs `make -j` for librealsense and runs out of memory below that) and 6 CPUs.
- [Foxglove app](https://foxglove.dev/download) (3.5 tested; the bridge speaks `foxglove.sdk.v1`).
- Shell exports `ARCH=aarch64`, `USERNAME`, `USER_UID`, `USER_GID`. [dc.sh](../../scripts/lib/dc.sh) derives them if missing.
- The PX4 fork checked out next to this repo (`PX4_DIR` in [.env](../../.env), default `../../PX4-Autopilot`, with submodules); it is mounted at `/px4`.
- Run the helper scripts with `bash`, not `zsh`: `dc.sh` uses bash arrays.

## How the compose files are selected

[dc.sh](../../scripts/lib/dc.sh) maps `arm64` to `aarch64` and, on Darwin, adds `-f docker/docker-compose.mac.yml` to every helper script automatically. A hand-written `docker compose` call must add it itself:

```bash
docker compose -f docker/docker-compose.yml -f docker/docker-compose.mac.yml <cmd> ackermann_slam
```

The examples below use `dcomp`, the `dc.sh` wrapper that does this for you.

## Steps

All commands run from the repo root in bash.

### 1. Build the image

```bash
# first time only: the Linux/Jetson base image, built WITHOUT the Mac override
# (the override's build target is the Mac layer only), about 25 min
export ARCH=aarch64 USERNAME=$(id -un) USER_UID=$(id -u) USER_GID=$(id -g)
docker compose -f docker/docker-compose.yml build ackermann_slam
# Mac layer on top (foxglove_bridge, rosdep cache, workspace rosdeps), a few minutes
bash -c 'source scripts/lib/dc.sh && dcomp build ackermann_slam'
```

The result is the image `ackermann_rover_aarch64_jazzy_mac`.

### 2. Start the container

```bash
bash -c 'source scripts/lib/dc.sh && dcomp up -d ackermann_slam'
docker logs jazzy_slam_aarch64 | grep -E 'ROS DDS mode|rosdeps'
```

Expected: `ROS DDS mode: local` and `All required rosdeps installed successfully`.

### 3. Patch the submodules (once)

```bash
bash scripts/apply_px4_ros2_patch.sh && bash scripts/apply_vins_fusion_patch.sh
```

### 4. Build the workspace

Three colcon stages in the same order as [build.yml](../../.github/workflows/build.yml). `px4_msgs` and `px4_ros2_cpp` must be in `install/` before the rest builds. Parallelism is capped so the container does not run out of memory.

```bash
docker exec -i jazzy_slam_aarch64 bash -s <<'EOF'
set -e
source /opt/ros/jazzy/setup.bash
cd /workspace
cat > /tmp/colcon_defaults.yaml <<'YAML'
build:
  parallel-workers: 3
YAML
export COLCON_DEFAULTS_FILE=/tmp/colcon_defaults.yaml
export MAKEFLAGS='-j6 -l7'
COMMON=(--symlink-install --packages-ignore-regex '^example_.*' --packages-ignore px4_ros2_py)

# stage 1: PX4 message definitions (about 4 min)
colcon build "${COMMON[@]}" --packages-select px4_msgs
source install/setup.bash

# stage 2: PX4 ROS 2 C++ interface (about 15 min on a loaded Mac)
colcon build "${COMMON[@]}" --packages-select px4_ros2_cpp
source install/setup.bash

# stage 3: the rover packages (about 5 min, 15 packages)
colcon build "${COMMON[@]}" --packages-up-to ackermann_control safety px4_bringup \
  robot_bringup ackermann_nav2_bringup rtabmap_bringup cuvslam_bringup \
  description_robot rover_monitor
EOF
```

To rebuild one package later: `bash scripts/start_ros2_nodes.sh --build-only=description_robot` (comma-separated list for several). A single-call build of the whole workspace has not been verified on the Mac; use the staged block above.

### 5. Run Gazebo headless

```bash
bash scripts/start_ros2_nodes.sh --headless
```

`--headless` starts server-only Gazebo and turns RViz off. Combine it with the other flags (`--rtabmap`, `--nav2`, `--px4`, ...). The controllers `joint_state_broadcaster` and `ackermann_steering_controller` become active after about 20 s.

Topic rates measured at a real-time factor of about 0.36 (Gazebo runs at roughly a third of real speed, so wall-clock rates are about a third of the simulated rates):

| Topic | Wall Hz | Sim Hz |
|-------|---------|--------|
| `/clock` | 304 | - |
| `/ackermann/odom` | 20 | 50 |
| `/joint_states` | 18.5 | 50 |
| `/l515/image` | 8 | 30 |
| `/l515/depth_image` | 8.7 | 30 |
| `/rplidar/scan` | 11.8 | 30 |
| `/l515/imu/raw` | 81 | 200 |

Do not trust single samples of `real_time_factor` in `/world/warehouse/stats`. Compare sim time with wall time over 20 s instead.

### 6. Visualise with Foxglove

```bash
bash scripts/start_foxglove_bridge.sh
```

In the Foxglove app: Open connection, Foxglove WebSocket, `ws://localhost:8766` (or `$FOXGLOVE_HOST_PORT`). The WebSocket handshake on that port was verified with curl (HTTP 101).

### 7. Drive the rover

Inside the container (sourced), `/cmd_vel` is a `TwistStamped` that is remapped to the `ackermann_steering_controller` reference:

```bash
docker exec -it jazzy_slam_aarch64 bash -c 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: ackermann/base_link}, twist: {linear: {x: 0.5}, angular: {z: 0.3}}}"'
```

Check: after 6 s `/ackermann/odom` moved from (0, 0) to (0.95, 0.25) m.

### 8. Stop

```bash
# Ctrl-C the launcher, then:
docker exec jazzy_slam_aarch64 pkill -INT -f 'ros2 launch'
docker exec jazzy_slam_aarch64 pkill -f 'ros2|gz sim|ruby|foxglove'
docker exec jazzy_slam_aarch64 pgrep -af 'gz sim|ros2 launch'   # must print nothing
```

- The container shares the Docker VM's pid namespace (`pid: host`), so these patterns also match other containers' processes of the same user.
- Call `pkill` directly through `docker exec`. Inside `bash -c '...'` the pattern matches its own shell.

## PX4 SITL

Verified on 2026-11-06 on this Mac: PX4 attached to the headless Gazebo model, EKF2 got a GNSS position, the external "Rover Speed Steering" mode registered (nav_state 23), the rover armed and drove 1.25 m on a left arc from `/cmd_vel`, and the mode's 2 s watchdog stopped it when the stream ended. Gazebo ran at a real-time factor of about 0.39.

`make px4_sitl_default` built in about 4 min inside the container on the mounted fork; the binary is `/px4/build/px4_sitl_default/bin/px4` (run it once from `/px4` via `docker exec -i jazzy_slam_aarch64 bash -c 'git config --global --add safe.directory "*" && cd /px4 && GZ_DISTRO=harmonic make px4_sitl_default'`).

Startup order (from [SKILL.md](../../.agent/skills/gazebo/SKILL.md) section 5 and [start_px4_sitl.sh](../../scripts/start_px4_sitl.sh)), each in its own terminal from the repo root:

| Step | Command | Note |
|------|---------|------|
| 1. Gazebo + mode node | `bash scripts/start_ros2_nodes.sh --px4 --headless --bridge=speed_steering` | `--px4` alone starts the **manual** mode node; add `--bridge=speed_steering` for the recommended Ackermann mode. The mode node starts 5 s after Gazebo and retries registration until PX4 is up (first attempt fails after about 25 s, harmless). |
| 2. DDS agent | `bash scripts/start_microxrce_agent.sh` | UDP 8888. Needs a terminal; from a script use `docker exec -i jazzy_slam_aarch64 bash -c 'export LD_LIBRARY_PATH=/opt/microxrce/lib:$LD_LIBRARY_PATH; exec /opt/microxrce/bin/MicroXRCEAgent udp4 -p 8888'`. |
| 3. PX4 SITL | `bash scripts/start_px4_sitl.sh` | Custom model, standalone: attaches to the running `ackermann` model in world `warehouse`, airframe 51000. Needs a terminal for the `pxh>` shell; see below for the daemon form. |
| 4. Wait for the mode | `bash scripts/wait_px4_modes_ready.sh 1 120` | The script expects 4 modes by default; pass `1` for a single mode node. |
| 5. Arm + activate | `bash scripts/activate_rover_manual.sh 23` | Sends VehicleCommand 100001 (mode 23) then arm. PX4 logs `Armed by external command`, the mode node `RoverSpeedSteeringMode activated`. |
| 6. Drive | `ros2 topic pub --times 80 -r 10 -w 1 /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: ackermann/base_link}, twist: {linear: {x: 0.5}, angular: {z: 0.3}}}"` (inside the container, sourced) | Measured: `/ackermann/odom` (0, 0) to (1.09, 0.62) m, yaw 51 deg; Gazebo `/model/ackermann/command/motor_speed` 9-10, `/model/ackermann/servo_0` 0.157 rad. After the stream stops the mode node logs `cmd_vel timeout (>2.00 s) - zeroing setpoint`. |
| 7. Disarm | `bash scripts/activate_rover_manual.sh --disarm` | |

PX4 SITL without a terminal (daemon form, same environment as `start_px4_sitl.sh`):

```bash
docker exec -i jazzy_slam_aarch64 bash -c 'cd /px4/build/px4_sitl_default/rootfs && rm -f lockfile && \
  export PX4_SYS_AUTOSTART=51000 PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_NAME=ackermann PX4_GZ_WORLD=warehouse PX4_SIMULATOR=gz GZ_DISTRO=harmonic && \
  exec /px4/build/px4_sitl_default/bin/px4 -d -s /px4/build/px4_sitl_default/etc/init.d-posix/rcS /px4/build/px4_sitl_default/etc' </dev/null
```

Status checks inside the container, from `/px4/build/px4_sitl_default/rootfs`: `/px4/build/px4_sitl_default/bin/px4-listener vehicle_status -n 1`, `px4-commander status` (lists `External Mode 1: nav_state: 23, name: Rover Speed Steering`), `px4-ekf2 status`. Do not use `-n 2` on rarely published topics such as `vehicle_command_ack`; it blocks until the second message.

Expected log lines: PX4 `[init] Standalone PX4 launch, waiting for Gazebo`, `[gz_bridge] world: warehouse, model: ackermann`, `[uxrce_dds_client] init UDP agent IP:127.0.0.1, port:8888`; `Preflight Fail: ekf2 missing data` only at boot. On the ROS side the PX4 topics are versioned in this fork: `/fmu/out/vehicle_status_v2`, `/fmu/out/vehicle_local_position_v1`.

Things to know (repo behaviour, not Mac-specific):

- `scripts/px4_check_modes.py` defaults to `/dev/ttyACM0` and gives interleaved, incomplete output on SITL even with `PX4_DEVICE=udpin:0.0.0.0:14550`; `px4-listener register_ext_component_reply` is the reliable check.
- `--px4` also starts the VO bridge, but nothing publishes `/odometry/filtered` in this launch set and `EKF2_EV_CTRL` is 0, so PX4 runs on GNSS and magnetometer from the Gazebo sensors. Harmless.
- Stop with the three background jobs killed, then `docker exec jazzy_slam_aarch64 pkill -INT -f 'ros2 launch'` and `docker exec jazzy_slam_aarch64 pkill -f 'px4|MicroXRCE|gz sim|ruby'`. PX4 leaves `parameters.bson`, `dataman`, `eeprom` and ULogs under `/px4/build/px4_sitl_default/rootfs` (git-ignored).

## Known limits and troubleshooting

| Symptom | Cause | Action |
|---------|-------|--------|
| No `odom` to `ackermann/base_link` transform in Gazebo-only mode | The controller has `enable_odom_tf: false`, and the Gazebo odometry plugin's TF goes to `/ackermann/tf` with frame `$odom`. Repo behaviour, not Mac-specific. | The `odom` frame appears with `--rtabmap` or `--t265-odom`. |
| Real-time factor about 0.36; slower under load | Software rendering on CPU | Close other heavy apps. Builds slow down too (`px4_ros2_cpp` took 15 min while the Mac swapped). |
| `Clock skew detected` during builds | virtiofs bind mount timestamps | Harmless. |
| Foxglove cannot connect | Host port 8765 is often taken by another local tool, hence the default 8766 | `lsof -nP -iTCP:8766 -sTCP:LISTEN`. Set `FOXGLOVE_HOST_PORT` before `dcomp up -d` to move it. |
| `start_foxglove_bridge.sh <port>` does not help | It changes only the container port, which the override does not publish | Keep 8765 inside the container. |
| Script fails with "the input device is not a TTY" | `start_px4_sitl.sh`, `start_microxrce_agent.sh`, `start_docker.sh` exec with a TTY | Run them from an interactive terminal. From scripts use `docker exec -i jazzy_slam_aarch64 ...`. |
| `cuvslam_bringup` warns "metadata only" | No CUDA | Expected. |
