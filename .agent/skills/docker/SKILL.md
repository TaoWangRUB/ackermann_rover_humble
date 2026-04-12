---
name: Docker and Docker Compose
description: Instructions for managing the ROS 2 environment lifecycle using Docker and Compose.
---

# Docker and Docker Compose Skill

This project uses a containerized ROS 2 development environment. Instead of running commands directly on the host, all ROS 2 building, testing, and debugging MUST occur inside the provided Docker environment.

## 1. Checking Container Status
Before running any ROS 2 commands, always check if the development container is currently running:
```bash
docker compose -f docker/docker-compose.yml ps
```
If the container `ackermann_slam` (or `jazzy_slam_x86_64`) is listed as `Up`, you can attach to it.

## 2. Managing the Container Lifecycle
The `docker/docker-compose.yml` file is the source of truth for the stack.

- **Start the environment (detached)**:
  ```bash
  docker compose -f docker/docker-compose.yml up -d ackermann_slam
  ```
- **Stop the environment**:
  ```bash
  docker compose -f docker/docker-compose.yml down
  ```
- **Rebuild the image**:
  (Do this if you modify `docker/Dockerfile` or install new system dependencies in the Dockerfile).
  ```bash
  docker compose -f docker/docker-compose.yml build ackermann_slam
  ```
- **View setup logs**:
  (Useful if the container starts but immediately exits, meaning the entrypoint failed).
  ```bash
  docker compose -f docker/docker-compose.yml logs ackermann_slam
  ```
- **Restart a service** (e.g., after a code/sim conflict):
  ```bash
  docker compose -f docker/docker-compose.yml restart ackermann_slam
  ```

## 3. Executing Commands Inside the Container
To run ROS 2 commands (like `colcon build`, `ros2 launch`, etc.), you must execute them inside the running container.

- **Interactive Shell** (for general debugging):
  ```bash
  docker compose -f docker/docker-compose.yml exec ackermann_slam bash
  ```

- **Single Execution Command** (for scripts or CI):
  When running a single command, bash must be invoked to evaluate the setup script automatically:
  ```bash
  docker compose -f docker/docker-compose.yml exec ackermann_slam bash -c "colcon build --symlink-install"
  ```
  *Note*: Do NOT string multiple unrelated or long-blocking commands together in one `bash -c` unless properly handled with `&&` or scripts.

- **Kill all ROS/Gazebo/PX4 processes** (clean slate inside container):
  ```bash
  docker compose -f docker/docker-compose.yml exec ackermann_slam bash -c "pkill -9 -f 'ros2|rviz2|gz|ruby|px4|MicroXRCE'"
  ```

## 4. Host-Side Launcher Scripts (recommended)
The `scripts/` directory provides wrapper scripts that handle container entry and workspace sourcing automatically. Prefer these over manually exec-ing into the container.

```bash
# Build workspace
./scripts/start_ros2_nodes.sh --build-only

# Launch Gazebo only
./scripts/start_ros2_nodes.sh

# Launch Gazebo + RTAB-Map + Nav2
./scripts/start_ros2_nodes.sh --rtabmap --nav2

# Launch hardware stack (D435i + T265 + RTAB-Map)
./scripts/start_ros2_nodes.sh --hw --depth-camera=d435i --t265 --rtabmap

# Launch with cuVSLAM odometry
./scripts/start_ros2_nodes.sh --hw --depth-camera=d435i --cuvslam-odom --rtabmap

# Launch PX4 SITL stack
./scripts/start_ros2_nodes.sh --px4

# Debug VIO/camera pipeline (run while stack is up)
./scripts/debug_vio.sh

# Stop all processes (host session or jetson session)
./scripts/stop_all.sh --session=jetson
```

## 5. Hardware and X11 Forwarding
The compose file maps the `/workspace` folder directly to your host directory. Any edits you make on the host are instantly reflected inside the container.

- **GUI Applications (Gazebo/RViz)**:
  The container uses X11 forwarding. If GUI applications fail to open with display errors, the host needs to allow local root access to XServer:
  ```bash
  xhost +local:root
  ```
- **GPU Acceleration**:
  The compose uses the NVIDIA runtime if available. If `nvidia-smi` fails inside the container, verify that the host has the NVIDIA Container Toolkit installed.

## 6. CUDA Host Mount
The CUDA toolkit is NOT baked into the image. It is bind-mounted from the host at runtime:
```yaml
- ${CUDA_HOST_MOUNT-../docker/empty_cuda}:/usr/local/cuda:ro
```
- `CUDA_HOST_MOUNT` is auto-detected by `scripts/lib/dc.sh` on x86 or Jetson.
- If no CUDA toolkit is installed on the host, the fallback is `docker/empty_cuda` (empty directory).
- Inside the container, CUDA is available at `/usr/local/cuda`.
- `CUDA_HOME` and `CUDA_PATH` env vars are set to `/usr/local/cuda` inside the container.

## 7. cuVSLAM Source Build (x86_64 first)
cuVSLAM must be built from source inside the container before use:

```bash
# From the host (Docker-aware helper):
./scripts/build_cuvslam.sh
```

This script:
1. Enters the `ackermann_slam` container if needed.
2. Builds vendored `src/cuVSLAM` with `gcc-11` / `g++-11`.
3. Builds `cuvslam_bringup`, `robot_bringup`, `rtabmap_bringup`, and related packages.
4. Runs the cuVSLAM smoke test (`src/cuvslam_bringup/test/smoke_test_cuvslam.sh`).

Or manually inside the container:
```bash
bash docker/install_cuvslam_deps.sh
```

## 8. Important Volume Mounts (from docker-compose.yml)
| Mount | Purpose |
|-------|---------|
| `..:/workspace` | Project workspace (bind mount, editable from host) |
| `../gazebo_cache:/root/.gazebo` | Gazebo world/model cache |
| `${PX4_DIR-../../PX4-Autopilot}:/px4` | PX4 source tree |
| `${REALSENSE_ROS_DIR-../../realsense-ros}:/realsense-ros` | RealSense ROS driver |
| `/dev:/dev` | Hardware device passthrough |
| `${CUDA_HOST_MOUNT}:/usr/local/cuda:ro` | CUDA toolkit (host-mounted) |

## 9. Image Configuration (.env)
Default ROS/Gazebo/Ubuntu versions live in `.env` at the project root:
- `UBUNTU_VERSION=24.04`, `ROS_DISTRO=jazzy`, `GZ_DISTRO=harmonic`, `ROS_UBUNTU_CODENAME=noble`
- Override to target ROS Humble on 22.04: `UBUNTU_VERSION=22.04`, `ROS_UBUNTU_CODENAME=jammy`, `ROS_DISTRO=humble`
- Architecture is auto-detected: `export ARCH=$(uname -m)` → `x86_64` or `aarch64`

## 10. Entrypoint Behavior
On container start, `docker/entrypoint.sh`:
1. Runs `rosdep install` to install any missing ROS package dependencies.
2. Sources `/opt/ros/$ROS_DISTRO/setup.bash`.
3. Sources `/workspace/install/setup.bash` if the workspace has been built.
4. Configures FastDDS network profile (eth-only, for multi-machine ROS 2 via `config/fastdds_eth_only.xml`).
