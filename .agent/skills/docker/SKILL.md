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
If the container `ackermann_slam` is listed as `Up`, you can attach to it.

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

## 4. Hardware and X11 Forwarding
The compose file maps the `/workspace` folder directly to your host directory. Any edits you make on the host are instantly reflected inside the container.

- **GUI Applications (Gazebo/RViz)**: 
  The container uses X11 forwarding. If GUI applications fail to open with display errors, the host needs to allow local root access to XServer:
  ```bash
  xhost +local:root
  ```
- **GPU Acceleration**:
  The compose uses the NVIDIA runtime if available. If `nvidia-smi` fails inside the container, verify that the host has the NVIDIA Container Toolkit installed.
