# Project Context

## Purpose
Production-grade autonomous Ackermann-steered rover. Real-time SLAM, autonomous navigation, and PX4 flight controller integration on resource-constrained Jetson Xavier NX hardware.

## Tech Stack
- **ROS 2 Humble** (host) / **Jazzy** (Docker container)
- **Gazebo Harmonic** (simulation)
- **RTAB-Map** (visual SLAM)
- **Nav2** (autonomous navigation)
- **ros2_control** (hardware abstraction, ackermann_steering_controller)
- **PX4 Autopilot** via Micro XRCE-DDS (flight controller integration)
- **RealSense D4xx** (depth camera)
- **Cube Black** (flight controller hardware)
- **Docker** with NVIDIA runtime, host networking, privileged mode

## Project Conventions

### Code Style
- C++ for performance-critical nodes and PX4 custom modes (rclcpp, rclcpp_components)
- Python for bridge/converter nodes and utilities (rclpy)
- All packages use `ament_cmake` build type
- Python nodes: class inherits `Node`, parameter declarations in `__init__`, timers for periodic work, `threading.Lock` for shared state
- C++ components: `rclcpp_components` with `MutuallyExclusiveCallbackGroup` where isolation is needed

### Architecture Patterns
- **Coordinate conventions**: ROS/Gazebo use ENU (world) + FLU (body); PX4 uses NED (world) + FRD (body); conversion isolated in bridge nodes
- **TF tree**: `map` -> `odom` -> `ackermann/base_link`
- **Launch composition**: `robot_bringup.launch.py` is the top-level orchestrator with conditional includes; PX4 bringup launched separately after MicroXRCEAgent
- **Config-driven**: YAML parameter files under each package's `config/` directory
- **Platform-aware**: Supports both x86_64 (dev/sim) and aarch64 Jetson (field deployment)

### Package Structure
Each package follows:
```
src/<package_name>/
  CMakeLists.txt
  package.xml
  launch/          # .launch.py files
  config/          # .yaml parameter files
  src/             # C++ sources (if applicable)
  include/         # C++ headers (if applicable)
  scripts/         # Python nodes (if applicable)
  msg/             # Custom messages (if applicable)
```

### Scripts
Shell scripts in `scripts/` at project root for common operations (build, launch, PX4 SITL, XRCE agent, parameter upload). Docker workflows use `docker-compose -f docker/docker-compose.yml`.

## Important Constraints
- **Jetson Xavier NX**: 8 GB RAM, 6-core ARM, limited thermal headroom. Minimize process count and memory copies.
- **XRCE-DDS baud rate**: 921600 on `/dev/ttyTHS0`. Limit publish rates to avoid overloading the serial link.
- **Docker**: Container runs ROS Jazzy/Noble. Workspace mounted at `/workspace`, sourced as `/workspace/install/setup.bash`.
- **Privileged mode required**: sysfs, `/dev`, GPU access need privileged Docker or explicit device mappings.
- **PX4 bringup is separate**: Never launched from `robot_bringup.launch.py` — requires MicroXRCEAgent + PX4 SITL running first.

## Testing Strategy
- `colcon build --symlink-install` for builds
- `colcon test --event-handlers console_direct+` for unit tests
- Full simulation stack validation per AGENTS.md (SLAM, Nav2, controller tracking, PX4 offboard)

## Git Workflow
- Feature branches: `feature/<name>`
- PRs to `main`
- Commits: conventional style (`feat:`, `fix:`, `refactor:`, `docs:`)
