---
name: cuVSLAM Visual-Inertial Odometry
description: Build, launch, and debug the cuVSLAM GPU-accelerated stereo fisheye VIO path in the ackermann_rover stack.
---

# cuVSLAM Visual-Inertial Odometry Skill

cuVSLAM provides a GPU-accelerated stereo fisheye + IMU visual-inertial odometry path using the Intel T265 camera. It is the **highest-priority** external VIO source in this repo, ahead of VINS-Fusion, T265 built-in odometry, RGB-D VO, and ICP.

## 1. Architecture / Data Flow

```
T265 Fisheye (848x800 @30Hz) + IMU (@200Hz)
  → realsense_camera_bringup (hw) or gazebo bridges (sim)
  → /t265/fisheye1/image_raw, /t265/fisheye2/image_raw
  → /t265/fisheye1/camera_info, /t265/fisheye2/camera_info, /t265/imu
  → [cuvslam_odom_node]         (CameraInfo-driven rig init, stereo VIO, IMU)
  → /cuvslam/raw_odometry
  → [odom_tf_relay]             (frame adaptation)
  → /cuvslam_odom               (frame_id=odom, child_frame_id=ackermann/base_link)
  → EKF (odom0; IMU yaw fusion disabled to avoid double-counting)
  → /odometry/filtered → RTAB-Map / Nav2 / PX4
```

**Odometry priority:** cuVSLAM > VINS-Fusion > T265 built-in > RGB-D VO > ICP

## 2. Key Files

| File | Purpose |
|------|---------|
| `src/cuvslam_bringup/src/cuvslam_odom_node.cpp` | ROS 2 wrapper around `cuvslam::Odometry` (cuvslam2.h) |
| `src/cuvslam_bringup/launch/cuvslam.launch.py` | Launch: cuvslam_odom_node + odom_tf_relay |
| `src/cuvslam_bringup/config/t265_stereo_fisheye.yaml` | Wrapper config (topics, frames, sync policy) |
| `src/cuvslam_bringup/test/smoke_test_cuvslam.cpp` | Links against libcuvslam.so, confirms it loads |
| `src/cuvslam_bringup/test/smoke_test_cuvslam.sh` | Runs the smoke test from shell |
| `src/cuVSLAM/` | Vendored cuVSLAM source tree (pinned submodule ref) |
| `src/cuVSLAM/build/bin/libcuvslam.so` | Built library (produced by install_cuvslam_deps.sh) |
| `docker/install_cuvslam_deps.sh` | Architecture-aware cuVSLAM source build script |
| `scripts/build_cuvslam.sh` | Docker-aware operator build helper (run from host) |
| `scripts/debug_vio.sh` | Runtime probe for T265, cuVSLAM, EKF, RTAB-Map topics |
| `docs/architecture/cuvslam_vio.md` | Architecture reference for this VIO path |

## 3. Build (x86_64)

cuVSLAM must be built from source before the `cuvslam_bringup` package can link against it.

**From the host (recommended):**
```bash
./scripts/build_cuvslam.sh
```

This script:
1. Enters the `ackermann_slam` Docker container.
2. Builds `src/cuVSLAM` with `gcc-11 / g++-11` (compatible with both x86 CUDA 12 and Jetson CUDA 11.4).
3. Runs `colcon build --packages-select cuvslam_bringup robot_bringup rtabmap_bringup ...`.
4. Runs the smoke test (`src/cuvslam_bringup/test/smoke_test_cuvslam.sh`) to confirm `libcuvslam.so` loads.

**Manually (inside container):**
```bash
bash docker/install_cuvslam_deps.sh
colcon build --symlink-install --packages-select cuvslam_bringup
```

> **Jetson Xavier:** Jetson enablement is phase 2. CUDA 11.4 + GCC 11 + shared CUDA runtime + glibc compat header applies. Use `install_cuvslam_deps.sh` with `ARCH=aarch64`.

## 4. Launch

**Hardware — T265 only with cuVSLAM:**
```bash
./scripts/start_ros2_nodes.sh --hw --cuvslam-odom
```

**Hardware — D435i depth + cuVSLAM odom + RTAB-Map:**
```bash
./scripts/start_ros2_nodes.sh --hw --depth-camera=d435i --cuvslam-odom --rtabmap
```

**Simulation — Gazebo + cuVSLAM + RTAB-Map:**
```bash
./scripts/start_ros2_nodes.sh --cuvslam-odom --rtabmap
```

**Manual inside container:**
```bash
# Use cuvslam.launch.py directly (standalone):
ros2 launch cuvslam_bringup cuvslam.launch.py use_sim_time:=true

# Or top-level with argument:
ros2 launch robot_bringup robot_bringup.launch.py use_cuvslam_odom:=true rtabmap:=true nav2:=false rviz:=false
```

When `use_cuvslam_odom:=true`:
- `robot_bringup` **automatically enables** the T265 hardware path and fisheye streams.
- `rtabmap_bringup` selects `/cuvslam_odom` as odom0 for both RTAB-Map and the EKF.
- EKF IMU yaw-rate fusion is **disabled** (`imu0_config` all-false) to avoid double-counting heading.
- Non-selected odom topics (VINS, RTAB-Map VO/ICP, T265 raw) continue publishing for debug/comparison.

## 5. Topic Contracts

| Topic | Message Type | frame_id | child_frame_id | Notes |
|-------|-------------|----------|----------------|-------|
| `/cuvslam/raw_odometry` | `nav_msgs/Odometry` | sensor-native | sensor-native | Raw cuVSLAM output |
| `/cuvslam_odom` | `nav_msgs/Odometry` | `odom` | `ackermann/base_link` | Relay-adapted, EKF/RTAB-Map input |
| `/odometry/filtered` | `nav_msgs/Odometry` | `odom` | `ackermann/base_link` | EKF output → Nav2/PX4 |

## 6. Debugging

While the stack is running:
```bash
./scripts/debug_vio.sh
```

The probe auto-detects and reports:
- `/t265/fisheye1/image_raw`, `/t265/fisheye2/image_raw`, `/t265/imu`
- `/cuvslam/raw_odometry`, `/cuvslam_odom`
- `/vo_odom`, `/odometry/filtered`, `/map` (for comparison)

**Manual checks:**
```bash
# Check cuVSLAM raw output rate
ros2 topic hz /cuvslam/raw_odometry

# Confirm adapted odom frame IDs
ros2 topic echo /cuvslam_odom --once

# Check EKF is subscribed to cuvslam_odom
ros2 node info /ekf_filter_node | grep cuvslam

# Confirm IMU yaw fusion is disabled when cuVSLAM is active
ros2 param get /ekf_filter_node imu0_config   # should be all-false

# Check RTAB-Map odom source
ros2 node info /rtabmap | grep cuvslam
```

## 7. Architecture-Aware Build Details

The cuVSLAM build uses the same CUDA toolchain pattern as VINS-Fusion:

| Platform | CUDA | GCC host compiler | Runtime linkage | Cache path |
|----------|------|------------------|-----------------|------------|
| x86_64 | Mounted from host at `/usr/local/cuda` (12.x) | System default (gcc-11 forced) | Static or shared | `/workspace/docker_cache/cuvslam/x86_64/current` |
| Jetson Xavier (aarch64) | CUDA 11.4 (device SDK) | `gcc-11 / g++-11` | **Shared** (`CUDA_RUNTIME_LIBRARY=Shared`) | `/workspace/docker_cache/cuvslam/aarch64/current` |

Jetson-specific flags also inject `realsense_camera_bringup/cuda_glibc_compat.h` during CUDA compilation and limit parallelism to `make -j2`.

## 8. Rollout Status

| Platform | Status |
|----------|--------|
| x86_64 (Docker) | ✅ Integrated and verified (CUDA 12.8, libcuvslam.so loads, `/cuvslam_odom` publishes at ~4 Hz in Gazebo sim) |
| Jetson Xavier (aarch64) | 🔲 Phase 2 — not yet started |

## 9. Rollback
Disable cuVSLAM with:
```bash
./scripts/start_ros2_nodes.sh --rtabmap   # (without --cuvslam-odom)
```
Downstream systems are unaffected because they consume `/odometry/filtered`, which falls back to the next priority source (VINS / T265 / RGB-D VO).
