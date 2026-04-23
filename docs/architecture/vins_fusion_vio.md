---
title: VINS-Fusion Visual-Inertial Odometry
status: Draft
last_updated: 2026-04-12
doc_type: architecture
---

## Overview

VINS-Fusion provides a stereo visual-inertial odometry (VIO) path using the
T265 fisheye cameras and IMU.  It runs as an alternative to the T265 built-in
VIO and RTAB-Map visual odometry, selectable at launch time with
`use_vins_odom:=true`.

### Data Flow

```
T265 Fisheye (848x800 @30 Hz) + IMU (@200 Hz)
  -> realsense_camera_node (fisheye auto-enabled when use_vins_odom:=true)
  -> /t265/fisheye1/image_raw, /t265/fisheye2/image_raw, /t265/imu
  -> [vins_node] (feature tracking via OpenCV, optional CUDA acceleration)
  -> /vins/raw_odometry (t265_pose_frame)
  -> [odom_tf_relay] (frame adaptation: t265_pose_frame -> ackermann/base_link)
  -> /vins_odom (odom -> ackermann/base_link)
  -> EKF (odom0, IMU yaw fusion disabled for external VIO)
  -> /odometry/filtered -> RTAB-Map / Nav2 / PX4
```

### Key Files

| File | Purpose |
|------|---------|
| `src/vins_fusion_bringup/launch/vins_fusion.launch.py` | Launch + relay wiring |
| `src/vins_fusion_bringup/config/t265_stereo_fisheye_imu.yaml` | Estimator config |
| `src/vins_fusion_bringup/config/left.yaml`, `right.yaml` | Kannala-Brandt fisheye calibration |
| `patches/vins-fusion-ros2-jazzy.patch` | ROS 2 Jazzy compatibility patch |
| `docker/install_vins_gpu_deps.sh` | CUDA OpenCV builder (per-arch cached) |
| `scripts/build_vins_gpu.sh` | Two-phase colcon build (VINS core + bringup) |

## Software Architecture

### Algorithm Overview

VINS-Fusion is an open-source tightly-coupled stereo visual-inertial
state estimator (HKUST). It fuses stereo image features with IMU
pre-integration in a sliding-window Ceres optimization. The rover
integration uses the stereo-inertial mode with the T265's Kannala-Brandt
fisheye cameras and BMI055 IMU.

The code is vendored as a git submodule at `src/VINS-Fusion` with a local
patch (`patches/vins-fusion-ros2-jazzy.patch`) applied for ROS 2 Jazzy
compatibility and fisheye numerical fixes.

### Node Lifecycle and Call Sequence

```text
                             ┌───────────────────┐
                             │  vins_node         │
                             │  (rosNodeTest.cpp) │
                             └───────┬───────────┘
                                     │
  1. INIT
     a. Parse config YAML from command-line argument
     b. readParameters(config_file):
        Load camera models (Kannala-Brandt), extrinsics, IMU noise, solver params
     c. estimator.setParameter():
        Initialize camera calibration, set IMU noise model
     d. registerPub(node):
        Create publishers: odometry, margin_cloud, keyframe_pose/point, extrinsic
                                     │
  2. SUBSCRIBE                       ▼
     /t265/imu ──────────────────► imu_callback
     /t265/fisheye1/image_raw ──┐
     /t265/fisheye2/image_raw ──┤── message_filters::ApproximateTime(queue=10)
                                │      ──► stereo_img_callback
                                │
  3. IMU PATH (imu_callback)
     a. Extract timestamp, accel[3], gyro[3] from sensor_msgs/Imu
     b. estimator.inputIMU(t, acc, gyr):
        - Locks mBuf, pushes to accBuf/gyrBuf
        - If solver_flag == NON_LINEAR: fastPredictIMU() for pose prediction
                                     │
  4. IMAGE PATH (stereo_img_callback + sync_process thread)
     a. stereo_img_callback: push synced pair to img0_buf/img1_buf (mutex)
     b. sync_process (dedicated thread, 2ms poll):
        - Pop oldest pair from buffers
        - Drop excess buffered frames to keep real-time
        - estimator.inputImage(time, img0, img1)
                                     │
  5. ESTIMATOR PIPELINE (processMeasurements)
     a. FEATURE TRACKING (feature_tracker.cpp):
        - Detect GoodFeaturesToTrack (Shi-Tomasi corners)
        - Track via calcOpticalFlowPyrLK (KLT sparse optical flow)
        - Optional GPU acceleration (use_gpu=1): cv::cuda::GoodFeaturesToTrackDetector,
          cv::cuda::SparsePyrLKOpticalFlow
        - Fundamental matrix RANSAC outlier rejection
        - Return feature observations per frame
     b. IMU PRE-INTEGRATION:
        - Integrate accel/gyro between consecutive image timestamps
        - Produces delta_p, delta_v, delta_q, Jacobians, covariance
        - Used as IMU factor in optimization
     c. INITIALIZATION (first ~10 frames):
        - initialStructure(): SfM with 5-point relative pose
        - visualInitialAlign(): gyroscope bias estimation, velocity recovery,
          gravity direction refinement, visual-inertial alignment
        - Sets solver_flag = NON_LINEAR once converged
     d. SLIDING-WINDOW OPTIMIZATION (optimization):
        - Ceres solver with factors:
          · IMU pre-integration factor (between consecutive keyframes)
          · Visual reprojection factors:
            - projectionTwoFrameOneCamFactor (same camera, two frames)
            - projectionTwoFrameTwoCamFactor (stereo cross, two frames)
            - projectionOneFrameTwoCamFactor (stereo cross, same frame)
          · Marginalization factor (Schur complement of removed states)
        - Solver budget: max_solver_time (25 ms), max_num_iterations (4)
     e. SLIDE WINDOW (slideWindow):
        - Remove oldest frame from sliding window
        - Marginalize old observations into prior
        - Update feature manager: remove out-of-window features
     f. PUBLISH:
        - nav_msgs/Odometry on /vins/raw_odometry (remapped from 'odometry')
          frame_id from config, child_frame_id = body_frame
                                     │
  6. RELAY (odom_tf_relay)           ��
     /vins_odom (nav_msgs/Odometry)
       Same relay pattern as cuVSLAM:
       Pose composition T_WB = T_WC × T_CB, origin latch, lever-arm velocity
       child_frame_id = ackermann/base_link, publish_tf = true
```

### Key Algorithm Parameters

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `max_cnt` | 80 (Jetson tuned) | Max features to track per frame |
| `min_dist` | 25 | Min pixel distance between features |
| `freq` | 20 | Feature tracking frequency cap (Hz) |
| `max_solver_time` | 0.025 | Ceres solver budget (seconds) |
| `max_num_iterations` | 4 | Ceres max iterations per frame |
| `keyframe_parallax` | 10.0 | Parallax threshold for keyframe insertion (pixels) |
| `use_gpu` | 1 | Enable CUDA feature tracking |
| `use_gpu_acc_flow` | 1 | GPU-accelerated optical flow |
| `estimate_extrinsic` | 0 | Fixed extrinsics (from hardware calibration) |
| `estimate_td` | 0 | Fixed time delay (T265 has hardware sync) |

### Submodule Patch System

VINS-Fusion is vendored at `src/VINS-Fusion` (git submodule). Local
modifications for ROS 2 Jazzy compatibility and Jetson tuning are
maintained as a single patch file:

```
patches/vins-fusion-ros2-jazzy.patch
```

**Patch contents** (10 files modified):

| File | Change | Reason |
|------|--------|--------|
| `camera_models/.../EquidistantCamera.cc` | Newton-Raphson fisheye distortion inversion | Faster, more stable than eigenvalue polynomial solver for Kannala-Brandt model |
| `vins/src/estimator/estimator.cpp` | Solver iteration / timing adjustments | Jetson real-time budget tuning |
| `vins/src/estimator/feature_manager.cpp` | Minor fix | Feature lifetime handling |
| `vins/src/estimator/parameters.h` | Header fix | ROS 2 compatibility |
| `vins/src/factor/projection*Factor.cpp` (3 files) | Numerical guard improvements | Prevent NaN in reprojection Jacobians with fisheye distortion |
| `vins/src/featureTracker/feature_tracker.cpp` | GPU flow parameter tuning | Better tracking on 848x800 fisheye images |
| `vins/src/utility/visualization.cpp` | Strip ROS 1 marker code | Unused in headless rover; reduces binary size |
| `vins/src/utility/visualization.h` | Remove unused declarations | Matches .cpp cleanup |

**Applying the patch** (done automatically by `scripts/build_vins_gpu.sh`):

```bash
cd src/VINS-Fusion
git checkout .              # reset to upstream
git apply ../../patches/vins-fusion-ros2-jazzy.patch
```

**Regenerating the patch** after editing VINS source:

```bash
git -C src/VINS-Fusion diff HEAD > patches/vins-fusion-ros2-jazzy.patch
```

### Shared Relay Pattern (odom_tf_relay)

Both cuVSLAM and VINS-Fusion use the same `odom_tf_relay` executable from
`realsense_camera_bringup` to adapt raw VIO output to the rover frame
contract:

1. **Pose composition**: `T_world_base = T_world_sensor × T_sensor_base`
   (static TF from sensor frame to `ackermann/base_link`, cached after
   first lookup)
2. **Origin latch**: first output pose is subtracted from all subsequent
   poses so odometry starts at (0,0,0) — matches EKF origin expectation
3. **Velocity lever-arm**: `v_base = R_sensor_base × (v_sensor + ω × r)`
4. **Covariance rotation**: 6×6 covariance rotated to base frame
5. **publish_tf = true**: `vins_odom_relay` owns the `odom → ackermann/base_link` TF

### EKF Integration and Odometry Source Selection

`rtabmap_slam.launch.py` resolves the odom topic with a priority cascade:

```
cuVSLAM → VINS-Fusion → T265 built-in → RGB-D VO → ICP
```

When any external VIO (cuVSLAM, VINS, T265) is selected:
- EKF `imu0_config` yaw rate is **disabled** (all-false) to prevent
  double-counting heading from both IMU and the VIO's internal IMU fusion
- EKF `odom0` points to the selected `/cuvslam_odom` or `/vins_odom` topic
- EKF `publish_tf` is false for external VIO modes; the selected odom relay
  owns the `odom -> ackermann/base_link` TF edge

---

## Docker GPU Setup

### Architecture-Qualified OpenCV CUDA Cache

The CUDA-enabled OpenCV build is cached per CPU architecture under
`docker_cache/opencv-cuda/<arch>/<version>/` so that x86_64 and aarch64
containers each maintain their own prebuilt binaries.  The cache is
bind-mounted into the container at `/workspace/docker_cache/`.

```
docker_cache/opencv-cuda/
  x86_64/
    4.10.0/           # CUDA 12.8, sm_86 (desktop GPU)
    current -> 4.10.0
  aarch64/
    4.5.5/            # CUDA 11.4, sm_72/87 (Jetson Xavier NX)
    current -> 4.5.5
```

The architecture is detected at build/launch time via `uname -m` (shell) or
`platform.machine()` (Python launch file).

### x86_64 Desktop (CUDA 12.x)

```bash
# Inside the Docker container (ackermann_slam):
# 1. Build CUDA OpenCV (one-time, cached)
./docker/install_vins_gpu_deps.sh

# 2. Build VINS packages against CUDA OpenCV
./scripts/build_vins_gpu.sh

# 3. Launch
ros2 launch robot_bringup robot_bringup.launch.py \
  use_gazebo:=false hw_enable_t265:=true use_vins_odom:=true \
  depth_camera:=none rtabmap:=false nav2:=false rviz:=false
```

Prerequisites:
- Host CUDA toolkit (12.x) mounted into container via `docker-compose.yml`
  (auto-detected by `scripts/lib/dc.sh`)
- NVIDIA GPU with compute capability >= 8.6

### Jetson Xavier NX (CUDA 11.4)

```bash
# Inside the Docker container (jazzy_slam_aarch64):
# 1. Build CUDA OpenCV 4.5.5 (one-time, ~45 min on Xavier NX)
OPENCV_VERSION=4.5.5 ./docker/install_vins_gpu_deps.sh

# 2. Build VINS packages
OPENCV_VERSION=4.5.5 ./scripts/build_vins_gpu.sh

# 3. Launch
ros2 launch robot_bringup robot_bringup.launch.py \
  use_gazebo:=false hw_enable_t265:=true use_vins_odom:=true \
  depth_camera:=none rtabmap:=false nav2:=false rviz:=false
```

Prerequisites:
- JetPack 5.x (L4T R35.4.1, CUDA 11.4)
- Host CUDA toolkit at `/usr/local/cuda-11.4` mounted into container

### Why OpenCV Versions Differ

OpenCV's generated `OpenCVConfig.cmake` enforces an `EXACT` CUDA version
match at CMake configure time.  A library built with CUDA 12.8 will reject
CUDA 11.4 and vice versa.  Additionally, the `.so` binaries are
architecture-specific (ELF x86-64 vs ARM aarch64) and cannot be cross-loaded.
This is why the cache must be both architecture-qualified and built with the
host's CUDA version.

### LD_LIBRARY_PATH Isolation

The CUDA OpenCV is not installed system-wide.  The launch file injects
`LD_LIBRARY_PATH=<opencv_prefix>/lib` only for the `vins_node` process,
avoiding conflicts with the system OpenCV (4.6.0 on Jazzy) used by RTAB-Map
and cv_bridge.

---

## Compilation Procedure

### Full Build (Fresh Setup)

```bash
# 1. Apply VINS-Fusion Jazzy compatibility patch
./scripts/apply_vins_fusion_patch.sh

# 2. Build CUDA OpenCV (skip if docker_cache/<arch>/current/ exists)
./docker/install_vins_gpu_deps.sh          # x86_64
OPENCV_VERSION=4.5.5 ./docker/install_vins_gpu_deps.sh  # Jetson

# 3. Build VINS packages with GPU support
./scripts/build_vins_gpu.sh
```

### Rebuild After Code Changes

```bash
# VINS source or config only (fast, ~1 min x86, ~10 min Jetson)
./scripts/build_vins_gpu.sh

# Force OpenCV rebuild (after CUDA update)
FORCE_REBUILD=1 ./docker/install_vins_gpu_deps.sh
```

### Build Verification

```bash
# Check binary links correct CUDA OpenCV
ldd install/vins/lib/vins/vins_node | grep opencv

# Expected: libopencv_cuda*.so from docker_cache/<arch>/current/lib/
# System OpenCV (4.6.0) from /lib/ is expected for cv_bridge deps
```

---

## VIO Performance Comparison

Unless noted otherwise, both platforms used the same tuned rover config from
`src/vins_fusion_bringup/config/t265_stereo_fisheye_imu.yaml`:

- `max_cnt: 80`
- `min_dist: 25`
- `freq: 20`
- `show_track: 0`
- `max_solver_time: 0.025`
- `max_num_iterations: 4`
- `use_gpu: 1`
- `use_gpu_acc_flow: 1`
- `use_gpu_ceres: 0`

The x86_64 measurements were taken from message header timestamps over an 8 s
window while the rover stayed stationary. This avoids the misleading
`ros2 topic hz` spikes seen with the dual-fisheye T265 streams.

### Test Platforms

| Platform | Notes |
|----------|-------|
| x86_64 + RTX A2000 Laptop GPU | Docker container exposed 16 logical CPU threads and an NVIDIA RTX A2000 Laptop GPU |
| Jetson Xavier NX | 6-core ARM Cortex-A57 @ 1.42 GHz, 384-core Volta GPU, 8 GB RAM |

### Side-By-Side Summary

| Metric | x86_64 tuned | Jetson tuned | Jetson stock |
|--------|--------------|--------------|--------------|
| T265 built-in odom | 198-200 Hz | 200 Hz | 200 Hz |
| VINS raw/adapted odom | 26 Hz | 16 Hz | 8.5 Hz |
| vins camera-pose output | 26 Hz | not separately sampled | not separately sampled |
| vins IMU propagate debug stream | 200 Hz before publisher cleanup | not sampled | not sampled |
| vins_node CPU | 50-73% of one host core | 126% | 176% |
| realsense_camera_node CPU | 10-17% | 54% | 55% |
| GPU utilization | 10-11% | 33-46% | 8-31% |
| vins_node RAM | ~660 MB | ~680 MB | ~810 MB |
| T265 built-in drift over 30 s | 0.0024 m | 0.004 m | 0.004 m |
| VINS drift over 30 s | 0.0100 m | 0.016 m | 0.073 m |

### x86_64 Measured Topic Rates

| Topic | Measured Rate |
|------|---------------|
| `/t265/fisheye1/image_raw` | 30.001 Hz |
| `/t265/fisheye2/image_raw` | 30.001 Hz |
| `/t265/odom` | 198.152 Hz |
| `/t265/odom_base` | 200.039 Hz |
| `/vins/raw_odometry` | 26.000 Hz |
| `/vins_odom` | 26.000 Hz |
| `/vins/camera_pose` | 26.000 Hz |
| `/vins/imu_propagate` | 200.039 Hz before publisher cleanup |

### Interpretation

- The T265 built-in VIO remains the highest-rate source on both platforms and
  stays near the device-native 200 Hz output.
- On Jetson Xavier NX, the tuned VINS profile is usable but still clearly
  compute-limited compared with the T265's built-in VPU pipeline.
- On x86_64, the same tuned VINS profile reaches 26 Hz with substantially more
  CPU and GPU headroom than Jetson, which makes loop-closure-driven workflows
  much more practical.
- Even on x86_64, the T265 built-in odometry still wins on both rate and
  stationary drift. VINS-Fusion remains the better choice when custom
  calibration, estimator internals, or later loop/global fusion stages matter.

### Tuned Config Parameters

| Parameter | Stock | Tuned | Effect |
|-----------|-------|-------|--------|
| `max_cnt` | 150 | 80 | Fewer features tracked per frame |
| `min_dist` | 30 | 25 | Tighter spacing, maintains coverage |
| `freq` | 20 | 20 | Target publish rate |
| `show_track` | 1 | 0 | Disables debug image rendering |
| `max_solver_time` | 0.04 s | 0.025 s | Tighter Ceres solver deadline |
| `max_num_iterations` | 8 | 4 | Fewer optimization iterations |
| `use_gpu` | 1 | 1 | CUDA feature tracking (marginal benefit on Xavier) |
| `use_gpu_acc_flow` | 1 | 1 | CUDA optical flow |
| `use_gpu_ceres` | 0 | 0 | CPU solver (more numerically stable) |

### Platform-Specific CPU / GPU Usage

| Metric | Stock | Tuned | T265 Only |
|--------|-------|-------|-----------|
| vins_node CPU | 176% | 126% | N/A |
| realsense_camera_node CPU | 55% | 54% | 55% |
| System CPU avg (6 cores) | 57% | 43% | ~20% |
| GPU (GR3D) | 8-31% | 33-46% | 0% |
| RAM (vins_node) | 810 MB | 680 MB | N/A |
| Total RAM | 2.9 GB | 2.8 GB | ~2.1 GB |

### Jetson Stationary Drift (Origin-Subtracted)

| Source | X (m) | Y (m) | Z (m) | Total (m) |
|--------|-------|-------|-------|-----------|
| T265 built-in | 0.000 | 0.003 | -0.002 | 0.004 |
| VINS (stock) | -0.058 | -0.031 | 0.031 | 0.073 |
| VINS (tuned) | -0.008 | -0.011 | 0.008 | 0.016 |

### Recommendation

For Jetson Xavier NX deployment, the T265 built-in VIO remains the preferred
odometry source due to 200 Hz rate, sub-5mm stationary drift, and near-zero
CPU overhead (processing runs on the T265's internal VPU). On x86_64,
VINS-Fusion is viable at 26 Hz with low enough GPU overhead to be practical,
but it still does not outperform the T265's built-in odometry on raw tracking
rate or stationary drift. VINS-Fusion is most useful when:

- The T265 built-in odometry is unreliable (poor lighting, high vibration)
- Custom camera calibration or extrinsics are needed
- Loop closure or map-aware relocalization is required (via loop_fusion)
- The platform has more CPU/GPU headroom (x86 desktop, Orin)
