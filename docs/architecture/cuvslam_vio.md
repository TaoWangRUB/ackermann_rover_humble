---
title: cuVSLAM Visual-Inertial Odometry
status: Draft
last_updated: 2026-04-12
doc_type: architecture
---

## Overview

cuVSLAM provides a stereo visual-inertial odometry path using the T265 fisheye
cameras and IMU. It runs as an alternative to the T265 built-in VIO,
VINS-Fusion, and RTAB-Map RGB-D odometry, selectable at launch time with
`use_cuvslam_odom:=true` or `./scripts/start_ros2_nodes.sh --cuvslam-odom`.

### Data Flow

```text
T265 Fisheye (848x800) + IMU
  -> realsense_camera_bringup or Gazebo T265 bridges
  -> /t265/fisheye1/image_raw, /t265/fisheye2/image_raw, /t265/imu
  -> [cuvslam_odom_node]
  -> /cuvslam/raw_odometry
  -> [odom_tf_relay]
  -> /cuvslam_odom (odom -> ackermann/base_link)
  -> EKF (odom0, IMU yaw fusion disabled for external VIO)
  -> /odometry/filtered -> RTAB-Map / Nav2 / PX4
```

### Key Files

| File | Purpose |
|------|---------|
| `src/cuvslam_bringup/src/cuvslam_odom_node.cpp` | Thin ROS 2 wrapper around `cuvslam::Odometry` |
| `src/cuvslam_bringup/launch/cuvslam.launch.py` | Launch + odom relay wiring |
| `src/cuvslam_bringup/config/t265_stereo_fisheye.yaml` | Wrapper config |
| `src/description_robot/launch/gazebo_bringup.launch.py` | Sim-side T265 fisheye/IMU ROS bridges |
| `scripts/build_cuvslam.sh` | Docker-aware operator build helper |
| `scripts/debug_vio.sh` | Runtime probe for T265, cuVSLAM, EKF, and RTAB-Map topics |
| `patches/` | (none — cuVSLAM submodule is used unmodified; Jetson arch patches applied at build time by sed) |

## Software Architecture

### Algorithm Overview

cuVSLAM is NVIDIA's stereo visual-inertial SLAM library (closed source,
vendored at `src/cuVSLAM` as a git submodule pinned to v15.0.0). It runs
feature tracking, stereo matching, and bundle adjustment on the GPU via
CUDA, exposing a C++ API through `cuvslam::Odometry`. The ROS 2 wrapper
(`cuvslam_odom_node`) is intentionally thin — it handles message conversion,
timestamp ordering, and IMU buffering, delegating all estimation to the
library.

### Node Lifecycle and Call Sequence

```text
                             ┌────────────────────┐
                             │ cuvslam_odom_node   │
                             │   (ROS 2 lifecycle) │
                             └────────┬───────────┘
                                      │
  1. SUBSCRIBE                        ▼
     /t265/fisheye{1,2}/camera_info ──► leftInfoCallback / rightInfoCallback
     (one-shot: store CameraInfo,       │
      call tryInitRig once both ready)  │
                                        ▼
  2. RIG INIT (tryInitRig)
     a. lookupExtrinsicsFromTf():
        TF buffer lookup for rig_frame → left_cam, right_cam, imu
        (timeout: rig_init_timeout_s = 10 s; fallback: identity + hardcoded baseline)
     b. buildCuvslamCamera() × 2:
        Extract fx, fy, cx, cy from K matrix
        Detect distortion model (equidistant/plumb_bob/rational_polynomial/pinhole)
        Set rig_from_camera pose from TF
     c. Configure cuvslam::ImuCalibration:
        rig_from_imu, noise densities, random walks, frequency
     d. Configure cuvslam::OdometryConfiguration:
        mode = Inertial (with IMU) or Multicamera (without)
        use_gpu = true, async_sba, use_motion_model
     e. cuvslam::WarmUpGPU()
     f. tracker_ = make_unique<cuvslam::Odometry>(rig, config)
                                        │
  3. SUBSCRIBE (after init)             ▼
     /t265/fisheye1/image_raw ──┐
     /t265/fisheye2/image_raw ──┤── message_filters::ApproximateTime
     /t265/imu ─────────────────┤      (queue=10, slop=0.01 s)
                                │
  4. IMU PATH (imuCallback)     ▼
     a. Validate timestamp > last_imu_received_ts
     b. Create cuvslam::ImuMeasurement {timestamp_ns, accel[3], gyro[3]}
     c. Append to pending_imu_ deque
                                │
  5. STEREO PATH (stereoCallback)
     a. Guard: drop if tracker_ not initialized
     b. Extract timestamps; warn if L/R delta > 1 ms
     c. Monotonic guard: drop if left_ts_ns <= last_track_ts_ns_
     d. flushImuMeasurements(left_ts_ns):
        for each pending IMU with ts > last_track_ts:
          tracker_->RegisterImuMeasurement(0, meas)
     e. cv_bridge → MONO8; build cuvslam::Image structs (CPU mem)
     f. tracker_->Track({left_image, right_image})
        → returns cuvslam::PoseEstimate { world_from_rig (optional) }
     g. last_track_ts_ns_ = left_ts_ns
     h. publishOdometry(world_from_rig, stamp)
                                │
  6. PUBLISH                    ▼
     /cuvslam/raw_odometry (nav_msgs/Odometry)
       frame_id: odom
       child_frame_id: ackermann/base_link (or left optical frame fallback)
       pose: position + quaternion from PoseEstimate
       covariance: 6×6 rotated from cuVSLAM order (rot,trans) to ROS order (pos,rot)
                                │
  7. RELAY (odom_tf_relay)      ▼
     /cuvslam_odom (nav_msgs/Odometry)
       Pose composition: T_WB = T_WC × T_CB (from cached TF)
       Origin latch: subtract first pose for (0,0,0) start
       Velocity: lever-arm corrected
       Covariance: rotated to base frame
       publish_tf = false (EKF owns odom→base_link TF)
```

### cuVSLAM C++ API Calls Used

| API Call | Where | Purpose |
|----------|-------|---------|
| `cuvslam::WarmUpGPU()` | tryInitRig | Pre-allocate GPU context |
| `cuvslam::Odometry(rig, cfg)` | tryInitRig | Create tracker with stereo rig and config |
| `tracker_->Track(ImageSet)` | stereoCallback | Process stereo pair, return `PoseEstimate` |
| `tracker_->RegisterImuMeasurement(idx, meas)` | flushImuMeasurements | Buffer IMU sample for next Track |
| `cuvslam::SetVerbosity(level)` | constructor | 0=quiet, 1=debug (from `debug` param) |

### Distortion Model Mapping

| ROS `distortion_model` | cuVSLAM `Distortion::Model` | Coefficients |
|-------------------------|------------------------------|-------------|
| `"equidistant"` (T265 fisheye) | `Fisheye` | 4 (k1-k4 Kannala-Brandt) |
| `"plumb_bob"` | `Brown` | 5 (k1, k2, p1, p2, k3) |
| `"rational_polynomial"` | `Polynomial` | 8 |
| `""` / `"none"` | `Pinhole` | 0 |

### Timestamp Ordering Contract

cuVSLAM throws an exception if stereo frame timestamps are not strictly
ascending. Two layers enforce this:

1. **realsense_camera_node** fisheye dedup (upstream fix): librealsense
   pipeline sync re-emits the most recent fisheye pair whenever a new
   pose/accel/gyro frame arrives. `last_fisheye{1,2}_frame_number_` members
   in the T265 frameset branch skip re-emissions by comparing
   `rs2::video_frame::get_frame_number()`.

2. **cuvslam_odom_node** monotonic guard: `stereoCallback` drops pairs
   where `left_ts_ns <= last_track_ts_ns_` with a throttled WARN log.
   Acts as a defensive fallback if upstream dedup misses an edge case.

### IMU Noise Model Defaults

Seeded from BMI055 (T265 internal IMU) datasheet values:

| Parameter | Value | Unit |
|-----------|-------|------|
| Gyroscope noise density | 1.7e-4 | rad/(s·√Hz) |
| Gyroscope random walk | 2.0e-5 | rad/(s²·√Hz) |
| Accelerometer noise density | 2.0e-3 | m/(s²·√Hz) |
| Accelerometer random walk | 3.0e-3 | m/(s³·√Hz) |
| IMU frequency | 200.0 | Hz |

### CMake Library Discovery

`src/cuvslam_bringup/CMakeLists.txt` searches for `libcuvslam.so` in priority
order:

1. `-DCUVSLAM_PREFIX=<path>` (CMake override)
2. `CUVSLAM_PREFIX` environment variable
3. `/docker_cache/cuvslam/*/install`
4. `/workspace/docker_cache/cuvslam/*/install`
5. `/usr/local` (system install)
6. `../cuVSLAM/build/bin` (vendored source build — the default path)

If none found, the package builds in metadata-only mode (launch files and
config installed, but no executables).

## Build

Use the Docker-aware helper from the host:

```bash
./scripts/build_cuvslam.sh
```

This script:

1. enters the `ackermann_slam` container if needed
2. on Jetson (aarch64), applies three build workarounds automatically (see below)
3. builds vendored `src/cuVSLAM` with `gcc-11` / `g++-11`
4. builds `cuvslam_bringup`, `robot_bringup`, `rtabmap_bringup`, and related packages
5. runs the cuVSLAM smoke test

### Jetson aarch64 Build Workarounds

`scripts/build_cuvslam.sh` detects aarch64 and applies three workarounds that
are needed because JetPack 5.x ships CUDA 11.4, while the cuVSLAM upstream
source targets CUDA 12+:

1. **`-arch=all` -> `-arch=sm_XX`** — CUDA 11.4 does not support
   `-arch=all` (added in 11.5). The script auto-detects the Jetson SoC from
   `/proc/device-tree/compatible` and patches the cuVSLAM cuda_kernels
   CMakeLists to target `sm_72` (Xavier NX / AGX Xavier) or `sm_87` (Orin).

2. **Empty `librt.a` stub** — glibc 2.34+ merged librt into libc, leaving
   an empty 8-byte archive at `/usr/lib/aarch64-linux-gnu/librt.a`. nvlink
   11.4 cannot open a zero-member archive. The script replaces it with a
   valid dummy archive containing a single stub symbol.

3. **`liblmdb-dev`** — required by cuVSLAM for the landmark database but
   not installed by default in the container image. The script installs it
   via `apt-get` if `/usr/include/lmdb.h` is missing.

## Launch

Hardware with T265-only odometry:

```bash
./scripts/start_ros2_nodes.sh --hw --cuvslam-odom
```

Hardware with D435i + RTAB-Map:

```bash
./scripts/start_ros2_nodes.sh --hw --depth-camera=d435i --cuvslam-odom --rtabmap
```

Gazebo with simulated T265 + RTAB-Map:

```bash
./scripts/start_ros2_nodes.sh --cuvslam-odom --rtabmap
```

When `use_cuvslam_odom:=true`:

- `robot_bringup` auto-enables the T265 path
- T265 fisheye streams are enabled alongside the IMU
- `rtabmap_bringup` prefers `/cuvslam_odom` over VINS, T265 built-in odom, VO, and ICP
- EKF IMU yaw-rate fusion is disabled to avoid double-counting heading

## Debugging

While the stack is running:

```bash
./scripts/debug_vio.sh
```

The probe reports:

- `/t265/fisheye1/image_raw`, `/t265/fisheye2/image_raw`, `/t265/imu`
- `/cuvslam/raw_odometry`, `/cuvslam_odom`
- `/vo_odom`, `/odometry/filtered`, `/map`, and related comparison topics

## VIO Performance Comparison

The numbers below were captured on the same physical Intel RealSense T265 used
in [vins_fusion_vio.md](vins_fusion_vio.md). The x86_64 and Jetson tables are
useful side-by-side, but the stationary drift windows are not identical:
x86_64 used a longer 30 s bench run, while the Jetson Xavier aarch64 path was
captured from a warmed 10 s stationary window during standalone validation.

Config: stock `src/cuvslam_bringup/config/t265_stereo_fisheye.yaml` (no
parameter tuning), built-in stereo + IMU integration. x86_64 used cuVSLAM
15.0.0+efdfbe5 against CUDA 12.8 on an NVIDIA RTX A2000 Laptop GPU. Jetson
used the same source tree rebuilt inside `jazzy_slam_aarch64` against CUDA
11.4 (`Build cuda_11.4.r11.4/compiler.31964100_0`).

### Test Platforms

| Platform | Notes |
|----------|-------|
| x86_64 + RTX A2000 Laptop GPU | Docker desktop path, CUDA 12.8, same host used for the VINS-Fusion comparison |
| Jetson Xavier aarch64 | Docker `jazzy_slam_aarch64`, CUDA 11.4, 6 CPU cores, ~6.8 GB usable RAM reported by `tegrastats` |

### x86_64 Topic Rates (Stationary)

| Topic | Measured Rate |
|------|---------------|
| `/t265/fisheye1/image_raw` | 27.5 Hz |
| `/t265/fisheye2/image_raw` | 27.9 Hz |
| `/t265/imu` | 200.053 Hz |
| `/t265/odom` (T265 built-in) | 200.031 Hz |
| `/cuvslam/raw_odometry` | 23.8 Hz |
| `/cuvslam_odom` (relay) | 28.0 Hz (steady state) |

### Jetson Xavier Topic Rates (Stationary)

| Topic | Measured Rate |
|------|---------------|
| `/t265/fisheye1/image_raw` | 29.7 Hz |
| `/t265/fisheye2/image_raw` | 29.2 Hz |
| `/t265/imu` | 200.1 Hz |
| `/t265/odom_base` (T265 built-in) | 200.1 Hz |
| `/cuvslam/raw_odometry` | 29.1 Hz |
| `/cuvslam_odom` (relay) | 29.1 Hz |

### x86_64 Stationary Drift (30 s, origin-subtracted, 744 samples)

| Axis | Final displacement | Span (max-min) |
|------|--------------------|----------------|
| X | -0.0003 m | 0.0038 m |
| Y | -0.0018 m | 0.0083 m |
| Z | -0.0004 m | 0.0021 m |
| **Total** | **0.0018 m** | — |

### Jetson Xavier Stationary Drift (Warm 10 s window, 297 samples)

| Axis | Final displacement | Span (max-min) |
|------|--------------------|----------------|
| X | +0.0028 m | 0.0423 m |
| Y | -0.0021 m | 0.0396 m |
| Z | +0.0024 m | 0.0557 m |
| **Total** | **0.0043 m** | — |

### x86_64 Resource Usage

| Metric | cuVSLAM |
|--------|---------|
| `cuvslam_odom_node` CPU | 15 – 23 % of one host core (avg ~19 %) |
| `cuvslam_odom_node` RAM (RSS) | ~441 MB |
| `cuvslam_odom_node` GPU VRAM | 364 MiB |
| `realsense_camera_node` CPU | 10 – 22 % of one host core (avg ~18 %) |
| `realsense_camera_node` RAM (RSS) | ~51 MB |
| GPU utilization (8 s window, stationary) | 0 – 1 % |

### Jetson Xavier Resource Usage

| Metric | cuVSLAM |
|--------|---------|
| `cuvslam_odom_node` CPU | ~32.5 % of one Jetson core |
| `cuvslam_odom_node` RAM (RSS) | ~851.5 MB |
| `realsense_camera_node` CPU | ~44.1 % of one Jetson core |
| `realsense_camera_node` RAM (RSS) | ~63.9 MB |
| System RAM (`tegrastats`, 10 s window) | ~2.64 – 2.71 GB / 6.85 GB |
| GPU utilization (`GR3D_FREQ`, 10 s window) | mostly 0 – 22 %, one brief 87 % spike |

### Side-by-Side: cuVSLAM on x86_64 vs Jetson Xavier

| Metric | x86_64 desktop | Jetson Xavier |
|--------|----------------|---------------|
| T265 fisheye rate | 27.5 / 27.9 Hz | 29.7 / 29.2 Hz |
| T265 built-in odom | 200.031 Hz | 200.1 Hz |
| cuVSLAM raw odom | 23.8 Hz | 29.1 Hz |
| cuVSLAM relayed odom | 28.0 Hz | 29.1 Hz |
| Stationary drift window | 30 s | 10 s warm window |
| Stationary net drift | **0.0018 m** | 0.0043 m |
| Stationary drift span | 3.8 x 8.3 x 2.1 mm | 42.3 x 39.6 x 55.7 mm |
| `cuvslam_odom_node` CPU | ~19 % of one core | ~32.5 % of one core |
| `cuvslam_odom_node` RAM (RSS) | ~441 MB | ~851.5 MB |
| `realsense_camera_node` CPU | ~18 % of one core | ~44.1 % of one core |
| GPU utilization | 0 – 1 % | mostly 0 – 22 %, brief 87 % spike |
| Platform takeaway | lowest drift, lowest host cost | full-rate tracking works, but pose is noisier and memory use is higher |

### Side-by-Side vs VINS-Fusion and T265 Built-in (x86_64)

Numbers for VINS-Fusion (tuned) and T265 built-in below are quoted directly
from [vins_fusion_vio.md](vins_fusion_vio.md) so the comparison is
apples-to-apples on the same host and sensor.

| Metric | cuVSLAM (stock) | VINS-Fusion (tuned) | T265 built-in |
|--------|-----------------|---------------------|----------------|
| Raw odom rate | 23.8 Hz | 26.0 Hz | 198 – 200 Hz |
| Relayed odom rate | 28.0 Hz | 26.0 Hz | 200.0 Hz |
| Stationary drift over 30 s (total) | **0.0018 m** | 0.0100 m | 0.0024 m |
| VIO-node CPU (single core) | **~19 %** | 50 – 73 % | — (runs on T265 VPU) |
| VIO-node RAM (RSS) | **~441 MB** | ~660 MB | — |
| GPU utilization (stationary) | **0 – 1 %** | 10 – 11 % | 0 % |

### Interpretation

- **Drift**: cuVSLAM stock is ~5.5× better than VINS-Fusion tuned on the same
  rig and within 0.6 mm of the T265's built-in VIO, making it the best
  host-side option for stationary startup.
- **Jetson vs x86_64**: the Xavier path now builds and runs cleanly at the full
  T265 shutter rate, but it is not yet performance-equivalent to x86_64.
  Net drift over the 10 s warm window stayed low (4.3 mm), yet the pose
  wandered inside a much larger 4-6 cm box than on the x86 desktop bench.
- **CPU**: cuVSLAM uses roughly 1/3 of the CPU that VINS-Fusion does in its
  tuned configuration — the heavy lifting is offloaded to CUDA.
- **Jetson resource cost**: on Xavier, cuVSLAM remains workable but is much
  less memory-efficient than on x86_64 (~852 MB RSS vs ~441 MB) and drives the
  camera node harder as well (~44 % CPU vs ~18 %).
- **GPU**: Stationary scenes barely touch the GPU (0 – 1 %), versus 10 – 11 %
  for VINS-Fusion on the same hardware. VRAM footprint is 364 MiB so
  co-tenancy with RTAB-Map or Nav2 GPU workloads is comfortable on a
  4 GB / 8 GB class GPU.
- **Jetson GPU behavior**: Xavier mostly stayed in the 0 – 22 % `GR3D_FREQ`
  range, but occasional spikes still appear under a stationary camera. That
  suggests the Jetson path is not throughput-limited, but likely needs tuning
  around noise, synchronization, or power/perf behavior.
- **Rate**: Raw tracking rate (23.8 Hz) is slightly below VINS tuned (26 Hz);
  this is the shutter rate minus drops from the fisheye dedup path and is well
  above the 10 Hz needed by Nav2/EKF downstream. The T265 built-in VIO still
  dominates on pure rate because it runs on the Movidius VPU inside the
  camera.
- **When to prefer cuVSLAM**: it is the strongest host-side VIO on x86_64 —
  lowest drift, lowest CPU, lowest GPU, and lower RAM than VINS-Fusion. Choose
  it whenever the pipeline benefits from a stable, low-overhead host-side VIO
  (e.g. for later map-fusion, loop closure, or when the T265 built-in VIO is
  unreliable due to lighting, vibration, or firmware issues).
- **When T265 built-in still wins**: pure rate (~200 Hz) and zero host CPU
  cost. If the rover is running tight PX4 offboard control loops that want
  the highest-rate pose stream and do not need custom calibration or host-side
  reprocessing, the T265 built-in path remains the default.

## Current Status

Both platforms are integrated, measured, and verified:

- **x86_64**: fully validated against VINS-Fusion and T265 built-in VIO
  under stationary conditions. Lowest drift (1.8 mm / 30 s), lowest CPU
  (~19%), lowest GPU (0-1%) of all host-side VIO options.
- **Jetson Xavier (aarch64)**: source build, phase-0 smoke test, and
  standalone T265 + cuVSLAM bringup all pass inside `jazzy_slam_aarch64`.
  Three build workarounds (CUDA arch flag, librt.a stub, liblmdb-dev) are
  now codified in `scripts/build_cuvslam.sh` so the Jetson path is
  reproducible.

The remaining caveat is quality, not basic functionality: Xavier currently
tracks at the expected rate (~29 Hz), but its stationary pose wanders inside
a 4-6 cm box versus sub-centimeter on x86. That makes the Jetson path ready
for continued tuning and comparison work, but not yet performance-equivalent
to the x86 desktop result.
