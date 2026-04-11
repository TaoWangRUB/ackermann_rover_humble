---
title: cuVSLAM Visual-Inertial Odometry
status: Draft
last_updated: 2026-04-11
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

## Build

Use the Docker-aware helper from the host:

```bash
./scripts/build_cuvslam.sh
```

This script:

1. enters the `ackermann_slam` container if needed
2. builds vendored `src/cuVSLAM` with `gcc-11` / `g++-11`
3. builds `cuvslam_bringup`, `robot_bringup`, `rtabmap_bringup`, and related packages
4. runs the cuVSLAM smoke test

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

The numbers below were captured on the same x86_64 desktop and the same
physical Intel RealSense T265 used in
[vins_fusion_vio.md](vins_fusion_vio.md), so the two tables can be read
side-by-side. Rate, drift, and resource numbers came from an 8 s and 30 s
stationary window on the bench (rover powered, stationary on a flat desk).

Config: stock `src/cuvslam_bringup/config/t265_stereo_fisheye.yaml` (no
parameter tuning), built-in stereo + IMU integration, cuVSLAM 15.0.0+efdfbe5
against CUDA 12.8 on an NVIDIA RTX A2000 Laptop GPU.

### x86_64 Topic Rates (Stationary)

| Topic | Measured Rate |
|------|---------------|
| `/t265/fisheye1/image_raw` | 27.5 Hz |
| `/t265/fisheye2/image_raw` | 27.9 Hz |
| `/t265/imu` | 200.053 Hz |
| `/t265/odom` (T265 built-in) | 200.031 Hz |
| `/cuvslam/raw_odometry` | 23.8 Hz |
| `/cuvslam_odom` (relay) | 28.0 Hz (steady state) |

### x86_64 Stationary Drift (30 s, origin-subtracted, 744 samples)

| Axis | Final displacement | Span (max-min) |
|------|--------------------|----------------|
| X | -0.0003 m | 0.0038 m |
| Y | -0.0018 m | 0.0083 m |
| Z | -0.0004 m | 0.0021 m |
| **Total** | **0.0018 m** | — |

### x86_64 Resource Usage

| Metric | cuVSLAM |
|--------|---------|
| `cuvslam_odom_node` CPU | 15 – 23 % of one host core (avg ~19 %) |
| `cuvslam_odom_node` RAM (RSS) | ~441 MB |
| `cuvslam_odom_node` GPU VRAM | 364 MiB |
| `realsense_camera_node` CPU | 10 – 22 % of one host core (avg ~18 %) |
| `realsense_camera_node` RAM (RSS) | ~51 MB |
| GPU utilization (8 s window, stationary) | 0 – 1 % |

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
- **CPU**: cuVSLAM uses roughly 1/3 of the CPU that VINS-Fusion does in its
  tuned configuration — the heavy lifting is offloaded to CUDA.
- **GPU**: Stationary scenes barely touch the GPU (0 – 1 %), versus 10 – 11 %
  for VINS-Fusion on the same hardware. VRAM footprint is 364 MiB so
  co-tenancy with RTAB-Map or Nav2 GPU workloads is comfortable on a
  4 GB / 8 GB class GPU.
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

The x86_64 Docker path is integrated, measured, and verified against
VINS-Fusion and the T265 built-in VIO under stationary conditions (see
[`openspec/changes/add-cuvslam-vio/tasks.md`](../../openspec/changes/add-cuvslam-vio/tasks.md)
task 4.3). Jetson Xavier enablement (task 4.5) remains a later phase because
JetPack ships CUDA 11.4, which the cuVSLAM source build must be reconfigured
for — the same architecture-qualified approach used for the OpenCV CUDA cache
will apply to cuVSLAM itself.
