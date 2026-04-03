## Architecture / Data Flow

```
T265 Fisheye (848x800 @30Hz) + IMU
  → realsense_camera_node (fisheye auto-enabled when use_vins_odom:=true)
  → /t265/fisheye1/image_raw, /t265/fisheye2/image_raw, /t265/imu
  → [vins_node] (GPU feature tracking via CUDA OpenCV, LD_LIBRARY_PATH injected)
  → /vins/raw_odometry (t265_pose_frame)
  → [odom_tf_relay] (frame adaptation: t265_pose_frame → ackermann/base_link)
  → /vins_odom (odom → ackermann/base_link)
  → EKF (odom0, IMU yaw fusion disabled for external VIO)
  → /odometry/filtered → RTAB-Map / Nav2 / PX4
```

**Odometry priority**: VINS-Fusion > T265 built-in > RGB-D VO > ICP

**Key nodes**:
- `vins_node` (`vins` package) — stereo VIO estimator, subscribes fisheye + IMU
- `vins_odom_relay` (`realsense_camera_bringup/odom_tf_relay`) — adapts raw VINS output to rover frame contract
- `robot_localization` EKF — fuses `/vins_odom` as `odom0`, disables IMU yaw when external VIO active

**Key files**:
- `src/vins_fusion_bringup/launch/vins_fusion.launch.py` — VINS launch + relay wiring
- `src/vins_fusion_bringup/config/t265_stereo_fisheye_imu.yaml` — estimator config (topics, GPU flags, extrinsics)
- `src/vins_fusion_bringup/config/left.yaml`, `right.yaml` — Kannala-Brandt fisheye calibration
- `patches/vins-fusion-ros2-jazzy.patch` — ROS 2 Jazzy compatibility patch (applied at build time)
- `docker/install_vins_gpu_deps.sh` — CUDA OpenCV 4.10.0 builder with GPU arch auto-detection
- `scripts/build_vins_gpu.sh` — two-phase colcon build (VINS core + bringup packages)

## Key Design Decisions

- **GPU Ceres disabled** (`use_gpu_ceres: 0`) — CPU solver for numerical stability; GPU used only for feature tracking and optical flow
- **CUDA OpenCV isolated** via `LD_LIBRARY_PATH` injection in launch file — avoids conflicts with system OpenCV linked by RTAB-Map and other apt packages
- **Loop closure disabled** — real-time performance on Jetson; loop_fusion node not launched
- **Configurable frame IDs** — patch adds `ODOM_FRAME`/`BODY_FRAME` globals read from YAML config, replacing all hardcoded frame strings in VINS-Fusion source
- **Patch-based fork management** — changes live in `patches/vins-fusion-ros2-jazzy.patch`, applied at build time via `scripts/apply_vins_fusion_patch.sh`; submodule tracks upstream clean ref, no fork push required
- **Thread safety** — patch adds `std::atomic<bool>` shutdown flag and `sync_thread.detach()` for clean ROS 2 lifecycle shutdown

## Open Questions

- Which fork becomes canonical after Jazzy validation: `zinuok` mainline or a maintained fallback?
- Authoritative calibration source for T265 fisheye intrinsics and body-to-camera extrinsics — current values captured 2026-04-03, needs cross-validation
- GPU enablement control: local patch/CMake option vs source edit inside vendored fork?

## ADDED Requirements

### Requirement: Selectable VINS-Fusion odometry bringup
The system SHALL provide a VINS-Fusion bringup path that consumes T265 stereo fisheye images and IMU data as a selectable visual-inertial odometry source.

#### Scenario: Standalone VINS bringup
- **WHEN** the VINS-Fusion bringup launch is started with T265 fisheye and IMU topics available
- **THEN** the VINS estimator SHALL subscribe to `/t265/fisheye1/image_raw`, `/t265/fisheye2/image_raw`, and `/t265/imu` and publish an odometry output for downstream adaptation

### Requirement: Rover-frame odometry adaptation
The system SHALL adapt the selected VINS-Fusion odometry output into the rover's standard odometry contract before exposing it to EKF and other consumers.

#### Scenario: Adapted VINS odometry output
- **WHEN** VINS-Fusion odometry is active
- **THEN** the odometry exposed to the rest of the stack as `/vins_odom` SHALL use `frame_id=odom` and `child_frame_id=ackermann/base_link`

### Requirement: GPU-capable VINS dependency path
The system SHALL provide a validated build path for VINS-Fusion with GPU-accelerated feature tracking dependencies in the Docker development environment.

#### Scenario: GPU-capable build environment
- **WHEN** the VINS-enabled Docker image is built on a CUDA-capable target
- **THEN** the image SHALL provide VINS-compatible Ceres and CUDA-enabled OpenCV dependencies required for GPU feature tracking

### Requirement: Preserved odometry-source comparability
The system SHALL preserve access to existing odometry topics when VINS-Fusion is introduced so VINS can be compared against current sources during validation.

#### Scenario: Debugging alongside existing odometry sources
- **WHEN** VINS-Fusion is selected as the active odometry source
- **THEN** the stack SHALL continue publishing the non-selected odometry topics needed for debugging and comparison, including RTAB-Map VO/ICP and T265 built-in odometry where their producers are enabled
