## Architecture / Data Flow

```
T265 Fisheye (848x800 @30Hz) + IMU (@200 Hz)
  → realsense_camera_node (fisheye auto-enabled when use_cuvslam_odom:=true)
  → /t265/fisheye1/image_raw, /t265/fisheye2/image_raw
  → /t265/fisheye1/camera_info, /t265/fisheye2/camera_info, /t265/imu
  → [cuvslam_odom_node] (CameraInfo-driven rig init, stereo VIO, IMU integration)
  → /cuvslam/raw_odometry
  → [odom_tf_relay] (frame adaptation: sensor-native pose → ackermann/base_link)
  → /cuvslam_odom (odom → ackermann/base_link)
  → EKF (odom0, IMU yaw fusion disabled for external VIO)
  → /odometry/filtered → RTAB-Map / Nav2 / PX4
```

**Odometry priority**: cuVSLAM > VINS-Fusion > T265 built-in > RGB-D VO > ICP

**Key nodes**:
- `cuvslam_odom_node` (`cuvslam_bringup`) — cuVSLAM ROS 2 wrapper that owns rig setup, stereo sync, IMU forwarding, and raw odometry publication
- `cuvslam_odom_relay` (`realsense_camera_bringup/odom_tf_relay`) — adapts raw cuVSLAM output to the rover frame contract
- `robot_localization` EKF — fuses `/cuvslam_odom` as `odom0` when cuVSLAM is selected

**Key files**:
- `src/cuvslam_bringup/src/cuvslam_odom_node.cpp` — wrapper node around `cuvslam2.h`
- `src/cuvslam_bringup/config/t265_stereo_fisheye.yaml` — wrapper configuration and topic policy
- `src/cuvslam_bringup/launch/cuvslam.launch.py` — cuVSLAM launch + relay wiring
- `docker/install_cuvslam_deps.sh` — architecture-aware cuVSLAM source build tooling
- `scripts/build_cuvslam.sh` — workspace build entrypoint for cuVSLAM integration

## ADDED Requirements

### Requirement: Selectable cuVSLAM odometry bringup
The system SHALL provide a cuVSLAM bringup path that consumes T265 stereo fisheye images and IMU data as a selectable visual-inertial odometry source.

#### Scenario: Standalone cuVSLAM bringup
- **WHEN** the cuVSLAM bringup launch is started with T265 fisheye images, CameraInfo, and IMU topics available
- **THEN** the cuVSLAM wrapper SHALL subscribe to the T265 stereo fisheye and IMU topics and publish a raw odometry output for downstream adaptation

### Requirement: CameraInfo-driven rig initialization
The cuVSLAM wrapper SHALL initialize its stereo rig from the live T265 CameraInfo topics before tracking frames.

#### Scenario: First valid stereo calibration received
- **WHEN** the wrapper receives a valid left and right CameraInfo pair for the selected fisheye topics
- **THEN** it SHALL build the cuVSLAM rig from those messages before invoking frame tracking

### Requirement: Rover-frame odometry adaptation
The system SHALL adapt cuVSLAM odometry into the rover's standard odometry contract before exposing it to EKF and other consumers.

#### Scenario: Adapted cuVSLAM odometry output
- **WHEN** cuVSLAM odometry is active
- **THEN** the odometry exposed to the rest of the stack as `/cuvslam_odom` SHALL use `frame_id=odom` and `child_frame_id=ackermann/base_link`

### Requirement: Odometry-only cuVSLAM integration
The cuVSLAM integration SHALL operate as an odometry provider and SHALL not replace RTAB-Map as the mapping owner in the rover stack.

#### Scenario: cuVSLAM active with RTAB-Map
- **WHEN** cuVSLAM is selected while RTAB-Map SLAM is enabled
- **THEN** cuVSLAM SHALL provide odometry only and RTAB-Map SHALL remain responsible for `/map` publication and the `map -> odom` transform

### Requirement: Architecture-aware cuVSLAM source build path
The system SHALL provide a validated source-build path for cuVSLAM in the Docker environment on both x86_64 and Jetson Xavier-class aarch64 targets.

#### Scenario: x86_64 source build
- **WHEN** the cuVSLAM dependency build is run on an x86_64 target with the host CUDA toolkit mounted into the container
- **THEN** the build tooling SHALL configure cuVSLAM against that toolkit, install an architecture-qualified artifact cache, and provide a smoke-testable library output

#### Scenario: Jetson Xavier source build
- **WHEN** the cuVSLAM dependency build is run on a Jetson Xavier-class aarch64 target with CUDA 11.4-era tooling
- **THEN** the build tooling SHALL use a compatible GCC 11 host compiler, shared CUDA runtime linkage, and the repo's CUDA/glibc compatibility pattern so the library can be compiled and smoke-tested from source

### Requirement: Preserved odometry-source comparability
The system SHALL preserve access to the existing odometry topics when cuVSLAM is introduced so operators can compare cuVSLAM with the other odometry sources.

#### Scenario: Debugging alongside other odometry sources
- **WHEN** cuVSLAM is selected as the active odometry source
- **THEN** the stack SHALL continue publishing the non-selected odometry topics needed for debugging and comparison, including VINS-Fusion, RTAB-Map VO/ICP, and T265 built-in odometry where their producers are enabled
