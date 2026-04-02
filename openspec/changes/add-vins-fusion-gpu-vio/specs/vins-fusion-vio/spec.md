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
