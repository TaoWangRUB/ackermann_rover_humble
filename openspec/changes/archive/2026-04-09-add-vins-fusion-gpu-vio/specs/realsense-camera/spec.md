## ADDED Requirements

### Requirement: T265 fisheye stream support for VIO consumers
The RealSense bringup SHALL support publishing T265 fisheye image streams for odometry consumers that require raw stereo fisheye input.

#### Scenario: T265 fisheye streams enabled
- **WHEN** `enable_t265:=true` and `t265_enable_fisheye:=true`
- **THEN** the T265 camera node SHALL publish `/t265/fisheye1/image_raw`, `/t265/fisheye2/image_raw`, and their corresponding camera-info topics alongside the T265 IMU topic

### Requirement: Top-level fisheye enablement compatibility
The RealSense bringup SHALL accept top-level launch control for T265 fisheye stream enablement so higher-level bringup can activate VINS prerequisites without manual per-camera overrides.

#### Scenario: Top-level launch requests VINS prerequisites
- **WHEN** higher-level bringup selects a VINS-based odometry mode
- **THEN** the RealSense launch SHALL honor the passed T265 fisheye enablement argument and start the required fisheye streams

#### Scenario: VINS mode not selected
- **WHEN** the T265 hardware path is enabled but higher-level bringup has not selected a VINS-based odometry mode
- **THEN** the RealSense launch SHALL keep T265 fisheye stream publishing disabled
