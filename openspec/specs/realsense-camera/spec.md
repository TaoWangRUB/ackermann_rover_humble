### Requirement: Multi-camera RealSense support
The system SHALL provide launch configuration for up to three RealSense cameras simultaneously: D435i (RGB-D + IMU), L515 (RGB-D), and T265 (visual tracking).

#### Scenario: D435i enabled
- **WHEN** `enable_d435i:=true` (default)
- **THEN** the D435i SHALL publish `d435i/color/image_raw`, `d435i/aligned_depth_to_color/image_raw`, and `d435i/imu` topics

#### Scenario: L515 enabled
- **WHEN** `enable_l515:=true`
- **THEN** the L515 SHALL publish `l515/color/image_raw` and `l515/aligned_depth_to_color/image_raw` (no IMU)

#### Scenario: T265 enabled
- **WHEN** `enable_t265:=true`
- **THEN** the T265 SHALL publish `/t265/odom` (VIO odometry) and the `odom_tf_relay` SHALL re-express it into `ackermann/base_link` child frame

### Requirement: Configurable camera parameters
The system SHALL allow per-camera configuration of resolution, frame rate, exposure, gain, depth alignment, and stream enables via launch parameters.

#### Scenario: Custom resolution
- **WHEN** `color_width`, `color_height`, `color_fps` are set
- **THEN** the camera node SHALL use the specified resolution and frame rate

### Requirement: USB power contention avoidance
The system SHALL support configurable startup delays (`startup_delay_s`) to prevent USB power races when multiple cameras share a bus.

#### Scenario: T265 startup delay
- **WHEN** T265 is enabled alongside a depth camera
- **THEN** the depth camera launch SHALL be delayed (default 12 s) to allow T265 to initialize first

### Requirement: Depth-to-color alignment
The system SHALL align depth frames to the color camera frame by default (`align_depth.enable: true`) for RGB-D SLAM consumption.

#### Scenario: Aligned depth output
- **WHEN** depth alignment is enabled
- **THEN** `{camera}/aligned_depth_to_color/image_raw` SHALL have the same resolution and intrinsics as the color stream
