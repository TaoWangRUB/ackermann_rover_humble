## ADDED Requirements

### Requirement: Camera device liveness detection
The cam_probe component SHALL detect RealSense camera connection state by monitoring the librealsense `rs2::context` device list and ROS 2 topic activity.

#### Scenario: Camera connected and streaming
- **WHEN** the RealSense device is present in the `rs2::context` device list and frame callbacks are firing
- **THEN** it SHALL report `connected=true` with `error_code=0`

#### Scenario: Camera disconnected
- **WHEN** the RealSense device disappears from the device list or no frame callbacks arrive within 500 ms
- **THEN** it SHALL report `connected=false` with `error_code=1` and `error_msg="Device disconnected"`

### Requirement: Frame delta micro-stutter detection
The cam_probe SHALL measure wall-clock interval between consecutive color frame callbacks to detect micro-stutters that rolling FPS averages smooth over.

#### Scenario: Normal frame delivery
- **WHEN** consecutive frame callbacks arrive within 33 ms (30 FPS nominal)
- **THEN** it SHALL report the measured `frame_delta_ms` with `error_code=0`

#### Scenario: Frame stutter detected
- **WHEN** `frame_delta_ms` exceeds 66 ms (below 15 FPS equivalent)
- **THEN** it SHALL report `error_code=2` with `error_msg="Camera frame delta exceeds 66 ms (below 15 FPS)"`

### Requirement: Depth quality fill ratio sampling
The cam_probe SHALL compute the depth image fill ratio (valid pixel count / total pixels) sampled at 1-in-10 frames to reduce per-second CPU load by 90%.

#### Scenario: Adequate depth quality
- **WHEN** the sampled fill ratio is above 0.5
- **THEN** it SHALL report `depth_quality_sampled` with the measured ratio and `error_code=0`

#### Scenario: Degraded depth quality
- **WHEN** the sampled fill ratio drops below 0.5
- **THEN** it SHALL report `error_code=3` with `error_msg="Depth fill ratio below 50% (sampled)"`

### Requirement: Depth FPS rolling average
The cam_probe SHALL compute a rolling 1-second average of depth stream frame rate.

#### Scenario: Depth FPS computed
- **WHEN** depth frames are being received
- **THEN** it SHALL report `depth_fps` as the 1-second rolling average frame rate

### Requirement: IMU stream liveness
The cam_probe SHALL monitor the RealSense IMU topic for liveness.

#### Scenario: IMU active
- **WHEN** IMU messages are being received on `/camera/imu`
- **THEN** it SHALL report `imu_active=true`

#### Scenario: IMU stream lost
- **WHEN** no IMU messages arrive for more than 500 ms
- **THEN** it SHALL report `imu_active=false` with `error_code=4`

### Requirement: Camera status publishing
The cam_probe SHALL publish a `CamStatus` message on `/monitor/cam` using intra-process shared pointer hand-off.

#### Scenario: Status published on every color frame callback
- **WHEN** a color frame callback fires
- **THEN** it SHALL publish a `CamStatus` message with all current metrics via `std::unique_ptr<CamStatus>` publish API
