### Requirement: Visual SLAM with RTAB-Map
The system SHALL provide 2D occupancy-grid SLAM using RTAB-Map with RGB-D input, visual odometry (ORB features), IMU fusion via EKF, and loop-closure detection.

#### Scenario: Map generation
- **WHEN** RTAB-Map SLAM is running with visual odometry enabled
- **THEN** `/rtabmap/map` SHALL publish an OccupancyGrid, `/vo_odom` SHALL publish visual odometry, and the `map → odom` TF transform SHALL be available

#### Scenario: Localization-only mode
- **WHEN** `localization:=true`
- **THEN** RTAB-Map SHALL load a saved database and localize against it without creating new loop closures (`Mem/IncrementalMemory: false`)

### Requirement: Visual odometry
The system SHALL compute visual odometry from RGB-D frames using ORB feature extraction (1000 features, 20 m max depth) in 2D-constrained mode (`Reg/Force3DoF: true`), publishing to `/vo_odom`.

#### Scenario: Visual odometry output
- **WHEN** `vision:=true` (default)
- **THEN** `/vo_odom` (nav_msgs/Odometry) SHALL publish with frame_id=`odom`, child_frame_id=`ackermann/base_link`

#### Scenario: ICP odometry fallback
- **WHEN** `vision:=false`
- **THEN** ICP-based odometry SHALL be used instead, publishing to `/icp_odom`

### Requirement: IMU preprocessing pipeline
The system SHALL transform raw IMU data from sensor frame to `ackermann/base_link`, then fuse orientation via Madgwick filter (ENU world frame, no TF publish), outputting to `/imu/data`.

#### Scenario: IMU chain
- **WHEN** the SLAM stack is running
- **THEN** raw IMU → `imu_transformer` (frame transform) → `imu_filter_madgwick` (orientation fusion) → `/imu/data` SHALL be active

### Requirement: EKF sensor fusion
The system SHALL fuse the selected odometry source and IMU data using a robot_localization EKF at 30 Hz in 2D mode, publishing `/odometry/filtered` in ENU/FLU convention.

#### Scenario: Filtered odometry with RTAB-Map VO or ICP
- **WHEN** the EKF is running with RTAB-Map visual odometry or ICP selected as odom0
- **THEN** `/odometry/filtered` SHALL fuse odom0 with IMU angular-rate data needed to stabilize heading

#### Scenario: Filtered odometry with external VIO
- **WHEN** the EKF is running with VINS-Fusion or T265 built-in odometry selected as odom0
- **THEN** `/odometry/filtered` SHALL use the selected external VIO odometry as odom0 and SHALL disable the redundant IMU yaw-rate fusion that would double-count heading information

### Requirement: Depth-to-laserscan conversion
The system SHALL convert depth images to 2D laser scan (`/scan`) with range 0.1–20.0 m for Nav2 costmap consumption.

#### Scenario: Scan output
- **WHEN** depth images are available
- **THEN** `/scan` (sensor_msgs/LaserScan) SHALL publish at the depth camera frame rate

### Requirement: Localization stability
The system SHALL maintain pose drift below 0.1 m and yaw drift below 3 degrees over 10 seconds when the robot is stationary.

#### Scenario: Stationary drift check
- **WHEN** the robot is stationary for 10 seconds
- **THEN** position drift SHALL be < 0.1 m and yaw drift SHALL be < 3 degrees

### Requirement: External VIO odometry source selection
The SLAM stack SHALL support VINS-Fusion as an external odometry source in addition to T265 built-in odometry, RTAB-Map visual odometry, and ICP odometry.

#### Scenario: VINS selected as odometry source
- **WHEN** `use_vins_odom:=true`
- **THEN** the SLAM stack SHALL use `/vins_odom` as the odometry input consumed by RTAB-Map and the EKF

#### Scenario: T265 built-in odometry selected
- **WHEN** `use_vins_odom:=false` and `use_t265_odom:=true`
- **THEN** the SLAM stack SHALL use the relayed T265 odometry topic as the odometry input consumed by RTAB-Map and the EKF
