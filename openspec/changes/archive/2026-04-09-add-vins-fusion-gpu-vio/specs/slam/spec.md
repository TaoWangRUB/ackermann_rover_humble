## ADDED Requirements

### Requirement: External VIO odometry source selection
The SLAM stack SHALL support VINS-Fusion as an external odometry source in addition to T265 built-in odometry, RTAB-Map visual odometry, and ICP odometry.

#### Scenario: VINS selected as odometry source
- **WHEN** `use_vins_odom:=true`
- **THEN** the SLAM stack SHALL use `/vins_odom` as the odometry input consumed by RTAB-Map and the EKF

#### Scenario: T265 built-in odometry selected
- **WHEN** `use_vins_odom:=false` and `use_t265_odom:=true`
- **THEN** the SLAM stack SHALL use the relayed T265 odometry topic as the odometry input consumed by RTAB-Map and the EKF

## MODIFIED Requirements

### Requirement: EKF sensor fusion
The system SHALL fuse the selected odometry source and IMU data using a robot_localization EKF at 30 Hz in 2D mode, publishing `/odometry/filtered` in ENU/FLU convention.

#### Scenario: Filtered odometry with RTAB-Map VO or ICP
- **WHEN** the EKF is running with RTAB-Map visual odometry or ICP selected as odom0
- **THEN** `/odometry/filtered` SHALL fuse odom0 with IMU angular-rate data needed to stabilize heading

#### Scenario: Filtered odometry with external VIO
- **WHEN** the EKF is running with VINS-Fusion or T265 built-in odometry selected as odom0
- **THEN** `/odometry/filtered` SHALL use the selected external VIO odometry as odom0 and SHALL disable the redundant IMU yaw-rate fusion that would double-count heading information
