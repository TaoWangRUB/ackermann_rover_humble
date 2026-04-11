## MODIFIED Requirements

### Requirement: EKF sensor fusion
The system SHALL fuse the selected odometry source and IMU data using a robot_localization EKF at 30 Hz in 2D mode, publishing `/odometry/filtered` in ENU/FLU convention.

#### Scenario: Filtered odometry with RTAB-Map VO or ICP
- **WHEN** the EKF is running with RTAB-Map visual odometry or ICP selected as odom0
- **THEN** `/odometry/filtered` SHALL fuse odom0 with IMU angular-rate data needed to stabilize heading

#### Scenario: Filtered odometry with external VIO
- **WHEN** the EKF is running with cuVSLAM, VINS-Fusion, or T265 built-in odometry selected as odom0
- **THEN** `/odometry/filtered` SHALL use the selected external VIO odometry as odom0 and SHALL disable the redundant IMU yaw-rate fusion that would double-count heading information

### Requirement: External VIO odometry source selection
The SLAM stack SHALL support cuVSLAM and VINS-Fusion as external odometry sources in addition to T265 built-in odometry, RTAB-Map visual odometry, and ICP odometry.

#### Scenario: cuVSLAM selected as odometry source
- **WHEN** `use_cuvslam_odom:=true`
- **THEN** the SLAM stack SHALL use `/cuvslam_odom` as the odometry input consumed by RTAB-Map and the EKF

#### Scenario: cuVSLAM takes priority over other external VIO modes
- **WHEN** `use_cuvslam_odom:=true` and one or more lower-priority external VIO flags are also enabled
- **THEN** the SLAM stack SHALL keep `/cuvslam_odom` as the selected odometry input

#### Scenario: VINS selected as odometry source
- **WHEN** `use_cuvslam_odom:=false` and `use_vins_odom:=true`
- **THEN** the SLAM stack SHALL use `/vins_odom` as the odometry input consumed by RTAB-Map and the EKF

#### Scenario: T265 built-in odometry selected
- **WHEN** `use_cuvslam_odom:=false` and `use_vins_odom:=false` and `use_t265_odom:=true`
- **THEN** the SLAM stack SHALL use the relayed T265 odometry topic as the odometry input consumed by RTAB-Map and the EKF
