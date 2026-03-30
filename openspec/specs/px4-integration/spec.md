### Requirement: PX4 custom rover modes via XRCE-DDS
The system SHALL provide C++ custom PX4 rover modes via the px4-ros2-interface-lib over Micro XRCE-DDS, supporting speed+steering, speed+attitude, manual, and offboard trajectory modes.

#### Scenario: Speed steering mode
- **WHEN** `mode_type:=speed_steering`
- **THEN** the node SHALL subscribe to `/cmd_vel` (Twist), map `linear.x` → RoverSpeedSetpoint and `angular.z` → RoverSteeringSetpoint, and publish to PX4

#### Scenario: Manual mode
- **WHEN** `mode_type:=manual` (default)
- **THEN** the node SHALL pass through manual control inputs with optional bidirectional ESC support (`bidirectional_esc` parameter)

#### Scenario: FMU unavailable
- **WHEN** the PX4 FMU is not responding
- **THEN** the mode node SHALL retry connection with 5-second delay between attempts

### Requirement: Vision odometry bridge (ROS 2 → PX4)
The system SHALL bridge `/odometry/filtered` (ENU/FLU) to `/fmu/in/vehicle_visual_odometry` (NED/FRD) at 10 Hz, performing coordinate frame conversion.

#### Scenario: Coordinate conversion
- **WHEN** an ENU/FLU odometry message is received
- **THEN** position SHALL be converted ENU→NED `[E,N,U]→[N,E,-U]`, quaternion SHALL be converted ENU/FLU→NED/FRD, velocity SHALL be converted body FLU→FRD (negate Y and Z)

#### Scenario: Publish rate limiting
- **WHEN** the VO bridge is running
- **THEN** it SHALL publish at 10 Hz maximum to avoid overloading the Cube Black serial link (921600 baud)

#### Scenario: Odometry timeout detection
- **WHEN** no odometry is received for 5 seconds
- **THEN** the bridge SHALL log a warning indicating odometry source is stale

### Requirement: Vehicle odometry bridge (PX4 → ROS 2)
The system SHALL optionally bridge `/fmu/out/vehicle_odometry` (NED/FRD) to `/px4_vehicle_odom` (ENU/FLU) for monitoring PX4's internal state estimate.

#### Scenario: Reverse conversion
- **WHEN** `enable_vehicle_odometry:=true` (default)
- **THEN** the node SHALL convert PX4 NED/FRD odometry to ROS 2 ENU/FLU convention and publish on `/px4_vehicle_odom`

### Requirement: MAVLink transport alternative
The system SHALL support MAVLink-over-serial as an alternative to XRCE-DDS for vision odometry delivery, selectable via `odometry_transport:=mavlink`.

#### Scenario: MAVLink mode
- **WHEN** `odometry_transport:=mavlink`
- **THEN** the system SHALL use `px4_mavlink_vpe.py` to send VPE messages over the configured serial device

### Requirement: DDS topic allowlist
The system SHALL configure a DDS topic allowlist (`dds_topics.yaml`) limiting which uORB topics are bridged over XRCE-DDS to reduce CPU load on the Cube Black flight controller.

#### Scenario: Allowlist enforcement
- **WHEN** the MicroXRCEAgent is running
- **THEN** only topics listed in `dds_topics.yaml` SHALL be bridged; this includes vehicle_status, battery_status, vehicle_odometry, and all rover setpoint topics

### Requirement: PX4 offboard mode stability
The system SHALL maintain PX4 OFFBOARD mode without timeout or failsafe triggers when the mode node is active, with command rate meeting PX4's minimum threshold.

#### Scenario: Offboard maintained
- **WHEN** a custom mode node is running
- **THEN** PX4 SHALL remain in OFFBOARD mode with command frequency >= configured threshold; offboard mode drop or failsafe trigger is treated as failure

### Requirement: Separate PX4 launch
The PX4 bringup SHALL NOT be launched from `robot_bringup.launch.py`. It SHALL be started separately after MicroXRCEAgent and PX4 SITL/hardware are running.

#### Scenario: Launch sequence
- **WHEN** starting the full stack
- **THEN** the operator SHALL first start MicroXRCEAgent, then PX4 (SITL or hardware), then `px4_bringup.launch.py` separately
