### Requirement: Top-level launch orchestration
The system SHALL provide a single `robot_bringup.launch.py` that conditionally launches Gazebo (or hardware drivers), RTAB-Map SLAM, Nav2 navigation, and RViz based on launch arguments.

#### Scenario: Simulation mode
- **WHEN** `use_gazebo:=true` (default)
- **THEN** the launch SHALL start Gazebo with ros2_control, sensor bridges, and robot spawning

#### Scenario: Hardware mode
- **WHEN** `use_gazebo:=false`
- **THEN** the launch SHALL start robot_state_publisher from URDF, RealSense camera drivers, and `cmd_vel_joint_relay` for RViz joint visualization

### Requirement: Conditional subsystem activation
The system SHALL support enabling/disabling RTAB-Map (`rtabmap`), Nav2 (`nav2`), and RViz (`rviz`) independently via boolean launch arguments.

#### Scenario: SLAM-only launch
- **WHEN** `rtabmap:=true` and `nav2:=false`
- **THEN** RTAB-Map SHALL start but Nav2 SHALL not be launched

#### Scenario: Full stack launch
- **WHEN** `rtabmap:=true` and `nav2:=true`
- **THEN** both RTAB-Map and Nav2 SHALL be active with proper topic remapping

### Requirement: Depth camera selection
The system SHALL support switching between D435i and L515 depth cameras via the `depth_camera` launch argument, with automatic topic remapping for downstream consumers (RTAB-Map, Gazebo bridge).

#### Scenario: Camera topic remapping
- **WHEN** `depth_camera:=d435i`
- **THEN** all camera topic remappings in RTAB-Map and Gazebo bridge SHALL point to D435i topics; similarly for L515

### Requirement: PX4 SITL mode
The system SHALL support PX4 SITL integration via `enable_px4_sitl:=true`, which disables ros2_control and enables PX4 joint plugins in Gazebo.

#### Scenario: PX4 SITL dynamics
- **WHEN** `enable_px4_sitl:=true`
- **THEN** ros2_control SHALL be disabled and PX4 SHALL drive vehicle dynamics through Gazebo joint plugins

### Requirement: cmd_vel_joint_relay for hardware mode
The system SHALL provide a `cmd_vel_joint_relay` Python node that derives JointState messages from `/cmd_vel` using inverse Ackermann kinematics (wheelbase=0.174 m, track=0.174 m, wheel_radius=0.0385 m) at 50 Hz for RViz visualization when not in simulation.

#### Scenario: Hardware joint visualization
- **WHEN** running in hardware mode (`use_gazebo:=false`)
- **THEN** `/joint_states` SHALL publish computed steering angles and integrated wheel rotation positions at 50 Hz based on `/cmd_vel` commands
