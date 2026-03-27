### Requirement: URDF/SDF robot model
The system SHALL provide a URDF/Xacro robot description defining an Ackermann-steered rover with base_link, 4 wheel joints (2 rear continuous drive, 2 front revolute steering with ±0.6 rad limits), and sensor mounting frames.

#### Scenario: TF tree availability
- **WHEN** the robot model is loaded
- **THEN** the TF tree SHALL include `ackermann/base_footprint` → `ackermann/base_link` and all wheel/sensor frames

### Requirement: Configurable sensor suite
The system SHALL support optional sensors via Xacro arguments: depth camera (D435i or L515), T265 tracking camera, RPLiDAR 2D scanner, and CubePilot IMU/GPS.

#### Scenario: Depth camera selection
- **WHEN** `depth_camera:=d435i` is set
- **THEN** the D435i model with RGB-D + IMU SHALL be included in the URDF; when `depth_camera:=l515`, the L515 model (no IMU) SHALL be used instead

#### Scenario: Optional sensors disabled
- **WHEN** `enable_rplidar:=false` or `enable_t265:=false` or `enable_cubepilot:=false`
- **THEN** the corresponding sensor link and plugin SHALL be excluded from the model

### Requirement: Gazebo simulation plugins
The system SHALL configure Gazebo plugins for physics-based simulation: ros2_control bridge, odometry publisher (`/ackermann/odom`), joint state publisher (`/joint_states`), and sensor simulation plugins.

#### Scenario: Standard simulation mode
- **WHEN** `enable_px4_sitl:=false` (default)
- **THEN** Gazebo SHALL use `gz_ros2_control::GazeboSimROS2ControlPlugin` for joint control via ros2_control

#### Scenario: PX4 SITL mode
- **WHEN** `enable_px4_sitl:=true`
- **THEN** Gazebo SHALL use direct joint controller/position plugins (bypassing ros2_control) so PX4 drives the vehicle dynamics

### Requirement: Gazebo-ROS topic bridge
The system SHALL bridge Gazebo simulation topics to ROS 2 via `ros_gz_bridge`: clock, camera streams (image, depth, camera_info, IMU), lidar scan, odometry, TF, and joint states.

#### Scenario: Sensor topics available
- **WHEN** Gazebo simulation is running
- **THEN** ROS 2 topics for the selected depth camera, RPLiDAR scan, `/ackermann/odom`, and `/joint_states` SHALL be publishing
