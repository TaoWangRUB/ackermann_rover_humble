### Requirement: Ackermann steering command conversion
The system SHALL convert geometry_msgs/Twist velocity commands into ackermann_msgs/AckermannDriveStamped messages using bicycle-model kinematics with configurable wheelbase, max steering angle, and max speed limits.

#### Scenario: Normal velocity command conversion
- **WHEN** a Twist message is received on `/cmd_vel` with `linear.x` and `angular.z`
- **THEN** the system SHALL compute `steering_angle = atan(wheelbase * angular.z / linear.x)`, clamp to `±max_steering_angle`, clamp speed to `±max_speed`, and publish an AckermannDriveStamped on `/cmd_ackermann`

#### Scenario: Zero linear velocity
- **WHEN** a Twist message has `linear.x == 0`
- **THEN** the system SHALL publish zero speed and zero steering angle (avoid division by zero)

### Requirement: ros2_control Ackermann steering controller
The system SHALL provide a ros2_control `ackermann_steering_controller` managing four wheel joints (two rear drive, two front steering) with closed-loop wheel feedback at 50 Hz.

#### Scenario: Controller active in simulation
- **WHEN** Gazebo is running with ros2_control enabled (not PX4 SITL mode)
- **THEN** `joint_state_broadcaster` and `ackermann_steering_controller` SHALL be in `active` state, `/joint_states` SHALL publish at 50 Hz, and `/ackermann_steering_controller/odometry` SHALL publish wheel-based odometry

#### Scenario: Stamped velocity input
- **WHEN** `use_stamped_vel` is true (default)
- **THEN** the controller SHALL accept `geometry_msgs/TwistStamped` on its command topic

#### Scenario: Command timeout
- **WHEN** no velocity command is received for `reference_timeout` seconds (default 2.0)
- **THEN** the controller SHALL stop the rover (zero speed, maintain last steering angle)

### Requirement: Vehicle kinematics parameters
The system SHALL use configurable kinematics parameters: wheelbase (default 0.174 m), front/rear wheel track (default 0.174 m), front/rear wheel radius (default 0.0385 m), and steering angle limits (±0.6 rad).

#### Scenario: Parameter loading
- **WHEN** the controller manager starts
- **THEN** it SHALL load kinematics parameters from `ackermann_controller.yaml` and apply them to the steering controller
