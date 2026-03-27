### Requirement: Autonomous navigation with Nav2
The system SHALL provide autonomous point-to-point and waypoint navigation using Nav2 with MPPI controller configured for Ackermann kinematics.

#### Scenario: NavigateToPose goal
- **WHEN** a NavigateToPose action goal is sent
- **THEN** Nav2 SHALL plan a global path, track it with MPPI controller, and publish velocity commands on `/cmd_vel_nav`

#### Scenario: NavigateThroughPoses waypoints
- **WHEN** a NavigateThroughPoses action goal is sent with multiple poses
- **THEN** Nav2 SHALL navigate through each waypoint sequentially with replanning and recovery behaviors

### Requirement: MPPI controller with Ackermann motion model
The system SHALL use the MPPI (Model Predictive Path Integral) controller with Ackermann bicycle-model kinematics, 56 time steps, 1000 batch samples, and configurable velocity limits.

#### Scenario: Forward-only mode
- **WHEN** `reversible_drive:=false` (default)
- **THEN** the motion model SHALL be DUBIN (forward arcs only) with `vx_min: 0.0`, `vx_max: 0.5 m/s`, min turning radius 0.2 m

#### Scenario: Reversible drive mode
- **WHEN** `reversible_drive:=true`
- **THEN** the motion model SHALL be REEDS_SHEPP (bidirectional arcs) with `vx_min: -0.35 m/s`

### Requirement: Costmap configuration
The system SHALL maintain a global costmap (static layer + inflation from `/map`) and a local costmap (3 m × 3 m rolling window, voxel layer from `/scan`, inflation radius 0.70 m, robot radius 0.22 m).

#### Scenario: Obstacle detection
- **WHEN** the RPLiDAR scan detects obstacles within 3.0 m
- **THEN** the local costmap voxel layer SHALL mark occupied cells and the inflation layer SHALL expand costs within 0.70 m radius

### Requirement: Global path planning
The system SHALL generate global paths using NavFn (A*) planner with planning time < 2 seconds and no more than 3 retries before abort.

#### Scenario: Planning success
- **WHEN** a valid goal is reachable
- **THEN** a global path SHALL be generated within 2 seconds

#### Scenario: Planning failure
- **WHEN** the planner retries more than 3 times or aborts
- **THEN** the navigation goal SHALL be marked as failed

### Requirement: Path execution quality
The system SHALL track paths with final pose error < 0.25 m, final yaw error < 5 degrees, RMS lateral error < 0.2 m, and steering saturation < 95%.

#### Scenario: Goal reached
- **WHEN** the rover reaches the goal
- **THEN** final position error SHALL be < 0.25 m and yaw error SHALL be < 5 degrees

#### Scenario: Collision avoidance
- **WHEN** the collision monitor detects imminent collision
- **THEN** the rover SHALL stop or reroute; any Gazebo collision is treated as failure

### Requirement: Velocity smoothing
The system SHALL smooth velocity commands via nav2_velocity_smoother with acceleration limits matching the Ackermann platform constraints.

#### Scenario: Smooth acceleration
- **WHEN** a new velocity command is issued
- **THEN** the output SHALL ramp to the target respecting configured acceleration limits, preventing jerk

### Requirement: Recovery behaviors
The system SHALL provide recovery behaviors (spin, backup) via the behavior server when the controller fails to make progress.

#### Scenario: Stuck recovery
- **WHEN** oscillation recovery is triggered
- **THEN** the rover SHALL execute recovery behaviors; more than one oscillation recovery per goal is treated as degraded performance
