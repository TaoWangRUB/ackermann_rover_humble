### Requirement: Fault-triggered emergency stop
The system SHALL provide a watchdog node that subscribes to `/fault` (std_msgs/Bool) and publishes an immediate stop command (AckermannDriveStamped with speed=0, steering_angle=0) on `/cmd_ackermann` when a fault is detected.

#### Scenario: Fault received
- **WHEN** a message with `data == true` is received on `/fault`
- **THEN** the watchdog SHALL immediately publish a zero-speed, zero-steering AckermannDriveStamped on `/cmd_ackermann` and log a warning

#### Scenario: No fault
- **WHEN** no fault messages are received or `data == false`
- **THEN** the watchdog SHALL not publish any stop commands
