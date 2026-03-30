# command-system Specification

## Purpose
TBD - created by archiving change system-monitor. Update Purpose after archive.
## Requirements
### Requirement: Command Protobuf schema
The command system SHALL define the following Protobuf messages in `rover_health.proto`: `RoverCommand` (cmd_id UUID, command type, issued_at timestamp, issued_by identity, oneof payload), `CommandAck` (cmd_id, ack status, detail, timestamp), `NavGoal` (x_m, y_m, yaw_rad, tolerance_m), `SetMode` (mode string), `SetParam` (param_name, param_value). It SHALL define `CommandType` enum (CMD_UNKNOWN, CMD_NAV_GOAL, CMD_ARM, CMD_DISARM, CMD_SET_MODE, CMD_ESTOP, CMD_CANCEL_GOAL, CMD_SET_PARAM) and `AckStatus` enum (ACK_RECEIVED, ACK_ACCEPTED, ACK_REJECTED, ACK_COMPLETED, ACK_FAILED).

#### Scenario: Protobuf schema compiled
- **WHEN** `protobuf_generate_cpp()` runs on the extended `rover_health.proto`
- **THEN** it SHALL produce C++ stubs for all telemetry AND command message types without errors

#### Scenario: Python codegen for control_center
- **WHEN** `protoc --python_out` runs on the same `.proto` file
- **THEN** it SHALL produce `rover_health_pb2.py` usable by the host control_center

### Requirement: MQTT topic contract
The command system SHALL use the following MQTT topics for inbound commands (host → rover): `rover/cmd/goal` (QoS 2), `rover/cmd/arm` (QoS 2), `rover/cmd/mode` (QoS 2), `rover/cmd/estop` (QoS 2). Command acknowledgements SHALL be published by the rover to `rover/cmd/ack` (QoS 1).

#### Scenario: Command delivered exactly once
- **WHEN** the host publishes a `RoverCommand` to `rover/cmd/goal` with QoS 2
- **THEN** the MQTT broker SHALL guarantee exactly-once delivery to the rover subscriber

#### Scenario: ACK delivered at least once
- **WHEN** the rover publishes a `CommandAck` to `rover/cmd/ack` with QoS 1
- **THEN** the MQTT broker SHALL guarantee at-least-once delivery to the host subscriber

### Requirement: Command lifecycle
Each command SHALL follow the lifecycle: SENT → ACK_RECEIVED (command parsed) → ACK_ACCEPTED (safety gate passed, executing) or ACK_REJECTED (safety gate blocked) → ACK_COMPLETED (action succeeded) or ACK_FAILED (execution error). If no ACK_ACCEPTED or higher is received within `ack_timeout_s` (default 5 s), the host SHALL mark the command as FAILED with reason "ACK timeout".

#### Scenario: Successful command lifecycle
- **GIVEN** the rover is armed and in OFFBOARD mode
- **WHEN** the host sends CMD_NAV_GOAL
- **THEN** the rover SHALL reply with ACK_RECEIVED immediately, ACK_ACCEPTED after validation, and ACK_COMPLETED when the goal is reached

#### Scenario: Command timeout
- **WHEN** no ACK_ACCEPTED is received within `ack_timeout_s`
- **THEN** the host ack_tracker SHALL mark the command as ACK_FAILED with detail "ACK timeout" and notify the dashboard

### Requirement: Safety gate rules
The host command gateway SHALL enforce per-CommandType preconditions before publishing to MQTT. CMD_ESTOP SHALL bypass all preconditions and always be dispatched immediately.

#### Scenario: NavGoal preconditions
- **WHEN** an operator submits CMD_NAV_GOAL
- **THEN** the safety gate SHALL require `px4.armed == true`, `px4.nav_state == OFFBOARD`, `slam_latency_ms < 200`, `overall_health != ERROR`, and cache not stale — rejecting with reason if any fail

#### Scenario: Arm preconditions
- **WHEN** an operator submits CMD_ARM
- **THEN** the safety gate SHALL require `px4.connected == true`, `px4.battery_remaining_pct > 20`, and cache not stale

#### Scenario: E-stop bypass
- **WHEN** an operator submits CMD_ESTOP
- **THEN** the safety gate SHALL dispatch immediately with no precondition checks, even if the telemetry cache is stale

### Requirement: Rover-side command deduplication
The rover cmd_receiver SHALL maintain a bounded set of recently processed `cmd_id` values (evict entries older than 30 s) and silently drop duplicate commands to guard against MQTT QoS 2 re-delivery and network retransmits.

#### Scenario: Duplicate command received
- **GIVEN** a command with `cmd_id="abc-123"` was processed 5 seconds ago
- **WHEN** the same `cmd_id` arrives again via MQTT re-delivery
- **THEN** the cmd_receiver SHALL drop it without re-dispatching to ROS 2

#### Scenario: Expired dedup entry
- **GIVEN** a command with `cmd_id="abc-123"` was processed 35 seconds ago
- **WHEN** a new command with the same `cmd_id` arrives
- **THEN** the cmd_receiver SHALL treat it as a new command (dedup entry expired)

### Requirement: Rover-side command dispatch
The cmd_receiver SHALL map MQTT topics to ROS 2 interfaces for command execution.

#### Scenario: NavGoal dispatched
- **WHEN** a CMD_NAV_GOAL command arrives on `rover/cmd/goal`
- **THEN** it SHALL send a `NavigateToPose` action goal to Nav2 with the specified x, y, yaw in the map frame

#### Scenario: Arm/Disarm dispatched
- **WHEN** CMD_ARM or CMD_DISARM arrives on `rover/cmd/arm`
- **THEN** it SHALL publish to `/fmu/in/vehicle_command` with MAV_CMD_COMPONENT_ARM_DISARM

#### Scenario: Mode change dispatched
- **WHEN** CMD_SET_MODE arrives on `rover/cmd/mode`
- **THEN** it SHALL publish to `/fmu/in/vehicle_command` with MAV_CMD_DO_SET_MODE

#### Scenario: E-stop dispatched
- **WHEN** CMD_ESTOP arrives on `rover/cmd/estop`
- **THEN** it SHALL publish zero `TwistStamped` to `/cmd_vel` AND send CMD_DISARM to PX4

#### Scenario: Goal cancelled
- **WHEN** CMD_CANCEL_GOAL arrives on `rover/cmd/cancel`
- **THEN** it SHALL cancel the active `NavigateToPose` goal

