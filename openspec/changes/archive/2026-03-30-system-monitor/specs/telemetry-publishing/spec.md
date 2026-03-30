## ADDED Requirements

### Requirement: Protobuf serialization
The telemetry_publisher component SHALL serialize `RoverHealth` data using Protocol Buffers binary encoding, not JSON. Protobuf is ~3-10x smaller and ~5x faster to serialize than equivalent JSON structs.

#### Scenario: RoverHealth serialized
- **WHEN** a RoverHealth message is received from the aggregator
- **THEN** it SHALL serialize the data using the `rover_health.proto` schema into Protobuf binary format

### Requirement: MQTT publishing with QoS split
The telemetry_publisher SHALL publish to an MQTT broker with QoS 0 for routine telemetry and QoS 1 (at-least-once) for alerts.

#### Scenario: Routine telemetry published
- **WHEN** the aggregator publishes a RoverHealth message
- **THEN** it SHALL publish to `rover/health/camera`, `rover/health/px4`, `rover/health/jetson`, and `rover/health/overall` MQTT topics with QoS 0

#### Scenario: Alert published with guaranteed delivery
- **WHEN** the RoverHealth message contains non-empty `active_alerts`
- **THEN** it SHALL publish to `rover/alerts` MQTT topic with QoS 1

### Requirement: MQTT reconnection
The telemetry_publisher SHALL automatically reconnect to the MQTT broker with a configurable delay if the connection drops.

#### Scenario: Broker connection lost
- **WHEN** the MQTT connection to the broker is lost
- **THEN** it SHALL attempt reconnection every `reconnect_delay_s` seconds (default 5) and log a warning on each retry

#### Scenario: Broker reconnected
- **WHEN** the MQTT connection is re-established
- **THEN** it SHALL resume publishing telemetry and alerts without data loss for QoS 1 messages

### Requirement: Configurable broker settings
The telemetry_publisher SHALL read broker host, port, client ID, QoS levels, and reconnect delay from YAML configuration.

#### Scenario: Custom broker config applied
- **WHEN** the config specifies `broker_host: "192.168.1.100"` and `broker_port: 1883`
- **THEN** it SHALL connect to that broker endpoint with the configured `client_id`

---

## ADDED Requirements (v1.2.0 — Command Receiver)

The telemetry_publisher component is extended with a cmd_receiver half that shares the single `paho::async_client` MQTT connection. This avoids a second MQTT socket on the Xavier.

### Requirement: MQTT command subscription
The cmd_receiver SHALL subscribe to inbound MQTT topics (`rover/cmd/goal`, `rover/cmd/arm`, `rover/cmd/mode`, `rover/cmd/estop`) with QoS 2, sharing the existing `paho::async_client` connection used by the telemetry publisher.

#### Scenario: Command subscription active
- **WHEN** the telemetry_publisher connects to the MQTT broker
- **THEN** it SHALL subscribe to all `rover/cmd/*` topics in addition to publishing on `rover/health/*`

#### Scenario: Subscription restored after reconnect
- **WHEN** the MQTT connection is re-established after a disconnect
- **THEN** it SHALL re-subscribe to both outbound health topics and inbound command topics

### Requirement: Protobuf command deserialization
The cmd_receiver SHALL deserialize incoming MQTT payloads as Protobuf `RoverCommand` messages using the extended `rover_health.proto` schema. Malformed payloads SHALL be logged and dropped.

#### Scenario: Valid command deserialized
- **WHEN** a valid Protobuf payload arrives on `rover/cmd/goal`
- **THEN** it SHALL deserialize into a `RoverCommand` with type CMD_NAV_GOAL and extract the `NavGoal` payload

#### Scenario: Malformed payload rejected
- **WHEN** a payload that fails Protobuf deserialization arrives
- **THEN** it SHALL log a warning with the topic and payload size and discard the message

### Requirement: Command deduplication
The cmd_receiver SHALL maintain a bounded set of recently processed `cmd_id` values (evict entries older than 30 s) and silently drop duplicate commands. See command-system spec for full deduplication requirements.

#### Scenario: Duplicate dropped
- **GIVEN** `cmd_id="abc-123"` was processed 5 seconds ago
- **WHEN** the same `cmd_id` arrives again
- **THEN** it SHALL drop the message without dispatching

### Requirement: Command dispatch to ROS 2
The cmd_receiver SHALL dispatch deserialized commands to the appropriate ROS 2 interface: `NavigateToPose` action for CMD_NAV_GOAL, `/fmu/in/vehicle_command` for CMD_ARM/CMD_DISARM/CMD_SET_MODE, zero `TwistStamped` + disarm for CMD_ESTOP, and goal cancel for CMD_CANCEL_GOAL.

#### Scenario: NavGoal dispatched
- **WHEN** CMD_NAV_GOAL is received with x=1.2, y=3.4, yaw=0.0
- **THEN** it SHALL send a `NavigateToPose` action goal with the specified pose in the map frame

#### Scenario: E-stop dispatched
- **WHEN** CMD_ESTOP is received
- **THEN** it SHALL immediately publish zero `TwistStamped` to `/cmd_vel` AND send disarm via `/fmu/in/vehicle_command`

### Requirement: CommandAck publishing
The cmd_receiver SHALL publish a `CommandAck` Protobuf message to `rover/cmd/ack` (QoS 1) for every processed command: ACK_RECEIVED immediately upon parsing, and ACK_ACCEPTED/ACK_REJECTED/ACK_COMPLETED/ACK_FAILED once the ROS 2 action or service responds.

#### Scenario: Two-phase ACK
- **WHEN** CMD_NAV_GOAL is received and validated
- **THEN** it SHALL publish ACK_RECEIVED immediately, then ACK_ACCEPTED when Nav2 accepts the goal, then ACK_COMPLETED when the goal is reached (or ACK_FAILED on abort)

### Requirement: Extended Protobuf schema
The `rover_health.proto` SHALL be extended with command-related messages: `RoverCommand`, `CommandAck`, `NavGoal`, `SetMode`, `SetParam`, `CommandType` enum, and `AckStatus` enum. The full schema is defined in the command-system spec.

#### Scenario: Extended proto compiles
- **WHEN** `protobuf_generate_cpp()` runs on the updated `rover_health.proto`
- **THEN** it SHALL produce stubs for both telemetry messages (CamStatus, Px4Status, JetsonStatus, RoverHealth) and command messages (RoverCommand, CommandAck) without errors
