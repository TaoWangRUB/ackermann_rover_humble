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
