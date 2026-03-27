## ADDED Requirements

### Requirement: Docker Compose deployment
The host-side infrastructure SHALL be deployed via a single `docker-compose.yaml` with three services: `mosquitto` (Eclipse Mosquitto 2), `influxdb` (InfluxDB 2), and `control_center` (Python application). The control_center service SHALL depend on mosquitto and influxdb.

#### Scenario: All services started
- **WHEN** the operator runs `docker compose up -d`
- **THEN** Mosquitto SHALL be available on port 1883, InfluxDB on port 8086, and the control_center dashboard on port 8080

#### Scenario: Service dependency ordering
- **WHEN** the control_center starts
- **THEN** Mosquitto and InfluxDB SHALL already be running (via `depends_on`)

### Requirement: Mosquitto broker configuration
The Mosquitto broker SHALL listen on port 1883 with persistence enabled for QoS 1/2 messages. It SHALL allow anonymous connections on the local network (TLS is an open question for non-LAN deployments).

#### Scenario: QoS 1 message persisted
- **WHEN** a QoS 1 alert message is published while the control_center subscriber is temporarily disconnected
- **THEN** Mosquitto SHALL deliver the message when the subscriber reconnects

### Requirement: InfluxDB setup
InfluxDB SHALL be initialized with org `rover`, bucket `rover_telemetry`, and an admin token passed via `INFLUX_TOKEN` environment variable. Data retention policy SHALL be configurable (default: indefinite).

#### Scenario: Bucket auto-created
- **WHEN** the InfluxDB container starts for the first time
- **THEN** it SHALL create the `rover_telemetry` bucket in the `rover` org using init environment variables

### Requirement: Control center package structure
The control_center SHALL be structured as a Python package with: `main.py` (entrypoint), `cc/` module directory (mqtt_receiver, telemetry_cache, influxdb_writer, alert_notifier, dashboard_server, cmd_gateway, safety_gate, ack_tracker, event_bus), `config/` (control_center.yaml, notifier.yaml), `proto/` (rover_health_pb2.py generated), `ui/dist/` (built React SPA static files), and `requirements.txt`.

#### Scenario: Control center starts
- **WHEN** `python main.py` is invoked
- **THEN** it SHALL load `control_center.yaml`, start all components, connect to MQTT and InfluxDB, and begin serving the dashboard on the configured port

### Requirement: Master configuration (control_center.yaml)
The control_center SHALL be configured via `control_center.yaml` with sections for: `mqtt` (host, port, client_id, reconnect delays), `telemetry_cache` (stale_threshold_s), `dashboard` (host, port, cors_origins), `influxdb` (url, org, bucket, batch settings), `cmd_gateway` (ack_timeout_s, estop_topic, default_goal_tolerance_m), and `notifier` (cooldown_s).

#### Scenario: Config loaded at startup
- **WHEN** the control_center starts
- **THEN** it SHALL read all settings from `control_center.yaml` and apply them to each component

### Requirement: Notifier configuration (notifier.yaml)
Alert notification channels SHALL be configured via `notifier.yaml` with per-channel settings: `id`, `enabled`, `min_severity`, and channel-specific fields (terminal format string, sound file path, desktop backend, Slack webhook URL, email SMTP host and recipients).

#### Scenario: Slack channel configured
- **WHEN** `notifier.yaml` has a Slack channel with `enabled: true` and a `webhook_url`
- **THEN** the alert_notifier SHALL POST to that webhook when an alert meets the `min_severity` threshold

### Requirement: React SPA build
The dashboard UI SHALL be built from the `ui/src/` React+TypeScript source via `npm run build`, producing static assets in `ui/dist/`. The Docker image SHALL use a multi-stage build to avoid shipping Node.js in the production image.

#### Scenario: Docker image built
- **WHEN** `docker compose build control_center` runs
- **THEN** the first stage SHALL build the React SPA and the second stage SHALL copy `dist/` into the Python runtime image
