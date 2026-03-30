# control-center Specification

## Purpose
TBD - created by archiving change system-monitor. Update Purpose after archive.
## Requirements
### Requirement: Async MQTT subscription
The mqtt_receiver SHALL maintain a single `paho.mqtt.client` connection to the Mosquitto broker, subscribing to all `rover/#` topics. It SHALL deserialize Protobuf payloads and emit typed events onto the internal asyncio event bus via `loop.call_soon_threadsafe`.

#### Scenario: Telemetry received
- **WHEN** a message arrives on `rover/health/overall` (QoS 0)
- **THEN** it SHALL deserialize the payload as `RoverHealth` Protobuf and emit `evt.health` on the event bus

#### Scenario: Alert received
- **WHEN** a message arrives on `rover/alerts` (QoS 1)
- **THEN** it SHALL deserialize the payload as `RoverHealth` Protobuf and emit `evt.alert` on the event bus

#### Scenario: CommandAck received
- **WHEN** a message arrives on `rover/cmd/ack` (QoS 1)
- **THEN** it SHALL deserialize the payload as `CommandAck` Protobuf and emit `evt.cmd_ack` on the event bus

### Requirement: MQTT reconnection
The mqtt_receiver SHALL reconnect automatically with exponential backoff (min 1 s, max 30 s) and re-subscribe to all topics on reconnect via the `on_connect` callback.

#### Scenario: Broker connection lost
- **WHEN** the MQTT connection drops
- **THEN** it SHALL emit `evt.connection_lost` on the bus and attempt reconnection with backoff

#### Scenario: Broker reconnected
- **WHEN** the connection is re-established
- **THEN** it SHALL re-subscribe to all `rover/#` topics and emit `evt.connection_restored`

---

### CC-2: Telemetry Cache

### Requirement: In-memory latest-value store
The telemetry_cache SHALL maintain the latest snapshot of every `rover/health/*` topic in an `asyncio.Lock`-protected dict keyed by subsystem (health, cam, px4, jetson) with an `updated_at` monotonic timestamp.

#### Scenario: Cache updated
- **WHEN** an `evt.health` event is received
- **THEN** it SHALL update the `health` entry and set `updated_at` to the current monotonic time

### Requirement: Staleness detection
The telemetry_cache SHALL report itself as stale if `time.monotonic() - updated_at > stale_threshold_s` (default 5 s). The safety gate SHALL treat a stale cache as an ERROR state and block all commands except CMD_ESTOP.

#### Scenario: Cache stale
- **GIVEN** the last telemetry update was 6 seconds ago and `stale_threshold_s` is 5
- **WHEN** the safety gate reads the cache
- **THEN** it SHALL treat the rover state as ERROR and reject all commands except CMD_ESTOP

---

### CC-3: InfluxDB Writer

### Requirement: Async batch writes for telemetry
The influxdb_writer SHALL write telemetry data to the `rover_telemetry` measurement using `WriteApi` in ASYNCHRONOUS mode with configurable batch size (default 50) and flush interval (default 2000 ms). It SHALL drop oldest points if the buffer exceeds 1000 points.

#### Scenario: Telemetry batched
- **WHEN** `evt.health` events arrive at 2 Hz
- **THEN** it SHALL buffer points and flush to InfluxDB every 2 seconds

### Requirement: Sync writes for alerts and commands
The influxdb_writer SHALL write alert events (measurement `rover_alerts`) and command audit records (measurement `rover_commands`) immediately using SYNCHRONOUS mode. These SHALL NOT be batched or dropped.

#### Scenario: Alert persisted immediately
- **WHEN** an `evt.alert` event is received
- **THEN** it SHALL write to InfluxDB synchronously before returning

#### Scenario: Command logged immediately
- **WHEN** a command is dispatched or an ACK is received
- **THEN** it SHALL write to `rover_commands` measurement synchronously with cmd_id, cmd_type, issued_by, ack_status, and round_trip_ms

### Requirement: Connection failure handling
The influxdb_writer SHALL log errors and continue operating if InfluxDB is unavailable. Live dashboard functionality (via WebSocket from cache) SHALL NOT be affected by InfluxDB outage.

#### Scenario: InfluxDB unavailable
- **WHEN** the InfluxDB write fails
- **THEN** it SHALL log the error and continue processing events without crashing

---

### CC-4: Alert Notifier

### Requirement: Alert deduplication
The alert_notifier SHALL suppress re-notification for the same `alert_id` within a configurable `cooldown_s` (default 60 s) per alert ID.

#### Scenario: Repeated alert suppressed
- **GIVEN** `CAM_DISCONNECTED` was notified 30 seconds ago and cooldown is 60 s
- **WHEN** another `evt.alert` contains `CAM_DISCONNECTED`
- **THEN** it SHALL NOT dispatch a notification for that alert

#### Scenario: Cooldown expired
- **GIVEN** `CAM_DISCONNECTED` was notified 65 seconds ago and cooldown is 60 s
- **WHEN** another `evt.alert` contains `CAM_DISCONNECTED`
- **THEN** it SHALL dispatch a notification

### Requirement: Notification channel dispatch
The alert_notifier SHALL support configurable notification channels defined in `notifier.yaml`: terminal (log format), sound (WAV file), desktop (plyer), Slack (webhook), email (SMTP). Each channel has `enabled`, `min_severity`, and channel-specific settings.

#### Scenario: Terminal notification
- **WHEN** a WARN-severity alert triggers and the terminal channel is enabled with `min_severity: WARN`
- **THEN** it SHALL print the alert in the configured format to stdout

#### Scenario: Sound notification for ERROR
- **WHEN** an ERROR-severity alert triggers and the sound channel is enabled with `min_severity: ERROR`
- **THEN** it SHALL play the configured `sound_file`

#### Scenario: Channel disabled
- **WHEN** the Slack channel is configured with `enabled: false`
- **THEN** it SHALL NOT send any Slack notifications regardless of severity

---

### CC-5: Dashboard Server

### Requirement: FastAPI REST API
The dashboard_server SHALL expose REST endpoints: `POST /api/command` (submit a RoverCommand), `GET /api/command/history` (paginated command log from InfluxDB), `GET /api/health/history` (time-range telemetry query), `GET /api/status` (control_center process health: broker connected, cache age).

#### Scenario: Command submitted via REST
- **WHEN** an operator POSTs to `/api/command` with `{"type": "CMD_ARM", "issued_by": "bob"}`
- **THEN** it SHALL pass the command to the cmd_gateway and return `{"cmd_id": "<uuid>", "status": "ACCEPTED"}` or `{"status": "REJECTED", "reason": "..."}`

#### Scenario: Cache stale blocks commands
- **WHEN** the telemetry cache is stale and a command (other than CMD_ESTOP) is submitted
- **THEN** it SHALL return HTTP 503 with `{"status": "UNAVAILABLE", "reason": "rover_health cache stale"}`

### Requirement: WebSocket live telemetry push
The dashboard_server SHALL push telemetry_cache snapshots to all connected WebSocket clients at 2 Hz on `/ws/health`, alert events on `/ws/alerts`, and command ACK events on `/ws/cmd_ack`.

#### Scenario: Health pushed to UI
- **WHEN** a WebSocket client connects to `/ws/health`
- **THEN** it SHALL receive JSON-serialized RoverHealth snapshots every 500 ms

#### Scenario: Alert pushed to UI
- **WHEN** an alert event arrives on the bus
- **THEN** it SHALL broadcast the alert to all `/ws/alerts` WebSocket clients

### Requirement: React SPA serving
The dashboard_server SHALL serve the React SPA static build from a `dist/` directory via FastAPI's `StaticFiles` mount. The SPA SHALL display panels for: connection status, overall health, battery, camera, PX4, Jetson, SLAM latency, active alerts, command panel (goal/arm/disarm/mode/e-stop), and command log.

#### Scenario: Dashboard loaded
- **WHEN** an operator navigates to `http://localhost:8080`
- **THEN** the FastAPI server SHALL serve the React SPA index.html

---

### CC-6: Command Gateway

### Requirement: Safety gate validation
The cmd_gateway SHALL validate every command against the telemetry_cache via a safety gate before publishing to MQTT. The safety gate rules are defined per-CommandType in the command-system spec. CMD_ESTOP SHALL always bypass the safety gate.

#### Scenario: Command passes safety gate
- **GIVEN** the telemetry cache shows `px4.armed=true`, `px4.nav_state=OFFBOARD`, `slam_latency_ms=50`, `overall_health="OK"`
- **WHEN** CMD_NAV_GOAL is submitted
- **THEN** the safety gate SHALL pass and the command SHALL be serialized and published

#### Scenario: Command blocked by safety gate
- **GIVEN** the telemetry cache shows `overall_health="ERROR"`
- **WHEN** CMD_NAV_GOAL is submitted
- **THEN** the safety gate SHALL reject with reason "overall_health is ERROR" and return REJECTED without publishing to MQTT

### Requirement: Command serialization and publishing
The cmd_gateway SHALL assign a UUID v4 `cmd_id`, serialize the command as `RoverCommand` Protobuf, publish to the appropriate `rover/cmd/*` MQTT topic with QoS 2, and log the command to InfluxDB via cmd_logger immediately.

#### Scenario: Command published
- **WHEN** a command passes the safety gate
- **THEN** it SHALL assign a UUID, serialize to Protobuf, publish to MQTT, and write to `rover_commands` in InfluxDB with status "PENDING"

---

### CC-7: ACK Tracker

### Requirement: Command-ACK matching
The ack_tracker SHALL maintain a dict of pending commands keyed by `cmd_id` with `asyncio.Future` values. When a `CommandAck` arrives via `evt.cmd_ack`, it SHALL resolve the matching future, update the InfluxDB command log, and broadcast the ACK to `/ws/cmd_ack`.

#### Scenario: ACK received
- **GIVEN** a command with `cmd_id="abc-123"` is pending
- **WHEN** a CommandAck with `cmd_id="abc-123"` and `status=ACK_COMPLETED` arrives
- **THEN** it SHALL resolve the pending future, write the ACK to InfluxDB with round_trip_ms, and broadcast to WebSocket

### Requirement: ACK timeout
The ack_tracker SHALL enforce a configurable `ack_timeout_s` (default 5 s). If no ACK_ACCEPTED or higher is received within the timeout, the command SHALL be marked ACK_FAILED with detail "ACK timeout".

#### Scenario: Command times out
- **GIVEN** a command with `cmd_id="abc-123"` was sent 6 seconds ago
- **WHEN** no ACK has been received and `ack_timeout_s` is 5
- **THEN** it SHALL mark the command as ACK_FAILED, log to InfluxDB, and notify the dashboard via `/ws/cmd_ack`

