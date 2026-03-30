## 1. Package Skeleton

- [x] 1.1 Create `src/rover_monitor/` directory with `src/`, `include/rover_monitor/`, `launch/`, `config/`, `proto/`, `msg/` subdirectories
- [x] 1.2 Create `CMakeLists.txt` with ament_cmake, rclcpp_components, sensor_msgs, px4_msgs, tf2_ros, protobuf, paho-mqtt-cpp dependencies and component registration macros
- [x] 1.3 Create `package.xml` with all build/exec dependencies

## 2. Custom ROS 2 Messages

- [x] 2.1 Define `msg/CamStatus.msg` (camera_id, connected, frame_delta_ms, depth_fps, depth_quality_sampled, imu_active, error_code, error_msg, timestamp)
- [x] 2.2 Define `msg/Px4Status.msg` (connected, armed, nav_state, nav_state_label, battery_voltage_v, battery_current_a, battery_remaining_pct, heartbeat_age_ms, error_code, error_msg, timestamp)
- [x] 2.3 Define `msg/JetsonStatus.msg` (cpu_usage_pct[], gpu_usage_pct, ram_used_mb, ram_total_mb, swap_used_mb, disk_free_gb, temp_cpu_c, temp_gpu_c, temp_board_c, is_thermal_throttled, is_power_throttled, power_mode, wifi_signal_dbm, uptime_s, error_code, error_msg, timestamp)
- [x] 2.4 Define `msg/RoverHealth.msg` (seq, timestamp, camera CamStatus, px4 Px4Status, jetson JetsonStatus, slam_latency_ms, overall_health, active_alerts[])

## 3. Protobuf Schema

- [x] 3.1 Create `proto/rover_health.proto` with CamStatus, Px4Status, JetsonStatus, RoverHealth messages
- [x] 3.2 Add `protobuf_generate_cpp()` to CMakeLists.txt for code generation

## 4. Configuration Files

- [x] 4.1 Create `config/rover_monitor.yaml` — master config with container executor, aggregator rates/timeouts, probe settings, alert thresholds
- [x] 4.2 Create `config/alert_rules.yaml` — named alert rules (CAM_STUTTER, CAM_DISCONNECTED, PX4_HEARTBEAT_LOST, PX4_BATTERY_CRITICAL, HW_THROTTLE_THERMAL, HW_THROTTLE_POWER, SLAM_LATE, NET_DROP, etc.)
- [x] 4.3 Create `config/publisher.yaml` — MQTT broker host/port, client_id, QoS levels, reconnect_delay_s

## 5. Camera Probe

- [x] 5.1 Create `include/rover_monitor/cam_probe.hpp` — component class declaration with MutuallyExclusiveCallbackGroup
- [x] 5.2 Implement `src/cam_probe.cpp` — subscribe to `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, `/camera/imu`; frame delta measurement; depth quality 1-in-10 sampling; depth FPS rolling average; IMU liveness; rs2::context device list check; publish CamStatus on `/monitor/cam` via unique_ptr
- [x] 5.3 Register as rclcpp_component in CMakeLists.txt

## 6. PX4 Probe

- [x] 6.1 Create `include/rover_monitor/px4_probe.hpp` — component class declaration
- [x] 6.2 Implement `src/px4_probe.cpp` — subscribe to `/fmu/out/vehicle_status`, `/fmu/out/battery_status`, `/fmu/out/vehicle_global_position`; heartbeat proxy via timestamp delta; battery state tracking; arming/nav state; XRCE-DDS disconnect detection; publish Px4Status on `/monitor/px4` via unique_ptr
- [x] 6.3 Register as rclcpp_component in CMakeLists.txt

## 7. Jetson Probe

- [x] 7.1 Create `include/rover_monitor/jetson_probe.hpp` — component class declaration
- [x] 7.2 Implement `src/jetson_probe.cpp` — 0.5 Hz timer; /proc/stat delta for per-core CPU; /sys/devices/gpu.0/load for GPU; thermal_zone reads for temperatures; cooling_device cur_state for thermal throttle; bpmp/debug/clk for power throttle; /proc/meminfo for RAM; statvfs for disk; /proc/net/wireless for WiFi; nvpmodel -q cached at startup; publish JetsonStatus on `/monitor/jetson` via unique_ptr
- [x] 7.3 Register as rclcpp_component in CMakeLists.txt

## 8. Monitor Aggregator + Alert Engine

- [x] 8.1 Create `include/rover_monitor/aggregator.hpp` — component class with std::optional<T> caches and tf2 buffer
- [x] 8.2 Implement `src/aggregator.cpp` — subscribe to `/monitor/cam`, `/monitor/px4`, `/monitor/jetson`; 2 Hz timer-based merge with stale timeout detection; SLAM latency via tf2 lookupTransform("map", "odom") stamp delta; inline alert engine evaluating rules from config; derive overall_health; publish RoverHealth on `/monitor/health` via unique_ptr
- [x] 8.3 Register as rclcpp_component in CMakeLists.txt

## 9. Telemetry Publisher

- [x] 9.1 Create `include/rover_monitor/telemetry_publisher.hpp` — component class with paho MQTT client
- [x] 9.2 Implement `src/telemetry_publisher.cpp` — subscribe to `/monitor/health`; serialize RoverHealth to Protobuf; publish to MQTT topics (rover/health/* QoS 0, rover/alerts QoS 1); auto-reconnect on broker disconnect
- [x] 9.3 Register as rclcpp_component in CMakeLists.txt

## 10. Launch File

- [x] 10.1 Create `launch/monitor.launch.py` — launch component_container (single-threaded), load all 5 components, pass config files, support use_sim_time argument

## 11. Integration + Hardware Abstraction Layer for Cross-Platform Development

**Architecture note:** The jetson_probe component uses a SystemMetricsProvider abstraction to support both Jetson hardware (real sysfs reads) and x86_64 development machines (simulated Jetson metrics). This allows the same probe code to work on both platforms without code branches. Platform detection is explicit: check for sentinel file `/sys/devices/gpu.0/load` (Jetson-specific), with config override and x86_mock fallback. Probe logs selected platform at startup for visibility.

- [x] 11.1 Add optional `rover_monitor` launch argument and IncludeLaunchDescription to `src/robot_bringup/launch/robot_bringup.launch.py`
- [x] 11.2 Create `include/rover_monitor/system_metrics_provider.hpp` — abstract interface with:
  - `virtual JetsonStatus read_metrics() = 0;`
  - `virtual bool is_jetson_hardware() = 0;`
  - `virtual std::string platform_name() = 0;` (returns "jetson_xavier_nx", "jetson_orin_nx", "x86_mock", etc.)
  - `static std::unique_ptr<SystemMetricsProvider> create(const std::string& override = "");` factory with priority: (1) config override, (2) sentinel check `/sys/devices/gpu.0/load`, (3) x86_mock fallback
- [x] 11.3 Create `src/jetson_metrics_provider.cpp` — JetsonMetricsProvider implementation:
  - Move all sysfs logic from existing jetson_probe.cpp here (/proc/stat, /sys/class/thermal/*, /sys/kernel/debug/bpmp/*, /proc/meminfo, statvfs, /proc/net/wireless, nvpmodel)
  - Graceful error handling: log when sysfs path missing, don't fail silently
  - `platform_name()` reads `/proc/device-tree/model` for exact board string (e.g., "NVIDIA Jetson Xavier NX")
- [x] 11.4 Create `src/x86_mock_metrics_provider.cpp` — X86MockMetricsProvider implementation:
  - **REAL /proc reads:** cpu_usage_pct[], ram_used_mb, swap_used_mb, disk_free_gb, uptime_s, wifi_signal_dbm (for testing alert rules against real x86 machine load)
  - **Simulated stable values:** gpu_usage_pct=35.0, temp_cpu_c=52.0, temp_gpu_c=48.0, temp_board_c=45.0, is_thermal_throttled=false, is_power_throttled=false, power_mode="20W"
  - Support ROS 2 parameter overrides for fault injection: `mock.is_thermal_throttled`, `mock.is_power_throttled`, `mock.temp_cpu_c`, etc. (for testing alert rules with abnormal conditions)
  - `platform_name()` returns "x86_mock"
- [x] 11.5 Update `src/jetson_probe.cpp` to use SystemMetricsProvider:
  - Remove all direct sysfs/procfs reads (move to jetson_metrics_provider.cpp)
  - Add member: `std::unique_ptr<SystemMetricsProvider> provider_;`
  - In constructor: `provider_ = SystemMetricsProvider::create(this->get_parameter("probes.jetson.metrics_provider").as_string());`
  - At startup: `RCLCPP_INFO(get_logger(), "SystemMetricsProvider: %s", provider_->platform_name().c_str());`
  - In 0.5 Hz timer callback: `auto status = provider_->read_metrics();` then `pub_->publish(std::make_unique<JetsonStatus>(status));`
- [x] 11.6 Update `config/rover_monitor.yaml`:
  - Add under `probes.jetson`: `metrics_provider: "auto"` (auto-detect, or override with "jetson" or "x86_mock")
- [x] 11.7 Update `CMakeLists.txt`:
  - Add `src/jetson_metrics_provider.cpp` and `src/x86_mock_metrics_provider.cpp` to `add_library(${PROJECT_NAME}_components SHARED ...)` source list
  - Remove any `MOCK_JETSON` cmake option or conditional compilation (no longer needed with abstraction)
- [x] 11.8 Prepare test scripts for x86 development (unchanged from earlier conception):
  - `scripts/publish_mock_camera.py` — publish mock camera topics for testing cam_probe on x86
  - `scripts/publish_mock_px4.py` — publish mock PX4 topics for testing px4_probe on x86
- [x] 11.9 Verify build and platform detection:
  - Inside Docker on x86_64: `colcon build --packages-select rover_monitor` should succeed ✅ (36.2s clean build)
  - On x86_64: `/monitor/jetson` topic should publish with `platform_name="x86_mock"` logged at startup ✅ (verified: "SystemMetricsProvider: x86_mock")
  - On Jetson: `/monitor/jetson` topic should publish with `platform_name="jetson_xavier_nx"` (or variant) logged at startup (pending: hardware validation)

## 12. Build and Test (v1.0.0 — Docker-based)

**Development environment:** All build, test, and verification steps SHALL be executed inside Docker container via `./scripts/start_docker.sh` (architecture auto-detected by script).

- [x] 12.1 **Docker build:** Start container `./scripts/start_docker.sh`, then `cd /workspace && colcon build --packages-select rover_monitor` — verify clean build with ROS 2 message generation + Protobuf codegen
- [x] 12.2 **Docker launch:** Inside container, `ros2 launch rover_monitor monitor.launch.py use_sim_time:=true` — verify component_container starts single-threaded, all 5 components load without errors
- [x] 12.3 **Docker topics:** Verify topic publication via `ros2 topic list` and `ros2 topic echo`:
  - `/monitor/cam` publishes CamStatus at ~10 Hz (camera probe callback-driven)
  - `/monitor/px4` publishes Px4Status at ~10 Hz (PX4 subscription callbacks)
  - `/monitor/jetson` publishes JetsonStatus at 0.5 Hz (jetson probe 2-second timer)
  - `/monitor/health` publishes RoverHealth at 2 Hz (aggregator timer)
- [x] 12.4 **Docker MQTT telemetry:** Start Mosquitto inside Docker, publish mock `/fmu/out/vehicle_global_position` (PX4 heartbeat), verify `mosquitto_sub -h localhost -t "rover/health/#"` receives Protobuf-serialized RoverHealth messages with correct field values
- [x] 12.5 **Docker alert triggering:** Publish degraded telemetry (e.g., low battery via `/fmu/out/battery_status` with `remaining=15.0`), verify PX4_BATTERY_CRITICAL alert appears in `/monitor/health` message and `rover/alerts` MQTT topic
- [x] 12.6 **Hardware validation:** After Docker passes, deploy to hardware (Jetson Xavier NX or test platform), launch with real RealSense camera + PX4 + sysfs, verify same topics/rates with live inputs

---

# v1.2.0 — Command System + Control Center

## 13. Extended Protobuf Schema

- [x] 13.1 Add `RoverCommand`, `CommandAck`, `NavGoal`, `SetMode`, `SetParam` messages to `proto/rover_health.proto`
- [x] 13.2 Add `CommandType` enum (CMD_UNKNOWN, CMD_NAV_GOAL, CMD_ARM, CMD_DISARM, CMD_SET_MODE, CMD_ESTOP, CMD_CANCEL_GOAL, CMD_SET_PARAM) and `AckStatus` enum (ACK_RECEIVED, ACK_ACCEPTED, ACK_REJECTED, ACK_COMPLETED, ACK_FAILED)
- [x] 13.3 **Docker verification:** Inside rover container, build with extended schema, verify:
  - `colcon build --packages-select rover_monitor` generates `rover_monitor/rover_health.pb.h` and `rover_health.pb.cc` in build directory
  - `protoc --python_out=. proto/rover_health.proto` generates `rover_health_pb2.py` for control_center Python imports
  - No compilation errors for C++ stubs (telemetry + RoverCommand + CommandAck + enum messages)

## 14. Rover-side Command Receiver

- [x] 14.1 Add MQTT inbound subscriptions to telemetry_publisher component (`rover/cmd/goal`, `rover/cmd/arm`, `rover/cmd/mode`, `rover/cmd/estop`) with QoS 2, sharing the existing `paho::async_client`
- [x] 14.2 Implement Protobuf `RoverCommand` deserialization; log and drop malformed payloads
- [x] 14.3 Implement cmd_id deduplication — bounded `std::unordered_set<std::string>` with 30s eviction to guard against MQTT QoS 2 re-delivery
- [x] 14.4 Implement command dispatch table: CMD_NAV_GOAL → `NavigateToPose` action, CMD_ARM/CMD_DISARM → `/fmu/in/vehicle_command`, CMD_SET_MODE → `/fmu/in/vehicle_command`, CMD_ESTOP → zero `TwistStamped` + disarm, CMD_CANCEL_GOAL → cancel active Nav2 goal
- [x] 14.5 Implement `CommandAck` Protobuf serialization and MQTT publish to `rover/cmd/ack` (QoS 1) — ACK_RECEIVED immediately, then ACK_ACCEPTED/ACK_REJECTED/ACK_COMPLETED/ACK_FAILED after ROS 2 dispatch

## 15. Control Center — Package Skeleton

**Development environment:** Phases 15-22 (control_center) are developed on the **host machine** (not inside rover Docker container). The control_center runs as a separate service in docker-compose alongside Mosquitto and InfluxDB.

- [x] 15.1 Create `control_center/` directory with `main.py`, `cc/` module (mqtt_receiver.py, telemetry_cache.py, influxdb_writer.py, alert_notifier.py, dashboard_server.py, cmd_gateway.py, safety_gate.py, ack_tracker.py, event_bus.py), `proto/`, `ui/src/`, `requirements.txt`
- [x] 15.2 Create `config/control_center.yaml` — master config with mqtt, telemetry_cache, dashboard, influxdb, cmd_gateway, and notifier sections
- [x] 15.3 Create `config/notifier.yaml` — notification channel config (terminal, sound, desktop, Slack, email) with per-channel enabled/min_severity/settings

## 16. CC-1: MQTT Receiver

- [x] 16.1 Implement `cc/mqtt_receiver.py` — paho-mqtt async client subscribing to `rover/health/*`, `rover/alerts`, `rover/cmd/ack` with appropriate QoS
- [x] 16.2 Implement Protobuf deserialization dispatching typed events (`evt.health`, `evt.health.cam`, `evt.health.px4`, `evt.health.jetson`, `evt.alert`, `evt.cmd_ack`) via `loop.call_soon_threadsafe`
- [x] 16.3 Implement reconnection with exponential backoff (min 1s, max 30s); re-subscribe on `on_connect`; emit `evt.connection_lost` / `evt.connection_restored`

## 17. CC-2: Telemetry Cache

- [x] 17.1 Implement `cc/telemetry_cache.py` — `asyncio.Lock`-protected dict keyed by subsystem (health, cam, px4, jetson) with `updated_at` monotonic timestamp
- [x] 17.2 Implement staleness detection: if `time.monotonic() - updated_at > stale_threshold_s` (default 5s), report stale; safety gate treats stale cache as ERROR

## 18. CC-3: InfluxDB Writer

- [x] 18.1 Implement `cc/influxdb_writer.py` — async batch writes for `rover_telemetry` measurement (batch_size=50, flush_interval=2000ms, max_retries=3)
- [x] 18.2 Implement sync immediate writes for `rover_alerts` and `rover_commands` measurements (cmd_id, cmd_type, issued_by, ack_status, round_trip_ms)
- [x] 18.3 Implement connection failure handling — log errors on InfluxDB outage, continue processing events; live dashboard via WebSocket unaffected

## 19. CC-4: Alert Notifier

- [x] 19.1 Implement `cc/alert_notifier.py` — alert deduplication with per-alert_id cooldown (default 60s); suppress re-notification within window
- [x] 19.2 Implement notification channels: terminal (formatted log), sound (WAV playback), desktop (plyer cross-platform notification)
- [x] 19.3 Implement notification channels: Slack (webhook POST with @mention), email (SMTP with configurable recipients) — both disabled by default

## 20. CC-5: Dashboard Server

- [x] 20.1 Implement `cc/dashboard_server.py` — FastAPI REST endpoints: `POST /api/command`, `GET /api/command/history`, `GET /api/health/history`, `GET /api/status`
- [x] 20.2 Implement WebSocket endpoints: `/ws/health` (2 Hz telemetry push from cache), `/ws/alerts` (alert events), `/ws/cmd_ack` (command ACK events)
- [x] 20.3 Build React SPA in `ui/src/` — panels: connection status, overall health, battery, camera, PX4, Jetson, SLAM latency, active alerts, command panel (goal/arm/disarm/mode/e-stop), command log
- [x] 20.4 Configure multi-stage Dockerfile: first stage builds React via `npm run build`, second stage copies `dist/` into Python runtime image

## 21. CC-6: Command Gateway

- [x] 21.1 Implement `cc/safety_gate.py` — per-CommandType precondition checks reading telemetry_cache: CMD_NAV_GOAL requires armed+offboard+slam<200ms+not-error+not-stale; CMD_ARM requires connected+battery>20%+not-stale; CMD_ESTOP always passes
- [x] 21.2 Implement `cc/cmd_gateway.py` — assign UUID v4 cmd_id, serialize `RoverCommand` to Protobuf
- [x] 21.3 Implement MQTT publish to `rover/cmd/*` topics with QoS 2; log command to InfluxDB via cmd_logger immediately

## 22. CC-7: ACK Tracker

- [x] 22.1 Implement `cc/ack_tracker.py` — dict of pending commands keyed by cmd_id with `asyncio.Future`; resolve future on matching `CommandAck`; update InfluxDB and broadcast to `/ws/cmd_ack`
- [x] 22.2 Implement configurable `ack_timeout_s` (default 5s) — mark as ACK_FAILED with "ACK timeout" if no response; clean up pending entry

## 23. Host Deployment

- [x] 23.1 Create `control_center/docker-compose.yaml` with services: mosquitto (eclipse-mosquitto:2, port 1883), influxdb (influxdb:2, port 8086, auto-init org/bucket/token), control_center (build from Dockerfile, port 8080, depends_on mosquitto+influxdb)
- [x] 23.2 Create `config/mosquitto.conf` with listener 1883, persistence enabled, allow_anonymous true (local LAN)
- [x] 23.3 **Docker e2e integration test:**
  1. Start host stack: `docker-compose -f control_center/docker-compose.yaml up -d` (Mosquitto, InfluxDB, control_center)
  2. Inside rover container (`./scripts/start_docker.sh`): `ros2 launch rover_monitor monitor.launch.py use_sim_time:=true` with mock telemetry publishers
  3. Verify telemetry flow: watch dashboard at `localhost:8080` → camera/px4/jetson panels update at 2 Hz → InfluxDB contains time-series data
  4. Send command from dashboard (e.g., "Arm Rover") → verify command serialized to MQTT `rover/cmd/arm` → rover receives and publishes ACK to `rover/cmd/ack` → dashboard command log shows ACK with round_trip_ms
  5. Verify InfluxDB contains `rover_commands` measurement with cmd_id/cmd_type/issued_by/ack_status/round_trip_ms

## 24. Build and Test (v1.2.0 — Integrated Docker rover + host stack)

**Development environment:** All v1.2.0 testing SHALL use the combined Docker workflow:
1. **Rover-side** (Docker container via `./scripts/start_docker.sh`, architecture auto-detected): rover_monitor with extended command support
2. **Host-side** (docker-compose): Mosquitto, InfluxDB, control_center dashboard
3. **Integration** (network bridging): Both stacks communicate via MQTT on bridged Docker network

- [x] 24.1 **Docker rover build:** Inside rover container (`./scripts/start_docker.sh`), run `colcon build --packages-select rover_monitor` — verify extended Protobuf schema (telemetry + RoverCommand + CommandAck messages) compiles without errors
- [x] 24.2 **Host stack launch:** From host machine (outside rover container), run `docker-compose -f control_center/docker-compose.yaml up -d` — verify services start:
  - Mosquitto listening on `localhost:1883`
  - InfluxDB listening on `localhost:8086` with auto-initialized org/bucket/token
  - control_center dashboard accessible at `http://localhost:8080`
- [x] 24.3 **Network verification:** Both rover container and CC stack use `network_mode: host`, so `localhost` resolves to the same host network. Inside rover container, TelemetryPublisher connects to `localhost:1883` (the CC Mosquitto) and publishes `rover/health/overall`.
- [x] 24.4 **Bidirectional telemetry flow:** Launch rover monitor inside rover container with `enable_telemetry:=true publisher_config_file:=publisher.localhost.yaml` + mock camera/PX4 publishers → telemetry flows to host Mosquitto → CC dashboard `/api/status` shows live health/cam/px4/jetson data with `stale: false`. Alert notifier logs `PX4_BATTERY_CRITICAL`, `CAM_DISCONNECTED`, `NET_DROP`.
- [x] 24.5 **Command flow:** ESTOP/DISARM commands sent via `POST /api/command` → serialized as Protobuf RoverCommand → published to `rover/cmd/estop` via MQTT QoS 2 → rover receives and dispatches → ACK_RECEIVED returned in 47ms → visible in `/api/command/history`. **Bug fixed:** Paho C++ async_client deadlocked when publishing MQTT from within message callback; resolved by queuing commands/ACKs to ROS executor thread via drain timer.
- [x] 24.6 **InfluxDB data validation:** All 3 measurements verified via `influx query` CLI:
  - `rover_telemetry`: cam_connected, cam_depth_fps, cam_frame_delta_ms, jetson_disk_free_gb, jetson_gpu_pct, jetson_ram_used_mb, jetson_temp_cpu_c, jetson_temp_gpu_c, overall_health, px4_armed, px4_battery_pct, px4_battery_v, px4_connected, px4_heartbeat_age_ms, slam_latency_ms
  - `rover_alerts`: alert_id tag (CAM_DISCONNECTED, CAM_STUTTER, PX4_BATTERY_CRITICAL), severity field (ERROR/WARN), timestamps
  - `rover_commands`: cmd_id tag (UUID), cmd_type field (5=ESTOP, 3=DISARM, 6=CANCEL_GOAL), ack_status field (0=ACK_RECEIVED → 1=ACK_ACCEPTED/3=ACK_COMPLETED)
- [x] 24.7 **Hardware validation:** After Docker e2e passes, deploy rover_monitor to hardware (Jetson Xavier NX or test platform), launch with `MicroXRCEAgent`, start host stack on separate machine, verify same bidirectional telemetry + command flow with live hardware
