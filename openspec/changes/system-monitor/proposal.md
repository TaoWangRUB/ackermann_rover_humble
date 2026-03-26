## Why

The rover has no runtime health monitoring. The only safety mechanism is a minimal watchdog in `src/safety/` that reacts to `/fault` with a stop command. There is zero visibility into camera frame drops, PX4 flight controller state, Jetson thermal/power throttling, or SLAM staleness. On the resource-constrained Xavier NX running SLAM + Nav2 + RealSense + PX4, silent degradation (thermal throttle, heartbeat loss, depth quality collapse) can cause mission failure without warning. Operators on the host machine have no telemetry dashboard or alert system.

## What Changes

- Add a new `rover_monitor` ROS 2 C++ package (`src/rover_monitor/`) using **rclcpp_components** in a single `component_container` process
- Implement three domain-specific probes: **cam_probe** (RealSense health), **px4_probe** (flight controller via Micro XRCE-DDS), **jetson_probe** (sysfs/procfs direct reads)
- Implement a **monitor_aggregator** with timer-based latest-value merge (2 Hz), SLAM latency computation, and inline alert engine
- Implement a **telemetry_publisher** streaming Protobuf-serialized `RoverHealth` over MQTT to a host machine (Mosquitto → Grafana + InfluxDB + AlertNotifier)
- All components use **intra-process shared-memory pub/sub** (zero-copy) with `MutuallyExclusiveCallbackGroup` per component for deterministic scheduling
- Define custom ROS 2 messages (CamStatus, Px4Status, JetsonStatus, RoverHealth) and a Protobuf schema (`rover_health.proto`)
- Provide configurable alert rules (YAML) with named IDs and severity levels (OK/WARN/ERROR)

## Capabilities

### New Capabilities

- `camera-monitoring`: RealSense frame delta detection, depth fill ratio (sampled 1-in-10), IMU liveness, device disconnect
- `px4-monitoring`: PX4 flight controller state via XRCE-DDS uORB topics — battery, arming, nav state, heartbeat proxy
- `jetson-monitoring`: Xavier NX sysfs/procfs direct reads — per-core CPU, GPU, temperatures, thermal/power throttle split, WiFi signal, disk, RAM
- `health-aggregation`: Timer-based merge of all probes into unified RoverHealth, SLAM latency from `/tf` map→odom stamp delta, inline alert engine with configurable rules
- `telemetry-publishing`: Protobuf serialization over MQTT (QoS 0 routine, QoS 1 alerts) to host machine broker

### Modified Capabilities

<!-- No existing capabilities are being modified at the spec level -->

## Impact

- **New package**: `src/rover_monitor/` (C++ components, headers, launch, config, proto, CMakeLists.txt, package.xml)
- **Dependencies**: rclcpp_components, sensor_msgs, px4_msgs, tf2_ros, paho-mqtt-cpp, protobuf
- **Infrastructure**: Host machine needs Mosquitto broker, InfluxDB, Grafana, AlertNotifier
- **Prerequisite**: MicroXRCEAgent must be running before launch (already required by px4_bringup)
- **Integration**: New `monitor.launch.py` (standalone); optional include from `robot_bringup.launch.py`
