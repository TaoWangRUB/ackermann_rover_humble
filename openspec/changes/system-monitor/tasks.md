## 1. Package Skeleton

- [ ] 1.1 Create `src/rover_monitor/` directory with `src/`, `include/rover_monitor/`, `launch/`, `config/`, `proto/`, `msg/` subdirectories
- [ ] 1.2 Create `CMakeLists.txt` with ament_cmake, rclcpp_components, sensor_msgs, px4_msgs, tf2_ros, protobuf, paho-mqtt-cpp dependencies and component registration macros
- [ ] 1.3 Create `package.xml` with all build/exec dependencies

## 2. Custom ROS 2 Messages

- [ ] 2.1 Define `msg/CamStatus.msg` (camera_id, connected, frame_delta_ms, depth_fps, depth_quality_sampled, imu_active, error_code, error_msg, timestamp)
- [ ] 2.2 Define `msg/Px4Status.msg` (connected, armed, nav_state, nav_state_label, battery_voltage_v, battery_current_a, battery_remaining_pct, heartbeat_age_ms, error_code, error_msg, timestamp)
- [ ] 2.3 Define `msg/JetsonStatus.msg` (cpu_usage_pct[], gpu_usage_pct, ram_used_mb, ram_total_mb, swap_used_mb, disk_free_gb, temp_cpu_c, temp_gpu_c, temp_board_c, is_thermal_throttled, is_power_throttled, power_mode, wifi_signal_dbm, uptime_s, error_code, error_msg, timestamp)
- [ ] 2.4 Define `msg/RoverHealth.msg` (seq, timestamp, camera CamStatus, px4 Px4Status, jetson JetsonStatus, slam_latency_ms, overall_health, active_alerts[])

## 3. Protobuf Schema

- [ ] 3.1 Create `proto/rover_health.proto` with CamStatus, Px4Status, JetsonStatus, RoverHealth messages
- [ ] 3.2 Add `protobuf_generate_cpp()` to CMakeLists.txt for code generation

## 4. Configuration Files

- [ ] 4.1 Create `config/rover_monitor.yaml` — master config with container executor, aggregator rates/timeouts, probe settings, alert thresholds
- [ ] 4.2 Create `config/alert_rules.yaml` — named alert rules (CAM_STUTTER, CAM_DISCONNECTED, PX4_HEARTBEAT_LOST, PX4_BATTERY_CRITICAL, HW_THROTTLE_THERMAL, HW_THROTTLE_POWER, SLAM_LATE, NET_DROP, etc.)
- [ ] 4.3 Create `config/publisher.yaml` — MQTT broker host/port, client_id, QoS levels, reconnect_delay_s

## 5. Camera Probe

- [ ] 5.1 Create `include/rover_monitor/cam_probe.hpp` — component class declaration with MutuallyExclusiveCallbackGroup
- [ ] 5.2 Implement `src/cam_probe.cpp` — subscribe to `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, `/camera/imu`; frame delta measurement; depth quality 1-in-10 sampling; depth FPS rolling average; IMU liveness; rs2::context device list check; publish CamStatus on `/monitor/cam` via unique_ptr
- [ ] 5.3 Register as rclcpp_component in CMakeLists.txt

## 6. PX4 Probe

- [ ] 6.1 Create `include/rover_monitor/px4_probe.hpp` — component class declaration
- [ ] 6.2 Implement `src/px4_probe.cpp` — subscribe to `/fmu/out/vehicle_status`, `/fmu/out/battery_status`, `/fmu/out/vehicle_global_position`; heartbeat proxy via timestamp delta; battery state tracking; arming/nav state; XRCE-DDS disconnect detection; publish Px4Status on `/monitor/px4` via unique_ptr
- [ ] 6.3 Register as rclcpp_component in CMakeLists.txt

## 7. Jetson Probe

- [ ] 7.1 Create `include/rover_monitor/jetson_probe.hpp` — component class declaration
- [ ] 7.2 Implement `src/jetson_probe.cpp` — 0.5 Hz timer; /proc/stat delta for per-core CPU; /sys/devices/gpu.0/load for GPU; thermal_zone reads for temperatures; cooling_device cur_state for thermal throttle; bpmp/debug/clk for power throttle; /proc/meminfo for RAM; statvfs for disk; /proc/net/wireless for WiFi; nvpmodel -q cached at startup; publish JetsonStatus on `/monitor/jetson` via unique_ptr
- [ ] 7.3 Register as rclcpp_component in CMakeLists.txt

## 8. Monitor Aggregator + Alert Engine

- [ ] 8.1 Create `include/rover_monitor/aggregator.hpp` — component class with std::optional<T> caches and tf2 buffer
- [ ] 8.2 Implement `src/aggregator.cpp` — subscribe to `/monitor/cam`, `/monitor/px4`, `/monitor/jetson`; 2 Hz timer-based merge with stale timeout detection; SLAM latency via tf2 lookupTransform("map", "odom") stamp delta; inline alert engine evaluating rules from config; derive overall_health; publish RoverHealth on `/monitor/health` via unique_ptr
- [ ] 8.3 Register as rclcpp_component in CMakeLists.txt

## 9. Telemetry Publisher

- [ ] 9.1 Create `include/rover_monitor/telemetry_publisher.hpp` — component class with paho MQTT client
- [ ] 9.2 Implement `src/telemetry_publisher.cpp` — subscribe to `/monitor/health`; serialize RoverHealth to Protobuf; publish to MQTT topics (rover/health/* QoS 0, rover/alerts QoS 1); auto-reconnect on broker disconnect
- [ ] 9.3 Register as rclcpp_component in CMakeLists.txt

## 10. Launch File

- [ ] 10.1 Create `launch/monitor.launch.py` — launch component_container (single-threaded), load all 5 components, pass config files, support use_sim_time argument

## 11. Integration

- [ ] 11.1 Add optional `rover_monitor` launch argument and IncludeLaunchDescription to `src/robot_bringup/launch/robot_bringup.launch.py`

## 12. Build and Test

- [ ] 12.1 Run `colcon build --packages-select rover_monitor` and verify clean build with message generation + protobuf codegen
- [ ] 12.2 Launch `ros2 launch rover_monitor monitor.launch.py` and verify `/monitor/cam`, `/monitor/px4`, `/monitor/jetson`, `/monitor/health` topics publish
- [ ] 12.3 Verify MQTT telemetry arrives at host broker (`mosquitto_sub -t "rover/health/#"`)
- [ ] 12.4 Test alert triggering by simulating degraded conditions (e.g., kill camera node, check CAM_DISCONNECTED alert)
