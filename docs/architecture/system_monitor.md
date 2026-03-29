---
title: System Monitor — Rover-Side Health Monitoring
status: Accepted
owner: Tao Wang
last_updated: 2026-03-28
doc_type: architecture
ros_distro: jazzy
---

# System Monitor (rover_monitor)

Real-time health monitoring for the Ackermann rover. The `rover_monitor` package runs on the rover (Jetson Xavier NX or x86_64 dev host) as composable ROS 2 C++ nodes inside a single-threaded container. It aggregates camera, flight-controller, and platform metrics into a unified health report, evaluates alert rules, and publishes telemetry over MQTT to the host-side [Control Center](control_center.md).

## 1. Overview

### 1.1 Purpose

| Concern | Solution |
|---|---|
| Camera health | Frame delta, depth FPS, IMU liveness, device enumeration |
| PX4 health | Heartbeat, armed state, nav mode, battery voltage/current/% |
| Platform health | CPU/GPU usage, RAM/swap, disk, thermals, throttling, Wi-Fi, uptime |
| SLAM latency | Measures `/rtabmap/odom` → aggregator delta |
| Alert detection | Rule-based engine with configurable thresholds |
| Remote telemetry | Protobuf serialization → MQTT to host broker |
| Remote commands | MQTT inbound → PX4 VehicleCommand / Nav2 goals |

### 1.2 Component Graph

```
┌─────────────┐   ┌──────────────┐   ┌──────────────┐
│  CamProbe   │   │   Px4Probe   │   │  JetsonProbe │
│  (10 Hz)    │   │   (2 Hz)     │   │  (0.5 Hz)    │
└──────┬──────┘   └──────┬───────┘   └──────┬───────┘
       │                 │                   │
       │  /monitor/cam   │  /monitor/px4     │  /monitor/jetson
       ▼                 ▼                   ▼
    ┌─────────────────────────────────────────────┐
    │              Aggregator (2 Hz)              │
    │         + AlertEngine (rule-based)          │
    ├─────────────────────────────────────────────┤
    │  publishes: /monitor/health (RoverHealth)   │
    └──────────────────┬──────────────────────────┘
                       │
                       ▼
              ┌────────────────┐
              │  Telemetry     │◄──── MQTT inbound (rover/cmd/*)
              │  Publisher     │
              │  (Protobuf +   │────► MQTT outbound (rover/health/overall)
              │   MQTT)        │────► MQTT outbound (rover/alerts)
              │                │────► MQTT outbound (rover/cmd/ack)
              └────────────────┘
```

All nodes run in a single `component_container` process via `rclcpp_components`.

## 2. Architecture

### 2.1 Package Structure

```
src/rover_monitor/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── monitor.launch.py          # Composable container + node loading
├── config/
│   ├── rover_monitor.yaml         # Master config (rates, timeouts, thresholds)
│   ├── alert_rules.yaml           # Named alert rules with severity/thresholds
│   ├── publisher.yaml             # MQTT config (production — Jetson IP)
│   └── publisher.localhost.yaml   # MQTT config (dev — localhost)
├── proto/
│   └── rover_health.proto         # Protobuf schema (shared with control_center)
├── msg/
│   ├── CamStatus.msg
│   ├── Px4Status.msg
│   ├── JetsonStatus.msg
│   └── RoverHealth.msg
├── include/rover_monitor/
│   ├── cam_probe.hpp
│   ├── px4_probe.hpp
│   ├── jetson_probe.hpp
│   ├── aggregator.hpp
│   ├── telemetry_publisher.hpp
│   └── system_metrics_provider.hpp
└── src/
    ├── cam_probe.cpp
    ├── px4_probe.cpp
    ├── jetson_probe.cpp
    ├── aggregator.cpp
    ├── telemetry_publisher.cpp
    ├── system_metrics_provider.cpp
    ├── jetson_metrics_provider.cpp   # Real sysfs/procfs reads (aarch64)
    └── x86_mock_metrics_provider.cpp # Synthetic data (x86_64 dev)
```

### 2.2 Node Details

#### CamProbe (10 Hz)

Subscribes to RealSense topics and evaluates camera health.

| Input Topic | Type | Purpose |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | Frame delta measurement |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | Depth FPS + quality sampling (1-in-10) |
| `/camera/imu` | `sensor_msgs/Imu` | IMU liveness |

Publishes `CamStatus` on `/monitor/cam`.

#### Px4Probe (2 Hz)

Subscribes to PX4 XRCE-DDS topics for flight controller status.

| Input Topic | Type | Purpose |
|---|---|---|
| `/fmu/out/vehicle_status` | `px4_msgs/VehicleStatus` | Armed state, nav mode, heartbeat |
| `/fmu/out/battery_status` | `px4_msgs/BatteryStatus` | Voltage, current, remaining % |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | Odometry liveness |

Publishes `Px4Status` on `/monitor/px4`.

#### JetsonProbe (0.5 Hz)

Reads platform metrics via `SystemMetricsProvider` abstraction.

| Platform | Provider | Source |
|---|---|---|
| `aarch64` (Jetson) | `JetsonMetricsProvider` | `/sys/devices/system/cpu/`, `/proc/meminfo`, `tegrastats`, sysfs thermal zones |
| `x86_64` (dev) | `X86MockMetricsProvider` | Synthetic data from `/proc/stat`, `/proc/meminfo` with randomized GPU/thermal values |

Platform is auto-detected at build time via CMake `CMAKE_SYSTEM_PROCESSOR` and at runtime via `uname -m`.

Publishes `JetsonStatus` on `/monitor/jetson`.

#### Aggregator (2 Hz)

Timer-driven merge of latest probe data into `RoverHealth`:
- Caches latest `CamStatus`, `Px4Status`, `JetsonStatus`
- Computes SLAM latency from `/rtabmap/odom` timestamps
- Runs the **Alert Engine** — evaluates `alert_rules.yaml` against current state
- Sets `overall_health` to the highest severity among active alerts (`OK` / `WARN` / `ERROR`)
- Publishes `RoverHealth` on `/monitor/health`

#### TelemetryPublisher

Bridges ROS 2 health data to MQTT and handles inbound commands.

**Outbound (ROS → MQTT):**

| MQTT Topic | QoS | Content |
|---|---|---|
| `rover/health/overall` | 0 | Protobuf `RoverHealth` (routine telemetry) |
| `rover/alerts` | 1 | Protobuf `RoverHealth` (only when active alerts exist) |

**Inbound (MQTT → ROS):**

| MQTT Topic | QoS | Content |
|---|---|---|
| `rover/cmd/arm` | 2 | Protobuf `RoverCommand` |
| `rover/cmd/disarm` | 2 | Protobuf `RoverCommand` |
| `rover/cmd/estop` | 2 | Protobuf `RoverCommand` |
| `rover/cmd/goal` | 2 | Protobuf `RoverCommand` (with `NavGoal`) |
| `rover/cmd/mode` | 2 | Protobuf `RoverCommand` (with `SetMode`) |
| `rover/cmd/cancel_goal` | 2 | Protobuf `RoverCommand` |

**ACK (MQTT outbound):**

| MQTT Topic | QoS | Content |
|---|---|---|
| `rover/cmd/ack` | 1 | Protobuf `CommandAck` |

**Thread Safety:** Inbound MQTT messages arrive on the Paho C++ async client's network thread. To avoid deadlocking the single-threaded ROS executor, commands and ACKs are queued via `std::queue` with a mutex and drained by a 50ms wall timer on the executor thread. See the design decision in [../decisions/ADR-009-mqtt-command-queue.md](../decisions/ADR-009-mqtt-command-queue.md) (pending).

### 2.3 Alert Rules

Defined in [`config/alert_rules.yaml`](../../src/rover_monitor/config/alert_rules.yaml). Each rule maps a condition to a named alert with severity.

| Alert ID | Severity | Condition |
|---|---|---|
| `CAM_STUTTER` | WARN | `frame_delta_ms > 66` (below 15 FPS) |
| `CAM_DISCONNECTED` | ERROR | `connected == false` |
| `PX4_HEARTBEAT_LOST` | ERROR | `heartbeat_age_ms > 3000` |
| `PX4_BATTERY_CRITICAL` | ERROR | `battery_remaining_pct < 20` |
| `HW_THROTTLE_THERMAL` | WARN | `is_thermal_throttled == true` |
| `HW_THROTTLE_POWER` | WARN | `is_power_throttled == true` |
| `SLAM_LATE` | WARN | `slam_latency_ms > 200` |
| `NET_DROP` | WARN | `wifi_signal_dbm < -75` |
| `JETSON_DISK_LOW` | ERROR | `disk_free_gb < 2.0` |

### 2.4 Protobuf Schema

Shared between rover and control center. Source: [`proto/rover_health.proto`](../../src/rover_monitor/proto/rover_health.proto).

**Telemetry messages:** `CamStatus`, `Px4Status`, `JetsonStatus`, `RoverHealth`

**Command messages (v1.2.0):**

```protobuf
enum CommandType {
  CMD_UNKNOWN = 0; CMD_NAV_GOAL = 1; CMD_ARM = 2;
  CMD_DISARM = 3;  CMD_SET_MODE = 4; CMD_ESTOP = 5;
  CMD_CANCEL_GOAL = 6; CMD_SET_PARAM = 7;
}

enum AckStatus {
  ACK_RECEIVED = 0; ACK_ACCEPTED = 1; ACK_REJECTED = 2;
  ACK_COMPLETED = 3; ACK_FAILED = 4;
}

message RoverCommand {
  string cmd_id = 1;
  CommandType cmd_type = 2;
  string issued_by = 3;
  int64 timestamp = 4;
  NavGoal nav_goal = 10;
  SetMode set_mode = 11;
  SetParam set_param = 12;
}

message CommandAck {
  string cmd_id = 1;
  CommandType cmd_type = 2;
  AckStatus status = 3;
  string message = 4;
  int64 timestamp = 5;
}
```

### 2.5 Launch Configuration

[`launch/monitor.launch.py`](../../src/rover_monitor/launch/monitor.launch.py) accepts:

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use simulation clock |
| `enable_telemetry` | `false` | Load TelemetryPublisher node |
| `publisher_config_file` | `publisher.yaml` | MQTT broker config path |

### 2.6 ROS Topics Published

| Topic | Type | Rate | Description |
|---|---|---|---|
| `/monitor/cam` | `rover_monitor/CamStatus` | 10 Hz | Camera health |
| `/monitor/px4` | `rover_monitor/Px4Status` | 2 Hz | PX4 health |
| `/monitor/jetson` | `rover_monitor/JetsonStatus` | 0.5 Hz | Platform health |
| `/monitor/health` | `rover_monitor/RoverHealth` | 2 Hz | Aggregated health + alerts |

## 3. Specification Reference

Detailed requirements and scenarios are in the OpenSpec specifications:

| Spec | Path | Covers |
|---|---|---|
| Camera Monitoring | [`openspec/specs/camera-monitoring/spec.md`](../../openspec/specs/camera-monitoring/spec.md) | Frame delta, depth quality, IMU, device enumeration |
| PX4 Monitoring | [`openspec/specs/px4-monitoring/spec.md`](../../openspec/specs/px4-monitoring/spec.md) | Heartbeat, battery, nav state, XRCE-DDS |
| Jetson Monitoring | [`openspec/specs/jetson-monitoring/spec.md`](../../openspec/specs/jetson-monitoring/spec.md) | Sysfs reads, cross-platform abstraction |
| Health Aggregation | [`openspec/specs/health-aggregation/spec.md`](../../openspec/specs/health-aggregation/spec.md) | Timer merge, alert engine, SLAM latency |
| Telemetry Publishing | [`openspec/specs/telemetry-publishing/spec.md`](../../openspec/specs/telemetry-publishing/spec.md) | Protobuf/MQTT, QoS strategy, reconnect |
| Command System | [`openspec/specs/command-system/spec.md`](../../openspec/specs/command-system/spec.md) | Bidirectional MQTT, dedup, ACK tracking |

Design decisions and trade-offs: [`openspec/changes/system-monitor/design.md`](../../openspec/changes/system-monitor/design.md)

Implementation task checklist: [`openspec/changes/system-monitor/tasks.md`](../../openspec/changes/system-monitor/tasks.md)

## 4. Software Implementation

### 4.1 Version History

| Version | Branch | Scope |
|---|---|---|
| v1.0.0 | `feature/system-monitor` | Probes + Aggregator + AlertEngine + TelemetryPublisher (outbound only) |
| v1.2.0 | `feature/system-monitor` | Command System (inbound MQTT) + Control Center + Host Deployment |

### 4.2 Key Design Decisions

1. **C++ composable nodes** — All probes, aggregator, and telemetry publisher are `rclcpp_components` loaded into a single `component_container`. This avoids inter-process serialization overhead and keeps the monitor lightweight on Jetson.

2. **Timer-based aggregation** — The aggregator uses a wall timer (2 Hz) to merge cached probe data, rather than synchronizing on all three probes. This decouples probe rates and ensures consistent output frequency.

3. **Direct sysfs/procfs** — JetsonProbe reads system metrics directly from `/sys/` and `/proc/` rather than wrapping `tegrastats` or `jtop`. This avoids subprocess overhead and dependency on Python tools.

4. **SystemMetricsProvider abstraction** — A compile-time and runtime abstraction layer allows the same JetsonProbe code to run on both Jetson (`JetsonMetricsProvider`) and x86_64 (`X86MockMetricsProvider`) without #ifdef.

5. **MQTT command queue** — Inbound MQTT commands are queued from the Paho network thread and drained by a 50ms ROS timer to prevent deadlocking the single-threaded executor. This was discovered and fixed during v1.2.0 verification.

6. **Command deduplication** — A `std::unordered_map<cmd_id, timestamp>` with 10s eviction timer prevents duplicate command execution from MQTT QoS 2 redelivery.

### 4.3 Dependencies

| Dependency | Version | Purpose |
|---|---|---|
| `rclcpp` / `rclcpp_components` | Jazzy | ROS 2 node framework |
| `sensor_msgs` | Jazzy | Camera topic types |
| `px4_msgs` | Latest | PX4 XRCE-DDS message types |
| `geometry_msgs` | Jazzy | TwistStamped for E-stop |
| `protobuf` | 3.x | Telemetry serialization |
| `paho-mqtt-cpp` | 1.3+ | MQTT async client |

## 5. Verification and Test Instructions

### 5.1 Prerequisites

- Docker CE installed (apt, not snap)
- Rover container started: `./scripts/start_docker.sh`
- For telemetry tests: Control Center stack running (see [control_center.md](control_center.md))

### 5.2 Build

```bash
# Inside rover container:
source /opt/ros/jazzy/setup.bash
cd /workspace
colcon build --packages-select rover_monitor --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 5.3 Run Without Telemetry (local ROS-only)

```bash
# Inside rover container:
ros2 launch rover_monitor monitor.launch.py use_sim_time:=false enable_telemetry:=false
```

In separate container shells, start mock publishers:

```bash
python3 /workspace/scripts/publish_mock_camera.py
python3 /workspace/scripts/publish_mock_px4.py
```

Verify health output:

```bash
ros2 topic echo /monitor/health
```

### 5.4 Run With Telemetry (MQTT to host)

Requires the [Control Center stack](control_center.md#4-verification-and-test-instructions) running on the host.

```bash
# Real hardware (default) — probes read live sensors, right panes echo probe topics:
./scripts/start_system_monitor_session.sh --with-telemetry

# x86 dev — starts mock camera/PX4 publishers, uses localhost broker:
./scripts/start_system_monitor_session.sh --mock --with-telemetry
```

This creates a tmux session `sysmon` with a 3×2 grid:

| Pane | Left | Right (--hw default) | Right (--mock) |
|---|---|---|---|
| Row 1 | rover_monitor launch | `/monitor/cam` echo | Mock camera publisher |
| Row 2 | `/monitor/health` echo | `/monitor/px4` echo | Mock PX4 publisher |
| Row 3 | Control Center stack | MQTT decoded / PX4 topic | MQTT decoded / PX4 topic |

### 5.5 Verify Telemetry Flow

```bash
curl -s http://localhost:8080/api/status | python3 -c "
import sys, json
d = json.load(sys.stdin)
for k in ['health','cam','px4','jetson']:
    e = d.get(k, {})
    print(f'{k}: stale={e.get(\"stale\")}, has_data={e.get(\"data\") is not None}')
"
```

Expected: all keys show `stale=False, has_data=True`.

### 5.6 Stop

```bash
# Kill tmux session
tmux kill-session -t sysmon

# Stop all ROS processes in container
./scripts/stop_all.sh

# Stop Control Center stack
docker compose -f control_center/docker-compose.yaml down
```

## 6. Related Documents

- [Control Center](control_center.md) — Host-side dashboard, command gateway, InfluxDB
- [Node Graph](node_graph.md) — Full ROS 2 node/topic graph
- [PX4 Overview](px4_overview.md) — PX4 XRCE-DDS integration details
- [Failure Modes](failure_modes.md) — Fault scenarios and recovery
