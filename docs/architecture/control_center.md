---
title: Control Center — Host-Side Dashboard & Command Gateway
status: Accepted
owner: Tao Wang
last_updated: 2026-03-28
doc_type: architecture
ros_distro: jazzy
---

# Control Center

Host-side monitoring dashboard and command gateway for the Ackermann rover. Receives Protobuf telemetry over MQTT from the rover's [System Monitor](system_monitor.md), displays it via a FastAPI + React web UI, stores time-series data in InfluxDB, and sends commands back to the rover with safety gating and ACK tracking.

## 1. Overview

### 1.1 Purpose

The Control Center runs on a host machine (laptop, base station, or cloud) and provides:

- **Live telemetry dashboard** — Real-time camera, PX4, and Jetson health panels via WebSocket
- **Command interface** — Send arm/disarm/E-stop/nav goal/mode changes with safety preconditions
- **ACK tracking** — Per-command UUID tracking with timeout detection
- **Time-series storage** — InfluxDB for historical telemetry, alerts, and command logs
- **Alert notifications** — Terminal, sound, desktop, Slack, and email channels with cooldown dedup

### 1.2 System Architecture

```
                    MQTT (Protobuf)
  ┌─────────┐    rover/health/overall     ┌──────────────────────────────────────┐
  │  Rover  │◄────────────────────────────│         Control Center               │
  │ Monitor │    rover/cmd/{arm,estop,..} │                                      │
  │         │────────────────────────────►│  ┌─────────────┐  ┌──────────────┐  │
  │         │    rover/cmd/ack            │  │ MQTT        │  │ Telemetry    │  │
  │         │◄────────────────────────────│  │ Receiver    │──│ Cache        │  │
  └─────────┘                             │  └──────┬──────┘  └──────┬───────┘  │
                                          │         │ EventBus       │           │
                                          │  ┌──────▼──────┐  ┌─────▼────────┐ │
                                          │  │ InfluxDB    │  │ Dashboard    │ │
                                          │  │ Writer      │  │ Server       │ │
                                          │  └─────────────┘  │ (FastAPI+WS) │ │
                                          │                    └─────▲────────┘ │
                                          │  ┌─────────────┐        │           │
                                          │  │ Alert       │  ┌─────┴────────┐ │
                                          │  │ Notifier    │  │ Cmd Gateway  │ │
                                          │  └─────────────┘  │ + Safety Gate│ │
                                          │  ┌─────────────┐  │ + ACK Tracker│ │
                                          │  │ Event Bus   │  └──────────────┘ │
                                          │  └─────────────┘                    │
                                          └──────────────────────────────────────┘
```

### 1.3 Infrastructure Stack

All services use `network_mode: host` so `localhost` resolves to the same host network.

| Service | Image | Port | Purpose |
|---|---|---|---|
| Mosquitto | `eclipse-mosquitto:2` | 1883 | MQTT broker |
| InfluxDB | `influxdb:2` | 8086 | Time-series storage |
| Control Center | Custom (Python 3.11 + React) | 8080 | Dashboard + API |

## 2. Architecture

### 2.1 Package Structure

```
control_center/
├── main.py                    # Async entry point (signal handling, component wiring)
├── docker-compose.yaml        # Mosquitto + InfluxDB + CC services
├── Dockerfile                 # Multi-stage: Node 20 (Vite build) → Python 3.11 runtime
├── requirements.txt           # paho-mqtt, fastapi, uvicorn, influxdb-client, protobuf, etc.
├── cc/                        # Python module
│   ├── event_bus.py           # CC-0: Async pub/sub with error isolation
│   ├── mqtt_receiver.py       # CC-1: Paho MQTT client + Protobuf deserialization
│   ├── telemetry_cache.py     # CC-2: asyncio.Lock-protected latest-value store
│   ├── influxdb_writer.py     # CC-3: Batch writes (telemetry) + sync (alerts/commands)
│   ├── alert_notifier.py      # CC-4: Multi-channel notifications with cooldown
│   ├── dashboard_server.py    # CC-5: FastAPI REST + WebSocket + static SPA serving
│   ├── cmd_gateway.py         # CC-6b: Command serialization + MQTT publish
│   ├── safety_gate.py         # CC-6a: Per-CommandType precondition checks
│   └── ack_tracker.py         # CC-7: Future-based ACK matching with timeout
├── proto/                     # Generated Python Protobuf stubs
│   └── rover_health_pb2.py
├── scripts/
│   └── setup_influxdb_dashboard.py  # Idempotent InfluxDB dashboard provisioning
├── config/
│   ├── control_center.yaml    # Master config (all component settings)
│   └── mosquitto.conf         # Mosquitto broker config
└── ui/                        # React SPA (Vite + JSX)
    ├── index.html
    ├── vite.config.js
    └── src/
        ├── index.jsx
        └── App.jsx            # Health panels, command buttons, WebSocket connections
```

### 2.2 Component Details

#### CC-0: Event Bus (`event_bus.py`)

Simple async pub/sub. Handlers are error-isolated — a failing handler does not block others.

```python
bus.subscribe("evt.health", handler)
await bus.emit("evt.health", data=health_msg)
```

#### CC-1: MQTT Receiver (`mqtt_receiver.py`)

- Paho MQTT v3.1.1 client with exponential backoff reconnection (1s → 30s)
- Subscribes to topics on connect/reconnect:

| Subscription | QoS | Emits Event |
|---|---|---|
| `rover/health/#` | 0 | `evt.health`, `evt.health.cam`, `evt.health.px4`, `evt.health.jetson` |
| `rover/alerts` | 1 | `evt.alert` |
| `rover/cmd/ack` | 1 | `evt.cmd_ack` |

- Deserializes Protobuf messages and dispatches typed events via `asyncio.run_coroutine_threadsafe`
- Exposes `publish()` method used by CommandGateway for outbound commands

#### CC-2: Telemetry Cache (`telemetry_cache.py`)

- `asyncio.Lock`-protected `dict` keyed by subsystem (`health`, `cam`, `px4`, `jetson`)
- Each entry has a `time.monotonic()` timestamp
- Staleness threshold: 5 seconds (configurable)
- Used by Safety Gate and Dashboard `/api/status`

#### CC-3: InfluxDB Writer (`influxdb_writer.py`)

| Measurement | Write Mode | Fields |
|---|---|---|
| `rover_telemetry` | Async batch (500ms flush) | Camera/PX4/Jetson metrics, SLAM latency |
| `rover_alerts` | Sync | alert_id, severity, message, timestamp |
| `rover_commands` | Sync | cmd_id, cmd_type, issued_by, ack_status |

Degrades gracefully if InfluxDB is unavailable (logs warning, continues operating).

#### CC-4: Alert Notifier (`alert_notifier.py`)

Per-alert_id cooldown deduplication (60s default). Supported channels:

| Channel | Implementation | Severity Filter |
|---|---|---|
| Terminal | `logging.warning()` | All |
| Sound | `aplay` system command | ERROR only |
| Desktop | `plyer` notification | WARN+ |
| Slack | Webhook POST | ERROR only |
| Email | SMTP | ERROR only |

#### CC-5: Dashboard Server (`dashboard_server.py`)

**REST Endpoints:**

| Method | Path | Description |
|---|---|---|
| `GET` | `/api/status` | Live telemetry (health, cam, px4, jetson + staleness) |
| `POST` | `/api/command` | Send command (`cmd_type` + `params`) |
| `GET` | `/api/command/history` | Command log with ACK status + round-trip |
| `GET` | `/api/health/history` | Current health snapshot |
| `GET` | `/` | React SPA (index.html) |

**WebSocket Endpoints:**

| Path | Broadcasts |
|---|---|
| `/ws/health` | Real-time `RoverHealth` (JSON) |
| `/ws/alerts` | Real-time alerts (JSON) |
| `/ws/cmd_ack` | Real-time command ACK (JSON) |

**POST `/api/command` request body:**

```json
{
  "cmd_type": "estop",
  "params": {},
  "issued_by": "dashboard"
}
```

#### CC-6a: Safety Gate (`safety_gate.py`)

Per-CommandType precondition checks using telemetry cache:

| Command | Preconditions |
|---|---|
| `estop` | Always allowed |
| `disarm` | Always allowed |
| `cancel_goal` | Always allowed |
| `arm` | Non-stale telemetry + PX4 connected + battery > 20% |
| `set_mode` | Non-stale telemetry + PX4 connected |
| `nav_goal` | Non-stale + armed + SLAM latency < 200ms + overall health ≠ ERROR |

#### CC-6b: Command Gateway (`cmd_gateway.py`)

1. Safety Gate check → reject if preconditions not met
2. Generate UUID v4 `cmd_id`
3. Build Protobuf `RoverCommand` with type-specific fields
4. Publish to MQTT topic (QoS 2)
5. Emit `evt.cmd_sent` for ACK tracker

| `cmd_type` | MQTT Topic |
|---|---|
| `arm` | `rover/cmd/arm` |
| `disarm` | `rover/cmd/disarm` |
| `estop` | `rover/cmd/estop` |
| `nav_goal` | `rover/cmd/goal` |
| `set_mode` | `rover/cmd/mode` |
| `cancel_goal` | `rover/cmd/cancel_goal` |

#### CC-7: ACK Tracker (`ack_tracker.py`)

- Tracks pending commands in a `dict[cmd_id, Future]`
- Resolves on matching `CommandAck` event from MQTT
- Timeout: 5 seconds (configurable) → marks as `ACK_FAILED`
- Records round-trip time in milliseconds
- History available via `/api/command/history`

### 2.3 Configuration

Master config: [`config/control_center.yaml`](../../control_center/config/control_center.yaml)

Key sections:

```yaml
mqtt:
  broker_host: localhost
  broker_port: 1883
  client_id: control_center_01
  subscriptions:
    - { topic: "rover/health/#", qos: 0 }
    - { topic: "rover/alerts", qos: 1 }
    - { topic: "rover/cmd/ack", qos: 1 }

cache:
  stale_threshold_s: 5.0

dashboard:
  host: 0.0.0.0
  port: 8080

influxdb:
  url: http://localhost:8086
  token: rover-dev-token
  org: rover
  bucket: rover_telemetry

cmd_gateway:
  ack_timeout_s: 5.0
  publish_qos: 2
```

### 2.4 Docker Compose

[`docker-compose.yaml`](../../control_center/docker-compose.yaml):

- All 3 services use `network_mode: host` — `localhost` resolves identically in all containers and on the host
- InfluxDB auto-initializes with org `rover`, bucket `rover_telemetry`, admin token `rover-dev-token`
- CC config is volume-mounted (`./config:/app/config:ro`) so changes don't require rebuild
- `restart: unless-stopped` for all services

### 2.5 Dockerfile

Multi-stage build ([`Dockerfile`](../../control_center/Dockerfile)):

1. **Stage 1 (Node 20):** `npm install` + `npx vite build` → produces `dist/` with React SPA assets
2. **Stage 2 (Python 3.11):** Install Python deps, copy app code + built UI assets, `protoc` generates Python stubs from shared `rover_health.proto`

## 3. Specification Reference

| Spec | Path | Covers |
|---|---|---|
| Command System | [`openspec/specs/command-system/spec.md`](../../openspec/specs/command-system/spec.md) | Bidirectional MQTT, dedup, ACK flow |
| Control Center | [`openspec/specs/control-center/spec.md`](../../openspec/specs/control-center/spec.md) | CC-1 through CC-7 component requirements |
| Host Deployment | [`openspec/specs/host-deployment/spec.md`](../../openspec/specs/host-deployment/spec.md) | Docker compose, InfluxDB setup, networking |
| Telemetry Publishing | [`openspec/specs/telemetry-publishing/spec.md`](../../openspec/specs/telemetry-publishing/spec.md) | Protobuf/MQTT protocol, QoS strategy |

Design decisions: [`openspec/changes/system-monitor/design.md`](../../openspec/changes/system-monitor/design.md)

Proposal: [`openspec/changes/system-monitor/proposal.md`](../../openspec/changes/system-monitor/proposal.md)

## 4. Verification and Test Instructions

### 4.1 Prerequisites

- Docker CE installed (apt, **not** snap — snap has AppArmor issues with `docker stop`)
- Rover container available: `./scripts/start_docker.sh`
- rover_monitor built (see [system_monitor.md](system_monitor.md#52-build))

### 4.2 Start the Control Center Stack

```bash
cd ~/workspace/ackermann_rover_humble
docker compose -f control_center/docker-compose.yaml up -d
```

Verify all 3 services:

```bash
# Mosquitto broker
docker logs control_center-mosquitto-1 2>&1 | tail -3

# InfluxDB (should say "ready for queries and writes")
curl -s http://localhost:8086/health | python3 -m json.tool

# Dashboard API (should return {} when no telemetry is flowing)
curl -s http://localhost:8080/api/status
```

### 4.3 Start Rover Monitor with Telemetry

**Option A — tmux session (recommended):**

```bash
# Real hardware (default):
./scripts/start_system_monitor_session.sh --with-telemetry

# x86 dev with mock publishers:
./scripts/start_system_monitor_session.sh --mock --with-telemetry
```

Creates a 6-pane tmux session (includes CC stack in bottom pane). See [system_monitor.md](system_monitor.md#54-run-with-telemetry-mqtt-to-host) for pane layout.

**Option B — manual launch:**

```bash
# Terminal 1: rover_monitor
docker exec -it jazzy_slam_x86_64 bash -c '
  source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
  ros2 launch rover_monitor monitor.launch.py \
    use_sim_time:=false \
    enable_telemetry:=true \
    publisher_config_file:=/workspace/src/rover_monitor/config/publisher.localhost.yaml'

# Terminal 2: mock camera
docker exec -it jazzy_slam_x86_64 bash -c '
  source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
  python3 /workspace/scripts/publish_mock_camera.py'

# Terminal 3: mock PX4
docker exec -it jazzy_slam_x86_64 bash -c '
  source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
  python3 /workspace/scripts/publish_mock_px4.py'
```

### 4.4 Automated Test

Run the full verification suite:

```bash
./scripts/test_control_center.sh
```

This tests telemetry flow, command dispatch (ESTOP, DISARM, ARM rejection, CANCEL_GOAL), ACK tracking, and post-command telemetry continuity. Override the CC URL with `CC_URL=http://host:8080`.

### 4.5 Manual Verification

**Telemetry flow:**

```bash
# All 4 keys should show stale=False
curl -s http://localhost:8080/api/status | python3 -c "
import sys, json
d = json.load(sys.stdin)
for k in ['health','cam','px4','jetson']:
    e = d.get(k, {})
    print(f'{k}: stale={e.get(\"stale\")}, has_data={e.get(\"data\") is not None}')
"
```

**Commands:**

```bash
# ESTOP — always allowed
curl -s -X POST http://localhost:8080/api/command \
  -H 'Content-Type: application/json' \
  -d '{"cmd_type": "estop", "params": {}}' | python3 -m json.tool

# DISARM — always allowed
curl -s -X POST http://localhost:8080/api/command \
  -H 'Content-Type: application/json' \
  -d '{"cmd_type": "disarm", "params": {}}' | python3 -m json.tool

# ARM — rejected with mock data (battery 10% < 20% threshold)
curl -s -X POST http://localhost:8080/api/command \
  -H 'Content-Type: application/json' \
  -d '{"cmd_type": "arm", "params": {}}' | python3 -m json.tool

# CANCEL_GOAL — always allowed
curl -s -X POST http://localhost:8080/api/command \
  -H 'Content-Type: application/json' \
  -d '{"cmd_type": "cancel_goal", "params": {}}' | python3 -m json.tool
```

**Command history + ACK:**

```bash
curl -s http://localhost:8080/api/command/history | python3 -m json.tool
```

Each sent command shows `ack_status`, `message`, and `round_trip_ms` (typically 30-60ms).

**Post-command telemetry:**

```bash
# Should still show stale=False after E-stop/disarm commands
curl -s http://localhost:8080/api/status | python3 -c "
import sys, json; d = json.load(sys.stdin)
print('health stale:', d.get('health',{}).get('stale'))
"
```

### 4.8 Dashboard UI

Open in browser: **http://localhost:8080**

The React SPA shows live health panels for Camera, PX4, and Jetson subsystems.

### 4.9 InfluxDB Dashboard Provisioning

The "Rover Health Monitor" dashboard (6 panels) is auto-provisioned when using `start_system_monitor_session.sh --with-telemetry`. To provision manually:

```bash
python3 control_center/scripts/setup_influxdb_dashboard.py
```

Panels: PX4 Battery %, PX4 Battery Voltage, Camera Depth FPS, Jetson Temperatures (°C), Jetson GPU %, Active Alerts.

### 4.10 InfluxDB UI

Open in browser: **http://localhost:8086**

Login: `rover` / `rover-password`

Navigate to **Dashboards** → "Rover Health Monitor" for the provisioned panels, or **Data Explorer** → bucket `rover_telemetry` for ad-hoc queries.

### 4.11 Stop Everything

```bash
# Kill tmux session (if using Option A)
tmux kill-session -t sysmon

# Stop all ROS processes in rover container
./scripts/stop_all.sh

# Stop Control Center stack
docker compose -f control_center/docker-compose.yaml down
```

Verify:

```bash
docker ps --format '{{.Names}}\t{{.Status}}'
```

Only `jazzy_slam_x86_64` should remain (rover container stays up for development).

## 5. Related Documents

- [System Monitor](system_monitor.md) — Rover-side health monitoring (probes, aggregator, alerts)
- [PX4 Overview](px4_overview.md) — PX4 XRCE-DDS integration
- [Interfaces](interfaces.md) — ROS 2 topic contracts
- [Failure Modes](failure_modes.md) — Fault scenarios and recovery
