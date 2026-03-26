## Context

The Ackermann rover runs ROS 2 Humble on a Jetson Xavier NX with a PX4 Cube Black flight controller (via Micro XRCE-DDS at 921600 baud) and RealSense D4xx depth camera. The Xavier runs SLAM (RTAB-Map), Nav2, ros2_control, and the RealSense driver concurrently. Under load, thermal throttling, PX4 heartbeat loss, camera frame drops, and SLAM staleness can occur silently. Operators need real-time telemetry on a host machine.

The existing codebase uses `ament_cmake` throughout, C++ for PX4 custom modes, and Python for bridge nodes. There is no `diagnostic_msgs` usage. The Docker container runs privileged with host network and NVIDIA runtime.

## Goals / Non-Goals

**Goals:**
- Real-time health monitoring of camera, PX4, and Jetson subsystems
- Deterministic scheduling that prevents probes from starving each other under SLAM load
- Zero-copy intra-process communication between components
- Remote telemetry via Protobuf/MQTT to host dashboard (Grafana + InfluxDB)
- Configurable alert rules with named IDs and severity levels

**Non-Goals:**
- Automatic recovery or self-healing (monitor only, not actuate)
- Replacing the existing safety watchdog (that handles emergency stop via `/fault`)
- Generic ROS 2 node/topic monitoring (probes are domain-specific)
- `diagnostic_msgs` integration (using custom messages + MQTT instead)

## Decisions

**C++ rclcpp_components, not Python**: All probes run as C++ components in a single `component_container` process. Intra-process pub/sub uses shared-memory pointer hand-off — zero copies, no serialization. Python cannot participate in intra-process comms and would add GIL contention under load.

**`component_container` (single-threaded), not `component_container_mt`**: The multi-threaded container's thread pool allows camera callback spikes under SLAM load to starve the PX4 heartbeat probe — the exact failure mode this system must detect. Each component gets a `MutuallyExclusiveCallbackGroup` with `StaticSingleThreadedExecutor` for deterministic scheduling.

**Timer-based latest-value cache, not ApproximateTimeSynchronizer**: `ApproximateTimeSynchronizer` buffers messages waiting for timestamp matches. At 0.5 Hz for the Jetson probe, it would hold camera and PX4 data for up to 2 seconds. The aggregator instead caches each probe's latest message in `std::optional<T>` and reads whatever is current on its 2 Hz timer, marking stale slots as degraded.

**Direct sysfs/procfs reads, not jtop/tegrastats/psutil**: `jtop` and `tegrastats` spawn background threads and maintain continuous state. Direct file reads at 0.5 Hz are synchronous, minimal, and avoid library dependencies. `psutil` is Python-only.

**Protobuf over MQTT, not JSON or ROS 2 topics for host telemetry**: Protobuf binary encoding is ~3-10x smaller and ~5x faster to serialize than JSON. MQTT provides broker-based decoupling with QoS levels (QoS 0 routine, QoS 1 alerts). ROS 2 DDS multicast is unreliable over WiFi.

**Micro XRCE-DDS for PX4, not MAVROS**: MAVROS adds two serialization boundaries and a Python GIL-bound thread. XRCE-DDS exposes uORB topics directly as ROS 2 topics with a single lightweight agent process. Already used by `px4_bringup`.

**Frame delta instead of rolling FPS average for camera**: Measures wall-clock interval between consecutive frame callbacks. Catches micro-stutters (individual delayed frames) that a 1-second rolling average smooths over.

**Depth quality sampled 1-in-10 frames**: Full-frame fill ratio scan on every frame wastes CPU. Sampling every 10th frame reduces load by 90% with negligible accuracy loss for health monitoring.

**Thermal vs power throttle split**: Xavier distinguishes two independent throttle causes that can occur simultaneously. Thermal → check fan/cooling. Power → check battery/regulator. Different operator responses require separate detection and alerting.

**SLAM latency via /tf stamp delta, not /tf broadcast frequency**: `/tf` republishes stale transforms at high frequency — frequency stays high while the pose goes stale. Comparing the `header.stamp` of the `map → odom` transform against wall clock gives true SLAM staleness.

## Risks / Trade-offs

**Jetson sysfs path fragmentation** — Thermal zone indices and GPU load paths differ between Xavier NX and AGX Xavier, and across JetPack versions. Mitigation: probe multiple known paths at startup, log which were found, fall back gracefully.

**`/proc/net/wireless` dBm offset** — Some kernels require subtracting 256 from the raw level field. Mitigation: verify on target device during integration; make offset configurable.

**`is_power_throttled` requires debugfs** — The `bpmp/debug/clk` path requires `mount -t debugfs`. Mitigation: verify availability in production image; fall back to comparing `tegrastats VDD_CPU` against configured TDP if debugfs unavailable.

**paho-mqtt-cpp dependency** — Not in standard ROS 2 packages. Mitigation: add as rosdep key or vendor as source dependency in CMakeLists.txt.

**Protobuf code generation** — Adds build complexity. Mitigation: use `protobuf_generate_cpp()` CMake macro; keep `.proto` schema in `proto/` directory.

**MQTT TLS not included in v1** — Plaintext MQTT is acceptable on local LAN but not for remote deployments. Mitigation: document as open question; add TLS support in future iteration.
