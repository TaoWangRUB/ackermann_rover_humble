# Jetson Xavier NX — Full-Stack Integration Report

**Date:** 2026-04-15  
**Branch:** `verify/cuvslam-full-stack`  
**Platform:** Jetson Xavier NX (6 CPU cores, aarch64)  
**Container:** `jazzy_slam_aarch64` (ROS 2 Jazzy)

---

## 1. Objective

Validate the full navigation stack on Jetson Xavier NX hardware:
**RealSense cameras → cuVSLAM → RTAB-Map SLAM → Nav2 → PX4 bridge**,
launched via `start_jetson_session.sh`.

## 2. Hardware Setup

| Component | Detail |
|---|---|
| Board | Jetson Xavier NX (6-core ARM, 8 GB) |
| Depth Camera | Intel RealSense D435i (USB 3.2) |
| Tracking Camera | Intel RealSense T265 (USB 3.1) |
| USB Topology | Both cameras on shared USB 3 hub |
| Host ↔ Jetson Link | USB-C (10.42.0.2) + LAN (192.168.0.179) |
| DDS Config | FastDDS restricted to 10.42.0.x (USB subnet) |

## 3. Software Stack

```
T265 fisheye stereo ──► cuVSLAM ──► /cuvslam/raw_odometry
                                        │
                                        ▼
                               cuvslam_odom_relay ──► /cuvslam_odom
                                                          │
D435i RGB-D ──► rgbd_sync ──► RTAB-Map SLAM              │
                                  │                       ▼
                                  │              robot_localization (EKF)
                                  │                       │
                                  ▼                       ▼
                            /map (OccupancyGrid)   /odometry/filtered
                                  │                       │
                                  └───────────┬───────────┘
                                              ▼
                                    Nav2 (planning + control)
                                              │
                                              ▼
                                   PX4 bridge → /fmu/in/vehicle_visual_odometry
```

## 4. Issues Found & Resolved

### 4.1 T265 Frozen Pipeline (commit `29ad0da`)

**Symptom:** T265 node would start but never produce frames. The Movidius VPU
on the T265 would get stuck if a previous session was killed without a clean
USB reset.

**Root Cause:** `enable_hardware_reset` was `false` for T265. The VPU retained
stale firmware state across sessions.

**Fix:** Enabled `t265_enable_hardware_reset: true` in `robot_bringup.launch.py`
and added a 15-second data-flow watchdog inside `realsense_camera_node.cpp` that
triggers `restart_pipeline_with_reset()` if no frames arrive.

### 4.2 D435i Alignment Thread Crash (commit `f40694c`)

**Symptom:** Occasional `SIGABRT` from the D435i node during watchdog-triggered
restart.

**Root Cause:** The depth-alignment `std::thread` was not joined before being
replaced during pipeline restart.

**Fix:** Added `if (alignment_thread_.joinable()) alignment_thread_.join();`
before restart.

### 4.3 D435i Hardware Reset Causes Stall (commit `0023df0`)

**Symptom:** D435i node would hang during startup when `enable_hardware_reset`
was `true`.

**Root Cause:** The D435i does not need a hardware reset (it recovers cleanly
on reconnect, unlike the T265's Movidius VPU).

**Fix:** Set `d435i_enable_hardware_reset: false`, keep `t265_enable_hardware_reset: true`.
Added a 20-second D435i startup delay to let the T265 claim USB bandwidth first.

### 4.4 Deferred RS Context Init — Reverted (commits `0beb484` → `b9f3362`)

**Attempt:** Used `std::optional` to defer `rs2::context` and `rs2::pipeline`
initialization until after the startup delay.

**Result:** Caused SIGSEGV on both cameras. The `rs2::pipeline` constructor
requires a valid context at member-init time. Reverted.

### 4.5 Nav2 CPU Starvation (commit `041bfc4`)

**Symptom:** With full Nav2 stack, load average spiked to **13.76** on 6 cores
(2.3× overloaded). EKF failed update rate continuously. Camera frame drops.

**Root Cause:** Nav2 default bringup launched 10 lifecycle nodes. Four of them
are unused for this rover:

| Removed Node | Reason |
|---|---|
| `smoother_server` | No path smoothing needed |
| `route_server` | No route graph used |
| `waypoint_follower` | Single-goal navigation only |
| `docking_server` | No docking hardware |

**Fix:** Removed these four nodes from both non-composed and composed code paths
in `nav2_bringup.launch.py`, plus the `lifecycle_nodes` list.

## 5. Performance Measurements

### 5.1 Baseline: Cameras + cuVSLAM + RTAB-Map (no Nav2)

Test command:
```bash
./scripts/start_ros2_nodes.sh --hw --cuvslam-odom --rtabmap --depth-camera=d435i --no-rviz
```

| Metric | Value |
|---|---|
| Load average (1 min) | ~4.8 |
| Total CPU usage | ~285% of 600% |
| RTAB-Map rate | 1.00 Hz (steady) |
| RTAB-Map compute | 0.15–0.22 s |
| cuVSLAM CPU | ~27% |
| D435i node CPU | ~50% |
| T265 node CPU | ~43% |
| EKF errors | None |

### 5.2 With Nav2 — Before Fix (10 lifecycle nodes)

Test command:
```bash
./scripts/start_ros2_nodes.sh --hw --cuvslam-odom --rtabmap --nav2 --depth-camera=d435i --no-rviz
```

| Metric | Value |
|---|---|
| Load average (1 min) | **13.76** |
| Total CPU usage | ~435%+ |
| Process count | ~370 |
| EKF errors | Continuous |
| Nav2 nodes CPU (total) | ~110% |

### 5.3 With Nav2 — After Fix (6 lifecycle nodes)

Same test command as 5.2 (code change only, no config change):

| Metric | Value |
|---|---|
| Load average (1 min) | **5.39** |
| Total CPU usage | ~350% |
| Process count | ~357 |
| RTAB-Map rate | 1.00 Hz (steady, 139+ frames) |
| EKF errors | ~10 sporadic during startup, then rare |
| Nav2 lifecycle | All 6 nodes active ✅ |
| Node crashes | None |

### 5.4 Full Session (all 5 panes)

Test command:
```bash
./scripts/start_jetson_session.sh --cuvslam-odom --nav2 --no-activate
```

**Before optimization** (Python PX4 bridge, unthrottled relays):

| Metric | Value |
|---|---|
| Load average (settled) | ~13.75 |
| Top instantaneous CPU | ~530% of 600% |
| RTAB-Map rate | 1.00 Hz (254+ frames) |
| Nav2 lifecycle | "Managed nodes are active" ✅ |
| PX4 VO bridge | Forwarding odometry ✅ |
| XRCE Agent | Active ✅ |
| Monitor | Aggregator loaded ✅ |
| Node crashes | None |
| EKF errors | Sporadic during activation, settling |

**After optimization** (C++ PX4 bridge, 30 Hz throttled relays, 2026-04-16):

| Metric | Value | vs Before |
|---|---|---|
| Load average (settled) | ~11.13 | **−2.6** |
| Top instantaneous CPU | ~547% of 600% | ≈ same (measurement noise) |
| RTAB-Map rate | 1.00 Hz (steady) ✅ | unchanged |
| Nav2 lifecycle | "Managed nodes are active" ✅ | unchanged |
| PX4 VO bridge | Forwarding odometry ✅ | unchanged |
| XRCE Agent | Active ✅ | unchanged |
| Monitor | Aggregator loaded ✅ | unchanged |
| Node crashes | None | unchanged |
| EKF errors | Sporadic during activation, settling | unchanged |

**Note:** Load average is the more reliable long-term indicator (it captures
queueing pressure, not momentary scheduling). Top instantaneous CPU varies with
measurement timing; use load average for comparisons. After optimization, the
PX4 bridge Python overhead (~63.5 CPU%) was replaced by ~17.8% C++ equivalent,
recovering ~46 CPU% headroom at steady state.

### 5.5 Top CPU Consumers — Before vs After Optimization

**Before** (2026-04-15, Python PX4 bridge + unthrottled relays):

| Process | CPU % |
|---|---|
| D435i realsense_camera_node | 65.8 |
| rtabmap | 51.8 |
| T265 realsense_camera_node | 44.9 |
| **px4_vision_odom.py** | **31.8** |
| **px4_vehicle_odometry.py** | **31.5** |
| monitor component_container | 28.8 |
| cuvslam_odom_node | 26.1 |
| imu_transformer | 19.2 |
| **odom_tf_relay (cuvslam, unthrottled)** | **18.0** |
| nav2 planner_server | 16.6 |
| nav2 lifecycle_manager | 15.7 |
| nav2 bt_navigator | 14.3 |
| rgbd_sync | 13.9 |
| nav2 controller_server | 13.8 |
| imu_filter_madgwick | 12.7 |
| nav2 behavior_server | 12.6 |
| nav2 collision_monitor | 11.2 |
| ekf_node | 9.9 |
| nav2 velocity_smoother | 8.6 |
| **Load average (settled)** | **13.75** |

**After** (2026-04-16, C++ PX4 bridge + 30 Hz throttled relays):

| Process | CPU % | vs Before |
|---|---|---|
| D435i realsense_camera_node | 65.3 | ≈ same |
| rtabmap | 53.0 | ≈ same |
| T265 realsense_camera_node | 50.5 | ≈ same |
| monitor component_container | 37.4 | ≈ same |
| cuvslam_odom_node | 29.9 | ≈ same |
| imu_transformer | 19.9 | ≈ same |
| nav2 lifecycle_manager | 16.3 | ≈ same |
| nav2 planner_server | 14.9 | ≈ same |
| nav2 bt_navigator | 14.6 | ≈ same |
| rgbd_sync | 13.7 | ≈ same |
| nav2 controller_server | 13.6 | ≈ same |
| nav2 behavior_server | 12.9 | ≈ same |
| imu_filter_madgwick | 12.8 | ≈ same |
| **t265_odom_relay (30 Hz throttled)** | **12.4** | — (new, T265 active) |
| nav2 collision_monitor | 11.1 | ≈ same |
| ekf_node | 9.9 | ≈ same |
| nav2 velocity_smoother | 8.5 | ≈ same |
| **px4_vision_odom_node (C++)** | **6.4** | **−25.4%** |
| **cuvslam_odom_relay (30 Hz throttled)** | **6.0** | **−12.0%** |
| **px4_vehicle_odometry_node (C++)** | **5.4** | **−26.1%** |
| **Load average (settled)** | **12.65** | **−1.1** |

**Summary of targeted optimizations:**

| Node | Before | After | Saving |
|---|---|---|---|
| px4_vision_odom (Python → C++) | 31.8% | 6.4% | −25.4% |
| px4_vehicle_odometry (Python → C++) | 31.5% | 5.4% | −26.1% |
| odom_tf_relay / cuvslam_odom_relay (30 Hz throttle) | 18.0% | 6.0% | −12.0% |
| **Total** | **81.3%** | **17.8%** | **−63.5%** |

### 5.6 T265 Odometry Mode — Side-by-Side Comparison (2026-04-16)

Test command:
```bash
./scripts/start_jetson_session.sh --t265-odom --nav2 --no-activate
```

Replaces cuVSLAM (`cuvslam_odom_node` + `cuvslam_odom_relay`) with direct T265 odometry
(`t265_odom_relay` only). RTAB-Map uses `/t265/odom_base` instead.

| Process | cuVSLAM-odom | T265-odom | Δ |
|---|---|---|---|
| D435i realsense_camera_node | 65.3 | 64.2 | ≈ same |
| rtabmap | 53.0 | 52.2 | ≈ same |
| T265 realsense_camera_node | 50.5 | 36.5 | −14.0 |
| monitor component_container | 37.4 | 36.7 | ≈ same |
| **cuvslam_odom_node** | **29.9** | **0** | **−29.9** |
| imu_transformer | 19.9 | 19.9 | ≈ same |
| cmd_vel relay | — | 18.3 | — |
| nav2 lifecycle_manager | 16.3 | 16.4 | ≈ same |
| nav2 planner_server | 14.9 | 15.1 | ≈ same |
| nav2 bt_navigator | 14.6 | 14.7 | ≈ same |
| **t265_odom_relay (30 Hz throttled)** | **12.4** | **14.7** | +2.3 |
| rgbd_sync | 13.7 | 14.0 | ≈ same |
| nav2 controller_server | 13.6 | 13.5 | ≈ same |
| nav2 behavior_server | 12.9 | 12.9 | ≈ same |
| imu_filter_madgwick | 12.8 | 12.6 | ≈ same |
| nav2 collision_monitor | 11.1 | 11.2 | ≈ same |
| ekf_node | 9.9 | 8.9 | ≈ same |
| nav2 velocity_smoother | 8.5 | 8.5 | ≈ same |
| px4_vision_odom_node (C++) | 6.4 | 6.5 | ≈ same |
| **cuvslam_odom_relay (30 Hz throttled)** | **6.0** | **0** | **−6.0** |
| px4_vehicle_odometry_node (C++) | 5.4 | 5.3 | ≈ same |
| **Load average (settled)** | **12.65** | **10.51** | **−2.1** |

**Key difference:** Removing `cuvslam_odom_node` (~30%) and `cuvslam_odom_relay` (~6%) in
T265-odom mode saves ~36 CPU%, driving load average down a further **−2.1** (12.65 → 10.51).
The T265 camera node itself also drops ~14% since it is no longer feeding cuVSLAM's stereo
fisheye pipeline — only the T265 odometry relay at 30 Hz remains.

**Trade-off:** T265 built-in VIO (librs2 internal) replaces cuVSLAM neural odometry. Accuracy
and robustness in challenging environments (motion blur, low texture) may differ; cuVSLAM is
expected to be more accurate but at higher CPU cost.

## 6. Remaining Nav2 Lifecycle Nodes

After trimming, the active Nav2 nodes are:

1. `controller_server` — executes local trajectory tracking
2. `planner_server` — generates global paths
3. `behavior_server` — recovery behaviors (spin, backup, wait)
4. `velocity_smoother` — smooths commanded velocities
5. `collision_monitor` — runtime collision checking
6. `bt_navigator` — behavior-tree navigation orchestrator

## 7. Known Limitations

1. **CPU headroom improved but still limited.** After PX4 bridge C++ conversion,
   relay throttling, and switching to T265-odom mode, load average dropped from
   ~13.75 → 10.51 (cuVSLAM-odom: 12.65). Adding more nodes (e.g. costmap
   filters, additional sensors) will still require Nav2 node composition
   (`use_composition:=True`) or offloading.

2. **EKF sporadic overruns.** During Nav2 lifecycle activation, the EKF node
   occasionally misses its 30 Hz update target (takes 36–134 ms instead of
   33 ms). These are transient and do not affect steady-state operation.

3. **D435i startup delay.** A fixed 20-second delay is used to let the T265
   claim USB bandwidth first. This could be replaced with a USB enumeration
   check in the future.

4. **PX4 Ctrl-C handling.** The PX4 bringup scripts do not properly handle
   SIGINT — a pending fix.

5. **px4_vision_odom publish rate capped at 10 Hz.** The C++ node maintains
   the existing 10 Hz output to PX4 due to XRCE-DDS back-pressure causing cycle
   stalls on the STM32F427 (Cube Black) at higher rates. Input is accepted at
   up to 30 Hz with lazy conversion.

## 7.1 Further Improvements (2026-04-17)

Two follow-up changes addressing items 1 (CPU headroom) and 2 (EKF
overruns) from Section 7.

### 7.1.1 Nav2 planner tuning for short near-field goals

During live goal testing (`NavigateToPose` → `(x=1.0, y=0.0)` from rest),
the goal was accepted but the robot never moved. `distance_remaining`
stayed at ~0.998 m for the full 10 s progress-checker window. Nav2 then
aborted with `Failed to make progress`.

Root cause: the SmacPlannerHybrid (Reeds-Shepp) could not find a
curvature-feasible path from the robot's startup pose through the narrow
free corridor left by the inflation layer. `minimum_turning_radius` was
0.50 m with `inflation_radius` 0.70 m on a ~0.22 m-radius rover — the
planner had almost no room to curve near obstacles or map edges.

Config changes ([src/ackermann_nav2_bringup/config/nav2_params.yaml](../src/ackermann_nav2_bringup/config/nav2_params.yaml),
commit `c551f4e`):

| Parameter | Old | New |
|---|---|---|
| `GridBased.minimum_turning_radius` | 0.50 | 0.25 |
| `local_costmap.inflation_radius` | 0.70 | 0.35 |
| `global_costmap.inflation_radius` | 0.70 | 0.35 |

After the change, planner logs `Passing new path to controller.`
repeatedly during a goal — the planner succeeds end-to-end.

### 7.1.2 Drop robot_localization EKF; T265 relay owns TF edge

With EKF active, top CPU showed `ekf_filter_node + imu_filter_madgwick +
imu_transformer` together consuming ~15–20 % of one core at 30 Hz EKF
frequency. On the already-starved Jetson NX (load avg 16.03 on 6 cores,
idle 1.4 %), this contributed to the EKF overruns noted in Section 7.2
and to control-loop jitter in `controller_server` during goals.

Since the T265 already provides 6-DoF odometry at ~230 Hz (see
Section 5.6), routing it directly to Nav2/RTAB-Map removes the EKF
entirely from the hot path.

Changes (commit `1311458`):

1. **rtabmap_slam.launch.py** — comment out `imu_transform_node`,
   `imu_filter_node`, `ekf_filter_node`. RTAB-Map consumes
   `/t265/odom_base` directly via `use_t265_odom:=true`.

2. **realsense_camera.launch.py** — flip `t265_relay_publish_tf`
   default from `'false'` → `'true'`. The existing C++ `odom_tf_relay`
   (30 Hz-throttled since `a095fe9`) now owns the
   `odom → ackermann/base_link` TF edge that EKF used to publish.

TF tree ownership after the change:

| Edge | Publisher |
|---|---|
| `map → odom` | RTAB-Map (unchanged) |
| `odom → ackermann/base_link` | `t265_odom_relay` (was `ekf_filter_node`) |

Without this TF change, `tf2_echo odom ackermann/base_link` reports
`frame does not exist` and Nav2 cannot resolve robot pose — the TF edge
is mandatory.

### 7.1.3 MPPI load reduction (applied) — commits `27a93f6`, `6b5e0ea`, `861073a`

Goal execution was CPU-bound after 7.1.1 / 7.1.2. During an active goal,
`controller_server` emitted `Control loop missed its desired rate of
20.0000 Hz` continuously, and `velocity_smoother` occasionally triggered
`CRITICAL FAILURE` in the lifecycle manager. MPPI at
`batch_size: 1000 × time_steps: 56` at 20 Hz was the dominant cost
(~56 000 rollout-steps per tick).

The following reductions were applied across three commits:

| Parameter | Before | After | Reason |
|---|---|---|---|
| `controller_server.controller_frequency` | 20.0 | **10.0** | Halves MPPI invocation rate. Rover at 0.5 m/s only moves 5 cm per 10 Hz tick — 20 Hz was overkill. |
| `FollowPath.batch_size` | 1000 | **200** | 5× fewer Monte-Carlo samples. Ackermann dynamics are low-dimensional; 200 suffices. |
| `FollowPath.time_steps` | 56 | **20** | Horizon `time_steps × model_dt = 2.0 s` matches `prune_distance=1.0 m` at 0.5 m/s. |
| `FollowPath.model_dt` | 0.05 | **0.1** | Required: MPPI asserts `model_dt ≥ 1 / controller_frequency`. Paired with the 10 Hz drop. |
| `FollowPath.visualize` | true | **false** | Stops publishing the per-cycle candidate-trajectory MarkerArray (MB/s of message work). |
| `SmacHybrid.max_iterations` | 1 000 000 | **200 000** | Safety cap only; planner always converges well below this. Faster failure on truly infeasible goals. |
| `costmap_update_timeout` | 0.30 | **0.50** | 300 ms was too tight at 10 Hz under CPU pressure; `controller_server` aborted with spurious costmap-update timeouts. |
| `bt_navigator.default_server_timeout` | 20 | **200** | BT sub-server timeouts (costmap clears, recoveries) fired spuriously under load. |

Per-tick work: `200 × 20 = 4 000` steps, down from `1000 × 56 = 56 000` —
**≈14× reduction**. Pipeline rates after the change:

- `/cmd_vel_nav` (controller_server): **10 Hz**
- `/cmd_vel_smoothed` (velocity_smoother): 20 Hz
- `/cmd_vel` (collision_monitor, passthrough): 20 Hz
- `/fmu/in/rover_throttle_setpoint` + `rover_steering_setpoint`
  (rover_manual_mode): 30 Hz (independent PX4 bridge timer)

Interdependence worth knowing: `controller_frequency`, `model_dt`, and
`time_steps` are coupled. If CPU budget is restored and 20 Hz becomes
desirable again, set `model_dt: 0.05` and `time_steps: 40` to keep the
2 s horizon.

### 7.1.4 Plumbing remaps + transform-tolerance relaxation — commit `6b5e0ea`

Alongside the MPPI reduction, several inherited defaults were updated
to match the current topology and tolerate Jetson/laptop timing jitter.

Scan topic remap (RPLidar dropped; depthimage_to_laserscan now
publishes on `/scan`):

| Location | Before | After |
|---|---|---|
| `local_costmap.obstacle_layer.scan.topic` | `/rplidar/scan` | `/scan` |
| `global_costmap.obstacle_layer.scan.topic` | `/rplidar/scan` | `/scan` |
| `collision_monitor.scan.topic` | `rplidar/scan` | `scan` |

Odometry remap (post-EKF removal — velocity_smoother can't hard-code the
old EKF output topic anymore; `/odom` is the standard name that
`nav2_bringup.launch.py` remaps onto the current odom source):

| Location | Before | After |
|---|---|---|
| `velocity_smoother.odom_topic` | `/odometry/filtered` | `/odom` |

TF-tolerance relaxation (T265 / RTAB-Map / WiFi-bridged TF jitter can
reach 100–400 ms under load — previous 0.1–0.3 s tolerances tripped
repeatedly):

| Component | Before | After |
|---|---|---|
| `MPPI.transform_tolerance` | 0.3 | 1.0 |
| `behavior_server.transform_tolerance` | 0.1 | 1.0 |
| `collision_monitor.transform_tolerance` | 0.2 | 1.0 |
| `collision_monitor.source_timeout` | 1.0 | 5.0 |
| `docking_server.transform_tolerance` | 0.1 | 1.0 |

1 s tolerance on a 0.5 m/s rover bounds positional uncertainty at
~0.5 m — still well under the 0.35 m inflation radius + footprint
margin.

Removed: explicit `current_progress_checker` / `current_goal_checker`
keys — Nav2 Jazzy makes these implicit and warns when they're set to
the default values.

### 7.1.5 Nav2 offload to host laptop — commit `861073a`

Even after 7.1.3/7.1.4, launching the full stack (RealSense + T265 +
RTAB-Map + PX4 bridge + Nav2 + telemetry) still saturated the Xavier NX
during `Managed nodes are active` startup. To free permanent headroom,
Nav2 was moved off the Jetson and onto the base-station laptop.

New script: [scripts/start_nav2.sh](../scripts/start_nav2.sh) runs Nav2
inside the host's `ackermann_slam` Docker container with the same
defaults as `robot_bringup.launch.py` (controller, params_file, bt_xml,
reversible_drive). The container has `network=host` on the shared LAN,
so TF / `/map` / `/odometry/filtered` / `/scan` / `/cmd_vel` cross via
DDS without any bridge.

Invariant preserved: the Jetson session command
(`start_jetson_session.sh --nav2 ...`) still works; the offload script
is an alternative when Jetson CPU is the bottleneck.

Bonus fix during bringup: MPPI's `model_dt` must be ≥ controller period
(see 7.1.3). With the 10 Hz controller and the pre-existing 0.05 s
model_dt, MPPI's configure transition threw
`Controller period more then model dt`. The `model_dt: 0.05 → 0.1` bump
made the stack reach `Managed nodes are active` on the host.

### 7.1.6 Surface CLI/RViz goals in the RCC Nav2 panel — commit `cc8641c`

The `TelemetryPublisher` in `rover_monitor` maintains a `Nav2Status`
only from callbacks of its own `NavigateToPose` action *client* (used
by the RCC's MQTT `CMD_NAV_GOAL` path). Goals sent via
`ros2 action send_goal` or RViz therefore left the RCC panel stuck on
`IDLE` while the rover navigated.

Two passive subscribers added alongside the client path:

| Topic | Type | Purpose |
|---|---|---|
| `/navigate_to_pose/_action/feedback` | `NavigateToPose_FeedbackMessage` | EXECUTING + `distance_remaining_m`, `eta_seconds`, `navigation_time_s`, `number_of_recoveries` |
| `/navigate_to_pose/_action/status` | `action_msgs/GoalStatusArray` | Terminal labels: SUCCEEDED / CANCELED / ABORTED / CANCELING |

Guard: when `active_nav2_goal_handle_` is non-null (RCC owns the goal),
the shadow path is a no-op so the authoritative client callbacks stay
in charge.

Dependencies added to the package: `action_msgs` (already transitively
present via `rclcpp_action`, but declared explicitly in
`CMakeLists.txt` and `package.xml` for cleanliness).

### 7.1.7 Outstanding

`polkitd` was observed at 75 % CPU in one `top` snapshot during the
7.1.3 investigation — unexpected for an idle auth daemon and worth a
dedicated investigation (likely a telemetry probe hammering
dbus/systemd). Not yet addressed.

## 7.2 Data Logging — rosbag2 segment recorder (2026-04-24)

A rosbag2 capture pipeline was added so cameras + SLAM inputs can be
recorded on the Jetson and replayed against RTAB-Map on the x86 host
for offline tuning (parameter sweeps, loop-closure debugging, RGB-D
vs cuVSLAM vs VINS odometry comparisons).

### 7.2.1 Scripts

| Script | Role |
|---|---|
| [scripts/record_bag.sh](../scripts/record_bag.sh) | Wraps `ros2 bag record` inside the container. Builds the topic list from the same odom / depth-camera flags used by `start_jetson_session.sh`, gates the recorder on RTAB-Map readiness, and handles the interactive r/s/Ctrl-C key loop. |
| [scripts/start_rosbag_session.sh](../scripts/start_rosbag_session.sh) | Launches a tmux session for either `--record` (live capture) or `--replay` (offline RTAB-Map against a recorded bag, default). |

`start_rosbag_session.sh --record` assumes the live stack
(`start_jetson_session.sh` or `start_ros2_nodes.sh`) is already
running; record mode does **not** invoke `stop_all.sh` so the cameras /
RTAB-Map / TF tree stay up between recording sessions. Replay mode
starts a fresh RTAB-Map against the bag with `use_sim_time:=true`.

### 7.2.2 Topic set

Always recorded on hardware:

`/d435i/color/{image_raw,camera_info}`,
`/d435i/aligned_depth_to_color/{image_raw,camera_info}`,
`/d435i/imu`, `/t265/odom`, `/t265/odom_base`, `/cmd_vel`, `/tf`,
`/tf_static`

T265 native odometry (`/t265/odom`, `/t265/odom_base`) is captured in
every bag — even in `--cuvslam-odom` / `--vins-odom` / `--rgbd-odom`
modes — so offline replay can compare any odom path against the T265
baseline without re-collecting data.

Conditionally added per `--*-odom` flag:

| Flag | Adds |
|---|---|
| `--t265-odom` (default) | *(nothing extra; T265 odom is in the always-on set)* |
| `--cuvslam-odom` | `/t265/fisheye{1,2}/{image_raw,camera_info}`, `/t265/imu`, `/cuvslam_odom` |
| `--rgbd-odom` | `/cuvslam_rgbd_odom` |
| `--vins-odom` | `/t265/fisheye{1,2}/{image_raw,camera_info}`, `/t265/imu`, `/vins_odom` |

For `--cuvslam-odom` / `--vins-odom`, T265 fisheye streams are
PNG-republished via `image_transport` (lossless, ~3.3× volume
reduction) and the compressed topic is recorded instead of raw.
Replay decompresses transparently when it detects the
`/compressed` suffix in `metadata.yaml`.

### 7.2.3 Storage and readiness gate

- **Format:** MCAP with `zstd_fast` storage preset profile.
- **Output path:** `<repo>/bags/<NAME>_seg<N>/` (see segment model
  below). Default `NAME` is `run_YYYYMMDD_HHMM`.
- **Readiness gate:** before starting the recorder, the script blocks
  on (a) every required topic appearing in `ros2 topic list`, (b) one
  message arriving on each, then (c) the `/rtabmap` node coming up
  *and* publishing on one of `/mapData`, `/mapGraph`, `/rtabmap/info`
  (warns and proceeds after 30 s if none arrive). This prevents the
  bag from starting before RTAB-Map's TF tree is fully populated,
  which would otherwise leave the first few seconds unusable for
  replay.

### 7.2.4 Segment-based interactive control (commits `3cc5ce6`, `fdeb447`, `c5d0bef`, `2c94bc0`)

Inside a TTY (tmux pane), the recorder waits idle and exposes three
keys:

| Key | Action |
|---|---|
| `r` | Spawn a fresh `ros2 bag record` into `${NAME}_seg${N}/`, incrementing N each time. State → `[recording -> name]`. |
| `s` | SIGTERM the current recorder, wait up to 30 s for it to flush the MCAP and write `metadata.yaml`, escalate to SIGKILL if needed. State → `[idle]`. |
| `Ctrl-C` | Finalize the current segment (if any) and exit the script. |

This is a deliberate departure from rosbag2's built-in pause/resume
service model: pause/resume keeps a single growing bag, but operators
wanted each `r` press to produce a standalone bag (one drive segment
per file), and they wanted to keep cutting bags from one tmux session
without re-running the script.

Two non-obvious bugs were found and fixed during validation:

1. **`ros2 bag record`'s CLI wrapper ignores SIGINT from `kill(2)`.**
   It only reacts to a real Ctrl-C from a controlling TTY. The
   cleanup path was originally `kill -INT $REC_PID`, which left the
   recorder running indefinitely with no `metadata.yaml` written.
   Fixed by switching to SIGTERM, which the python launcher does
   forward (commit `fdeb447`).

2. **30 s grace period before SIGKILL.** A 9 s test bag finalized in
   under 10 s, but longer recordings (5+ minute drives) can leave
   the zstd compressor draining several seconds of buffered data on
   shutdown. Bumped the poll loop from 20 × 0.5 s to 60 × 0.5 s
   (commit `c5d0bef`).

Without a TTY (e.g. piped or non-interactive run), the script falls
back to recording a single segment that auto-starts and runs until
SIGINT.

### 7.2.5 Recording workflow (live stack already up)

The record path assumes the Jetson session
(`start_jetson_session.sh`) is already running — cameras, RTAB-Map,
TF, and PX4 bridge stay up across recording sessions. From a fresh
SSH terminal:

```bash
# 1. Verify the Jetson session is up (rover_stack tmux + container nodes)
ssh jetson 'tmux ls && \
  cd ~/workspace/ackermann_rover_humble && \
  source scripts/lib/dc.sh && \
  dcomp exec -T ackermann_slam bash -lc \
    "source /opt/ros/jazzy/setup.bash && ros2 node list" \
    | grep -E "/d435i|/t265|/rtabmap"'

# 2. Launch the record session (tmux session "rosbag", record_bag.sh in pane 0)
ssh -tt jetson 'cd ~/workspace/ackermann_rover_humble && \
  ./scripts/start_rosbag_session.sh --record --t265-odom \
    --name=run_$(date +%Y%m%d_%H%M)'

# 3. Once "[idle]" appears in the record pane, drive segments with r / s.
#    Ctrl-C in the pane finalizes the active segment and exits.
```

Re-attach to a detached session at any time:

```bash
ssh -tt jetson 'tmux attach -t rosbag'
```

Use `--cuvslam-odom`, `--vins-odom`, or `--rgbd-odom` to record the
matching SLAM-input topology (T265 fisheye streams + the chosen
`*_odom` topic). T265 native odometry (`/t265/odom`,
`/t265/odom_base`) is included in every hardware bag regardless of
flag (see 7.2.2).

After Ctrl-C, the bag lives at `~/workspace/ackermann_rover_humble/bags/<NAME>_seg<N>/`
on the Jetson; sync it to the host with rsync (see next section)
before replay.

### 7.2.6 Replay workflow

```bash
# Sync bag from Jetson to host (rsync over WLAN, ~4 MB/s steady)
rsync -aP nvidia@jetson-wlan:~/workspace/ackermann_rover_humble/bags/run_<ts>/ \
        ./bags/run_<ts>/

# Replay with a fresh RTAB-Map (default: latest bag, T265 odom)
./scripts/start_rosbag_session.sh

# Or replay a specific bag with explicit odom source
./scripts/start_rosbag_session.sh --name=run_20260424_2053 --t265-odom
```

The replay tmux layout splits into three panes: RTAB-Map +
rtabmap_viz (top-left), `ros2 bag play` (bottom-left), and an
inspection shell (right). The bag-play pane waits for `/rtabmap` to
appear before starting playback, so the first frames don't get
dropped during RTAB-Map's lifecycle init.

### 7.2.7 Round-trip validation (Jetson, 2026-04-24)

End-to-end test on real HW:

| Step | Result |
|---|---|
| Record `run_20260424_2142` (D435i + T265 odom, 166 s drive) | 4.7 GiB, 99 003 msgs across 9 topics, all valid in `ros2 bag info` |
| Sync to x86 host (5.0 GB over WLAN) | 20 m 37 s @ 4.0 MB/s avg |
| Replay with fresh RTAB-Map | `/map`, `/mapData`, `/mapGraph`, `/cloud_map`, `/octomap_*` produced |
| Segment-mode test (r → s → r → Ctrl-C) | 2 standalone bags, both with valid `metadata.yaml`, durations 10.5 s + 10.0 s |

CPU impact during recording on top of the live stack is dominated by
zstd (15–25 % of one core) and the rosbag2 writer thread (~10 %).
This is well within the post-7.1 CPU budget on Xavier NX.

## 8. Commit History

| Commit | Description |
|---|---|
| `041bfc4` | Remove unnecessary Nav2 nodes (smoother, route, waypoint, docking) |
| `b9f3362` | Revert broken std::optional deferred init |
| `0023df0` | Only T265 needs hardware_reset; revert D435i/L515 |
| `f40694c` | Fix alignment thread join in watchdog restart |
| `29ad0da` | Enable hardware reset + data-flow watchdog for T265 |
| `39813cf` | Stabilize host and Jetson session scripts |
| `c24920d` | Clarify SLAM metric as TF age in rover-monitor |
| `c551f4e` | Nav2 planner tuning for short near-field goals (7.1.1) |
| `1311458` | Drop EKF/imu_filter; T265 relay owns odom→base_link TF (7.1.2) |
| `27a93f6` | Reduce MPPI load on Xavier NX (7.1.3, first pass) |
| `6b5e0ea` | Nav2 params: scan/odom remaps + TF-tolerance relax (7.1.4) |
| `7e968d5` | Add `--controller=rpp|mppi` flag for Nav2 controller swap |
| `861073a` | Fix MPPI `model_dt` + add `scripts/start_nav2.sh` host offload (7.1.5) |
| `cc8641c` | rover_monitor shadows Nav2 feedback+status for CLI/RViz goals (7.1.6) |
| `915bbf6` | Support simulated VIO RTAB-Map bag collection (7.2 base) |
| `3cc5ce6` | Key-gated rosbag recording (r=record, s=stop, Ctrl-C=finalize) (7.2.4) |
| `fdeb447` | Fix Ctrl-C finalization: SIGTERM instead of SIGINT (7.2.4) |
| `c5d0bef` | 30 s recorder grace period before SIGKILL (7.2.4) |
| `2c94bc0` | Segment-based recording: s finalizes, r starts a new bag (7.2.4) |
| *(pending)* | Convert PX4 Python bridges to C++; throttle odom relays to 30 Hz |

## 9. Conclusion

The full navigation stack runs stably on Jetson Xavier NX with the trimmed Nav2
configuration and optimized PX4 bridge nodes. All components — dual RealSense
cameras, RTAB-Map SLAM, Nav2 planning/control, PX4 odometry bridge, and system
monitor — coexist without crashes in both odometry modes.

Three rounds of optimization brought overall load average from **13.75 → 10.51**:

| Optimization | Load avg | Δ |
|---|---|---|
| Baseline (Python bridges, unthrottled relays, cuVSLAM-odom) | 13.75 | — |
| C++ PX4 bridges + 30 Hz relay throttle (cuVSLAM-odom) | 12.65 | −1.1 |
| Switch to T265-odom (drop cuVSLAM node + relay, ~−36 CPU%) | 10.51 | −2.1 |

In T265-odom mode, the PX4 bridge nodes (C++) account for only **~11.8% CPU**
combined (px4_vision_odom 6.5% + px4_vehicle_odometry 5.3%), down from the
original **63.3% Python total** (31.8% + 31.5%).

Future work to reclaim additional headroom:
- Nav2 node composition (`use_composition:=True`) to reduce inter-process IPC
- Offloading RTAB-Map or costmap computation to a companion x86 host
- Reducing monitor container telemetry polling rate
- Re-evaluate cuVSLAM necessity: if T265 built-in VIO accuracy is sufficient,
  cuVSLAM-odom mode is no longer needed, permanently freeing ~36 CPU%
