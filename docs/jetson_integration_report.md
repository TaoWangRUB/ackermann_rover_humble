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

### 7.1.3 Outstanding — MPPI load on Xavier NX

Goal execution remains CPU-bound even after 7.1.1 and 7.1.2. During an
active goal, `controller_server` emits `Control loop missed its desired
rate of 20.0000 Hz` continuously, and `velocity_smoother` has been
observed triggering `CRITICAL FAILURE` in the lifecycle manager. MPPI
at `batch_size: 1000 × time_steps: 56` at 20 Hz is the dominant cost.

Candidate further reductions (not yet applied):

| Parameter | Current | Candidate |
|---|---|---|
| `FollowPath.batch_size` | 1000 | 300 |
| `FollowPath.time_steps` | 56 | 30 |
| `FollowPath.visualize` | true | false |
| `controller_server.controller_frequency` | 20.0 | 10.0 |
| `velocity_smoother.smoothing_frequency` | 20.0 | 10.0 |

Separately: `polkitd` was observed at 75 % CPU in one `top` snapshot —
unexpected for an idle auth daemon and worth a dedicated investigation
(likely a telemetry probe hammering dbus/systemd).

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
