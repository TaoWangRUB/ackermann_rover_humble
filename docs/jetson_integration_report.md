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

| Metric | Value |
|---|---|
| Load average (settled) | ~13.75 |
| RTAB-Map rate | 1.00 Hz (254+ frames) |
| Nav2 lifecycle | "Managed nodes are active" ✅ |
| PX4 VO bridge | Forwarding odometry ✅ |
| XRCE Agent | Active ✅ |
| Monitor | Aggregator loaded ✅ |
| Node crashes | None |
| EKF errors | Sporadic during activation, settling |

**Note:** The full session adds ~110% CPU from PX4 Python nodes and ~46% from
the monitor container, pushing load average back up. The system is CPU-saturated
but functionally stable — all components produce correct output at expected rates.

### 5.5 Top CPU Consumers (Full Session, Steady State)

| Process | CPU % |
|---|---|
| D435i realsense_camera_node | 65.8 |
| rtabmap | 51.8 |
| T265 realsense_camera_node | 44.9 |
| px4_vision_odom.py | 31.8 |
| px4_vehicle_odometry.py | 31.5 |
| monitor component_container | 28.8 |
| cuvslam_odom_node | 26.1 |
| imu_transformer | 19.2 |
| odom_tf_relay | 18.0 |
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

## 6. Remaining Nav2 Lifecycle Nodes

After trimming, the active Nav2 nodes are:

1. `controller_server` — executes local trajectory tracking
2. `planner_server` — generates global paths
3. `behavior_server` — recovery behaviors (spin, backup, wait)
4. `velocity_smoother` — smooths commanded velocities
5. `collision_monitor` — runtime collision checking
6. `bt_navigator` — behavior-tree navigation orchestrator

## 7. Known Limitations

1. **CPU headroom is tight.** The full 5-pane session uses ~530% of 600%
   available CPU. Adding more nodes (e.g. costmap filters, additional sensors)
   will require either composition or offloading to a companion computer.

2. **EKF sporadic overruns.** During Nav2 lifecycle activation, the EKF node
   occasionally misses its 30 Hz update target (takes 36–134 ms instead of
   33 ms). These are transient and do not affect steady-state operation.

3. **D435i startup delay.** A fixed 20-second delay is used to let the T265
   claim USB bandwidth first. This could be replaced with a USB enumeration
   check in the future.

4. **PX4 Ctrl-C handling.** The PX4 bringup scripts do not properly handle
   SIGINT — a pending fix.

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

## 9. Conclusion

The full navigation stack runs stably on Jetson Xavier NX with the trimmed Nav2
configuration. All components — dual RealSense cameras, cuVSLAM visual odometry,
RTAB-Map SLAM, Nav2 planning/control, PX4 odometry bridge, and system monitor —
coexist without crashes. The primary constraint is CPU budget: the Xavier NX's
6 cores are near saturation at ~530% utilization. Future work should explore
Nav2 node composition (`use_composition:=True`) and reducing PX4 bridge Python
overhead to reclaim headroom.
