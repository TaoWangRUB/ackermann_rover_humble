---
title: "ADR-008: PX4 Visual Odometry Frame and Origin Alignment"
status: Accepted
owner: decisions_team
last_updated: 2026-03-21
doc_type: ADR
ros_distro: humble
---

## Context

The rover has three odometry sources that must agree on frames and origin:

| Source | Topic | Frame | Origin at t=0 |
|---|---|---|---|
| D435i RGB-D VO → EKF | `/odometry/filtered` | `odom → ackermann/base_link` | base_link position (0,0,0) |
| T265 VIO → odom_tf_relay | `/t265/odom_base` | `t265_odom_frame → ackermann/base_link` | base_link position (0,0,0) |
| PX4 EKF2 visual odometry | `/fmu/in/vehicle_visual_odometry` | NED, relative to IMU | first received measurement |

PX4's IMU is physically on the Cube Black flight controller (`cubepilot_link`), mounted
at `(+0.087, 0, +0.1)` from `ackermann/base_link` in the URDF.

Two questions must be answered:

1. **Which frame does `px4_vision_odom.py` feed to PX4?** — base_link or cubepilot_link?
2. **How does `px4_vehicle_odometry.py` convert PX4 output back to base_link?**

## Decision

### Forward path: feed cubepilot_link position to PX4

`px4_vision_odom.py` uses `base_frame = cubepilot_link` to look up the TF
`odom → cubepilot_link` and feeds the Cube's actual position to PX4 EKF2.

This eliminates the IMU lever-arm error: PX4 EKF2 internally integrates
accelerometer data at the IMU location. If we feed base_link position instead,
PX4 thinks its IMU is 8.7 cm from where it actually is, introducing centripetal
acceleration errors during turns (ω² × 0.087 m ≈ 0.09 m/s² at 1 rad/s yaw rate).

### EKF2_EV_POS = (0, 0, 0)

`EKF2_EV_POS_X/Y/Z` defines the offset from the IMU to the vision measurement
reference point, in body FRD. Since the vision reference point IS `cubepilot_link`
(where the IMU is), the offset is zero.

### PX4 EKF2 origin auto-resets

PX4 EKF2 has no configurable origin parameter. When `startEvPosFusion()` runs,
it calls `resetHorizontalPositionTo(measurement)`, making the first received EV
position the local origin. No additional configuration needed.

### Reverse path: cubepilot_link → base_link via Stage 2

`px4_vehicle_odometry.py` already has a two-stage pipeline:

- **Stage 1**: `/fmu/out/vehicle_odometry` (NED/FRD) → `/px4_vehicle_odom` (ENU/FLU)
  with `child_frame_id = cubepilot_link`
- **Stage 2**: TF compose `T_world_cubepilot ∘ T_cubepilot_base` → `/px4_vehicle_odom_base`
  with `child_frame_id = ackermann/base_link`

This correctly removes the mount offset on the return path.

### T265 odom_tf_relay origin latching

`odom_tf_relay` latches the first computed `pos_WB` (base_link position in the
T265 odom frame) and subtracts it from all subsequent outputs. This ensures
`/t265/odom_base` starts at (0,0,0) — aligned with the EKF origin — rather than
at the raw sensor→base_link mount offset.

## Signal chains

### Forward: ROS → PX4

```
/odometry/filtered (odom → ackermann/base_link, ENU/FLU)
  → robot_localization EKF broadcasts TF: odom → ackermann/base_link
  → px4_vision_odom.py: TF lookup odom → cubepilot_link
  → ENU→NED coordinate conversion
  → /fmu/in/vehicle_visual_odometry (NED/FRD, cubepilot position)
  → PX4 EKF2 fuses with IMU (lever-arm = 0, no error)
```

### Reverse: PX4 → ROS

```
/fmu/out/vehicle_odometry (NED/FRD, cubepilot position)
  → px4_vehicle_odometry.py Stage 1: NED→ENU
  → /px4_vehicle_odom (vehicle_odom → cubepilot_link, ENU/FLU)
  → px4_vehicle_odometry.py Stage 2: TF cubepilot_link → base_link
  → /px4_vehicle_odom_base (odom → ackermann/base_link, ENU/FLU)
```

## URDF mount offsets (base_link → sensor, FLU)

| Frame | X (forward) | Y (left) | Z (up) |
|---|---|---|---|
| `cubepilot_link` | 0.087 m | 0 | 0.10 m |
| `d435i_link` | 0.187 m | 0 | 0.17 m |
| `t265_link` | 0.187 m | 0 | 0.21 m |

## PX4 parameters

| Parameter | Value | Reason |
|---|---|---|
| `EKF2_EV_POS_X` | 0.0 | Vision ref = IMU position |
| `EKF2_EV_POS_Y` | 0.0 | Vision ref = IMU position |
| `EKF2_EV_POS_Z` | 0.0 | Vision ref = IMU position |
| `EKF2_EV_CTRL` | 13 | Fuse hpos + vpos + vel (no yaw) |
| `EKF2_GPS_CTRL` | 0 | GPS disabled (indoor rover) |

## Consequences

- PX4 EKF2 lever-arm error eliminated (was ~0.09 m/s² at 1 rad/s yaw rate)
- All three odom sources (`/odometry/filtered`, `/t265/odom_base`,
  `/px4_vehicle_odom_base`) share the same origin and base_link frame
- `EKF2_EV_POS` simplifies to zero — no manual measurement needed
- Changing the cubepilot mount position in URDF automatically propagates
  through the TF tree; no PX4 parameter update required

## Files changed

| File | Change |
|---|---|
| `src/px4_bringup/scripts/px4_vision_odom.py` | `base_frame` default: `ackermann/base_link` → `cubepilot_link` |
| `src/px4_bringup/launch/px4_bringup.launch.py` | `base_frame` default: `ackermann/base_link` → `cubepilot_link` |
| `src/px4_bringup/config/cube_black_ackermann.params` | `EKF2_EV_POS_X/Y/Z` → `0.0` |
| `src/realsense_camera_bringup/src/odom_tf_relay.cpp` | Added origin-latching (subtract first `pos_WB`) |
