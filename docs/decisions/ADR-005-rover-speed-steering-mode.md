---
title: "ADR-005: RoverSpeedSteeringMode — Closed-Loop Speed with Normalized Steering"
status: Accepted
owner: taowang
last_updated: 2026-03-16
doc_type: ADR
ros_distro: humble
---

## Context

After validating the actuator path with `RoverManualMode` (open-loop throttle),
the next control layer needed is one that closes the loop on forward speed while
keeping steering as a direct normalized command. This mode was originally
designed as the primary Nav2 interface, but **steering is open-loop** — the
normalized steering value goes directly to PX4's actuator allocation without
any yaw rate or heading feedback. This means terrain disturbances, mechanical
backlash, and tire slip are not corrected.

**For Nav2 integration, `RoverSpeedRateMode` is now recommended instead** — it
uses PX4's closed-loop yaw rate controller (IMU gyro feedback) for reliable
steering tracking. See ADR-008.

`RoverSpeedSteeringMode` remains useful as a fallback or for scenarios where
PX4's yaw rate PID is not yet tuned and geometric steering is acceptable.

## Decision

Implement `RoverSpeedSteeringMode` as a `px4_ros2::ModeBase` subclass that:

1. **Uses `RoverSpeedSteeringSetpointType`** — sends `speed_body_x` [m/s]
   directly and normalized steering ∈ [-1, 1] to PX4. PX4's rover speed
   controller closes the loop on body-x velocity; the steering is still a
   direct normalized value (not a rate or angle command).

2. **Maps `cmd_vel` as follows**:
   - `linear.x` → `speed_body_x` [m/s] (passed through, no normalization)
   - `-angular.z / max_steering_rate` → normalized steering (negated: ROS CCW+
     → PX4 right+, clamped to [-1, 1])

3. **No `max_speed` parameter** — speed is passed in physical units (m/s)
   unlike `RoverManualMode`. The PX4 speed controller handles saturation.

4. **Exposes `skip_message_compatibility_check` as a ROS 2 parameter** —
   allows bypassing the 60 s FMU wait at startup via the YAML config
   (`px4_bridge.yaml`) without recompiling. This was added because the mode is
   also used in simulation setups where PX4 SITL may not be running when the
   node starts.

5. **Same cmd_vel watchdog pattern as `RoverManualMode`** — zero setpoint
   before first message; zero setpoint on timeout after first message.

## Alternatives Considered

### A. Normalize speed to [-1, 1] like RoverManualMode
Rejected. `RoverSpeedSteeringSetpointType` accepts physical m/s — normalizing
would hide the intended speed unit and require an extra `max_speed` parameter
that duplicates PX4's internal speed limit.

### B. Use angular.z directly as steering angle (not rate-normalized)
Rejected. Nav2's DWB/TEB planners output `angular.z` as a yaw rate [rad/s],
not a steering angle. Dividing by `max_steering_rate` correctly scales the
rate to a normalized steering demand for the current vehicle geometry.

### C. Use `RoverSpeedAttitudeMode` for Nav2
Rejected for Nav2. Nav2 issues continuous `cmd_vel` corrections; the
attitude mode integrates `angular.z` into an absolute heading setpoint,
which leads to heading drift when Nav2 repeatedly recomputes the path.
Speed+Steering is the correct interface for a velocity-tracking controller.

### D. Hard-code `skip_message_compatibility_check = true`
Rejected in favour of making it a runtime parameter. Keeping the check on
by default catches DDS misconfiguration; the YAML override provides the
flexibility for mixed sim/hw launch configurations.

## Consequences

**Positive:**
- Simple mapping — no intermediate translation layer needed.
- PX4 speed PID closes the loop; Nav2 does not need to know the motor curve.
- `skip_message_compatibility_check` parameter avoids recompilation for
  sim-first workflows.

**Negative / Trade-offs:**
- Requires a tuned PX4 speed controller (`RO_SPEED_P`, etc.) before useful
  autonomous navigation is possible.
- **Steering is open-loop** (normalized, not angle-controlled) — lateral
  tracking quality depends on tuning `max_steering_rate` to match the physical
  vehicle. No closed-loop correction for disturbances, backlash, or tire slip.
  `RoverSpeedRateMode` is preferred for Nav2 because it uses PX4's yaw rate
  PID with IMU gyro feedback.
- ~~Does not implement the FMU reconnect retry loop (unlike `RoverManualMode`)
  — the `rover_speed_steering_main.cpp` entry point uses the single-spin
  pattern. A future ADR or task should align this with ADR-004.~~
  **Resolved:** `rover_speed_steering_main.cpp` now mirrors the
  `while(true) { rclcpp::init(); try {...} catch (px4_ros2::Exception&) {
  shutdown; sleep; } }` pattern from `rover_manual_main.cpp`. Settings ctor
  also gained an explicit `.preventArming(false)` for symmetry.

## Implementation Notes

| File | Role |
|---|---|
| `src/px4_bringup/include/px4_bringup/rover_speed_steering_mode.hpp` | Mode implementation |
| `src/px4_bringup/src/rover_speed_steering_main.cpp` | Entry point |
| `src/px4_bringup/config/px4_bridge.yaml` | `skip_message_compatibility_check: true` for SITL |

### Setpoint type comparison

| Mode | Setpoint type | Speed input | Steering input | Steering feedback |
|---|---|---|---|---|
| RoverManualMode | `RoverThrottleSteeringSetpointType` | normalized [-1,1] | normalized [-1,1] | None (open-loop) |
| **RoverSpeedSteeringMode** | `RoverSpeedSteeringSetpointType` | m/s (physical) | normalized [-1,1] | None (open-loop) |
| RoverSpeedRateMode | `RoverSpeedRateSetpointType` | m/s (physical) | yaw rate [rad/s] | IMU gyro (closed-loop) — **Nav2 recommended** |
| RoverSpeedAttitudeMode | `RoverSpeedAttitudeSetpointType` | m/s (physical) | absolute yaw [rad] | IMU + heading (closed-loop) |
