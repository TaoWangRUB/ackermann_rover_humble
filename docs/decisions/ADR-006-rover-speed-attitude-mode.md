---
title: "ADR-006: RoverSpeedAttitudeMode — Closed-Loop Speed with Integrated Heading Hold"
status: Accepted
owner: taowang
last_updated: 2026-03-16
doc_type: ADR
ros_distro: humble
---

## Context

`RoverSpeedSteeringMode` tracks `cmd_vel` well for continuous navigation, but
offers no heading stability when `angular.z = 0` — the rover drifts under
disturbances because steering is a direct normalized command with no closed-loop
correction. A heading-hold mode is needed for:

- Straight-line driving where drift correction is required
- Manual teleoperation scenarios where the operator sends brief angular
  corrections and expects the vehicle to hold the resulting heading
- Any use case where lateral stability at low speeds is important and a heading
  feedback loop is preferable to a rate-based steering command

PX4 provides `RoverSpeedAttitudeSetpointType` which accepts `speed_body_x`
[m/s] and an absolute yaw target [rad, NED CW+]. PX4's attitude controller
then closes the loop on heading → steering conversion.

## Decision

Implement `RoverSpeedAttitudeMode` as a `px4_ros2::ModeBase` subclass that:

1. **Uses `RoverSpeedAttitudeSetpointType`** — sends `speed_body_x` [m/s] and
   an absolute yaw setpoint [rad, NED CW+]. PX4's Ackermann attitude controller
   converts the heading error into a steering command.

2. **Integrates `angular.z` into an absolute heading setpoint each cycle**:
   ```
   yaw_setpoint_ += (-angular.z) * dt_s
   ```
   The negation converts ROS ENU CCW+ to PX4 NED CW+. The result is wrapped
   to [-π, π] after each integration step.

3. **Seeds the heading on activation** — `onActivate()` reads the current
   vehicle heading from `OdometryLocalPosition::heading()` (PX4 telemetry, NED
   frame) and uses it as the initial `yaw_setpoint_`. This prevents a sudden
   heading step when the mode is activated.

4. **Holds heading when `angular.z = 0`** — because the setpoint is absolute
   and not updated when there is no rate command, PX4's attitude controller
   actively corrects any drift back to the last commanded heading.

5. **On timeout or before first message: zero speed, hold last heading** —
   unlike `RoverManualMode` and `RoverSpeedSteeringMode` which zero the full
   setpoint, this mode holds the heading to prevent the vehicle from spinning
   while stopped.

6. **Does not expose `skip_message_compatibility_check`** — the FMU check
   runs unconditionally at startup (60 s timeout).

## Alternatives Considered

### A. Rate-based steering (like RoverSpeedSteeringMode)
Rejected. Rate-based steering with `angular.z = 0` produces normalized
steering = 0 but offers no correction for external disturbances (camber,
surface unevenness). The attitude mode actively holds heading.

### B. Integrate heading in ENU, convert to NED for setpoint
Considered but rejected for simplicity. The integration is done directly in
NED (by negating `angular.z` before integrating), avoiding an intermediate
frame conversion. Since only the sign differs and there is no trigonometric
component, a direct negate-then-integrate approach is correct and simpler.

### C. Read heading from `/odometry/filtered` (robot_localization EKF) instead of PX4 telemetry
Rejected. `OdometryLocalPosition` reads PX4's own local position estimate,
which is the frame in which the `RoverSpeedAttitudeSetpointType` operates.
Using `robot_localization` output would introduce a frame inconsistency if the
two EKF estimates diverge.

### D. Use a PID on heading error in ROS 2, send corrected steering to SpeedSteering mode
Rejected. This duplicates PX4's attitude controller in ROS 2. PX4 already
provides heading hold; the correct approach is to use the right setpoint type.

## Consequences

**Positive:**
- Heading stability at zero angular rate — PX4 closes the heading loop.
- Smooth activation — heading seeded from current telemetry, no step.
- Safe timeout behaviour — holds heading on topic drop, does not spin.

**Negative / Trade-offs:**
- Requires PX4 attitude controller tuned for the rover kinematics
  (`RO_YAW_P`, etc.) in addition to the speed controller.
- Integrating `angular.z` accumulates error over time if the topic rate is
  low or jitters; the integration is open-loop with respect to actual heading
  achieved.
- Not suitable for continuous Nav2 path following — Nav2 recomputes `cmd_vel`
  at high frequency, and the cumulative integration of its angular corrections
  drifts from Nav2's intended heading. Use `RoverSpeedSteeringMode` for Nav2.
- Does not implement the FMU reconnect retry loop (unlike `RoverManualMode`).
  A future task should align this with ADR-004.

## Implementation Notes

| File | Role |
|---|---|
| `src/px4_bringup/include/px4_bringup/rover_speed_attitude_mode.hpp` | Mode implementation |
| `src/px4_bringup/src/rover_speed_attitude_main.cpp` | Entry point |

### Coordinate frame detail

| Quantity | ROS frame | PX4 frame | Conversion |
|---|---|---|---|
| `angular.z` input | ENU CCW+ [rad/s] | — | negate → NED CW+ rate |
| `yaw_setpoint_` | — | NED CW+ [rad] | integrated from negated rate |
| `heading()` seed | — | NED CW+ [rad] | read directly from PX4 telemetry |

### Mode selection guide

| Use case | Recommended mode |
|---|---|
| Actuator / wiring validation | RoverManualMode (ADR-004) |
| Nav2 autonomous navigation | RoverSpeedSteeringMode (ADR-005) |
| Teleoperation with heading hold | **RoverSpeedAttitudeMode (this ADR)** |
| Straight-line stability | **RoverSpeedAttitudeMode (this ADR)** |
