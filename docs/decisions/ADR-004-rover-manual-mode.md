---
title: "ADR-004: RoverManualMode — Open-Loop Throttle/Steering via px4-ros2-interface-lib"
status: Accepted
owner: taowang
last_updated: 2026-03-16
doc_type: ADR
ros_distro: humble
---

## Context

The rover needs a low-level PX4 custom mode that maps `cmd_vel` directly to
normalized throttle and steering without engaging any PX4 closed-loop
controller (speed PID, heading hold, etc.). This is required for:

- Open-loop functional testing before PID tuning
- Manual teleoperation (joystick, keyboard)
- Integration testing of the actuator path independent of Nav2 or EKF2

The px4-ros2-interface-lib (Auterion) provides a `ModeBase` class for
registering custom modes with the PX4 FMU over DDS/XRCE.

## Decision

Implement `RoverManualMode` as a `px4_ros2::ModeBase` subclass that:

1. **Bypasses all PX4 controllers** — uses `RoverThrottleSteeringSetpointType`
   to write normalized throttle and steering ∈ [-1, 1] directly to
   the actuator allocation layer (`AckermannActControl`).

2. **Maps `cmd_vel` linearly**, with throttle range controlled by `bidirectional_esc`:
   - `linear.x / max_speed` → throttle, clamped to **[-1, 1]** if `bidirectional_esc=true`
     (supports reverse), or **[0, 1]** if `bidirectional_esc=false` (unidirectional ESC only)
   - `-angular.z / max_steering_rate` → steering (negated: ROS CCW+ → PX4 right+)

3. **Implements a cmd_vel watchdog** — if no message arrives within
   `cmd_vel_timeout` seconds after the first message, the setpoint is zeroed.
   Before the first message is ever received, the setpoint is also zero.

4. **Does not call `setSkipMessageCompatibilityCheck()`** — the FMU heartbeat
   check (`waitForFMU`, 60 s) runs at startup to confirm PX4 is alive before
   registration.

5. **Persistent reconnect in `main()`** — rather than crashing on FMU timeout
   or runtime disconnect, `rover_manual_main.cpp` wraps the node in a retry
   loop with `rclcpp::init()` per iteration (required because the library's
   watchdog calls `rclcpp::shutdown()` before throwing).

## Alternatives Considered

### A. `RoverSpeedSteeringMode` (existing)
Maps `linear.x` → speed setpoint, `angular.z` → steering rate. Goes through
PX4's speed PID controller. Rejected for this mode because it requires tuned
gains and is unsuitable for raw open-loop testing.

### B. `RoverSpeedAttitudeMode` (existing)
Adds heading hold on top of speed control. Rejected for the same reason —
adds controller dependency not wanted at this layer.

### C. Skip FMU check (`setSkipMessageCompatibilityCheck`)
Would allow the node to start immediately without waiting for PX4. Rejected
because it can hide DDS/XRCE misconfiguration and leaves the registration
service call to fail silently later. The 60 s wait is acceptable for startup;
the retry loop handles the "not yet available" case without skipping checks.

### D. Crash and rely on a process supervisor (systemd / ros2 launch respawn)
Simpler but adds an external dependency and introduces a full process restart
(cold init) on every disconnect. Rejected in favour of the in-process retry
loop, which re-registers without restarting the OS process.

## Consequences

**Positive:**
- Zero controller dependencies — works without any PX4 parameter tuning.
- Consistent with px4-ros2-interface-lib conventions (no library fork needed).
- Node survives FMU restarts automatically via the retry loop.
- cmd_vel watchdog prevents runaway on topic drop.

**Negative / Trade-offs:**
- Open-loop only — no speed or heading feedback. Suitable only for testing and
  direct teleoperation; Nav2 or higher-level nodes must switch to a controller
  mode for autonomous operation.
- 60 s startup wait if PX4 is not running (mitigated: retry loop, no crash).
- Re-registration after reconnect incurs ~1–5 s delay (waitForFMU + service
  handshake).

## Implementation Notes

| File | Role |
|---|---|
| `src/px4_bringup/include/px4_bringup/rover_manual_mode.hpp` | Mode implementation |
| `src/px4_bringup/src/rover_manual_main.cpp` | Entry point with retry loop |
| `src/px4_bringup/config/px4_bridge.yaml` | Default parameters (incl. `bidirectional_esc`) |
| `src/px4_bringup/launch/px4_bringup.launch.py` | Launch arg `reversible_drive` → `bidirectional_esc` |
| `src/px4-ros2-interface-lib/.../wait_for_fmu.cpp` | FMU heartbeat check (unmodified) |
| `src/px4-ros2-interface-lib/.../health_and_arming_checks.cpp` | Runtime watchdog, 4 s (unmodified) |

### ESC type configuration

`bidirectional_esc` (default: `false`) controls the throttle output range:

| `bidirectional_esc` | Throttle range | ESC wiring |
|---|---|---|
| `false` | [0, 1] | Unidirectional — PWM_MIN = stop, PWM_MAX = full forward |
| `true`  | [-1, 1] | Bidirectional — centre = stop, extremes = full reverse/forward |

Set via launch arg (preferred): `reversible_drive:=true/false` in `px4_bringup.launch.py`.
The same arg is accepted by `nav2_bringup.launch.py` to align Nav2's `vx_min` and
`min_velocity` with the ESC capability.

### Retry loop rationale (`rover_manual_main.cpp`)

The library's `HealthAndArmingChecks` watchdog calls `rclcpp::shutdown()`
before throwing `px4_ros2::Exception` on FMU disconnect. This tears down the
entire ROS 2 context, so `rclcpp::init()` must be called again before creating
a new node. Placing `rclcpp::init()` inside the while loop ensures a clean
context on every attempt. Normal SIGINT is handled by the `break` after
`rclcpp::spin()` returns without throwing.
