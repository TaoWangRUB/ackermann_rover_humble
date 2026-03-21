---
title: "ADR-007: Steering and Wheel Joint Control Across Deployment Scenarios"
status: Accepted
owner: decisions_team
last_updated: 2026-03-21
doc_type: ADR
ros_distro: humble
---

## Context

The Ackermann rover has six non-fixed joints that require active state or command
sources depending on the deployment scenario:

| Joint | Type | Role |
|---|---|---|
| `ackermann/rear_left_wheel_joint` | continuous | rear-left drive wheel spin |
| `ackermann/rear_right_wheel_joint` | continuous | rear-right drive wheel spin |
| `ackermann/front_left_wheel_joint` | continuous | front-left wheel spin (free-rolling) |
| `ackermann/front_right_wheel_joint` | continuous | front-right wheel spin (free-rolling) |
| `ackermann/front_left_wheel_steering_joint` | revolute ±0.6 rad | left kingpin steering angle |
| `ackermann/front_right_wheel_steering_joint` | revolute ±0.6 rad | right kingpin steering angle |

Three deployment scenarios need to be supported:

1. **Gazebo sim + ros2_control** (`enable_px4_sitl:=false`) — ROS 2 controls Gazebo joints
2. **Gazebo sim + PX4 SITL** (`enable_px4_sitl:=true`) — PX4 firmware controls Gazebo joints
3. **Real hardware** (`use_gazebo:=false`) — PX4 drives physical ESC/servo; no Gazebo

Each scenario differs in who commands the joints, what state is fed back to ROS, and
therefore how `robot_state_publisher` receives its `/joint_states` feed.

---

## Decision

### Scenario 1 — Gazebo sim + ros2_control

**Active plugin:** `gz_ros2_control::GazeboSimROS2ControlPlugin`
(`libgz_ros2_control-system.so`), loaded conditionally via `<xacro:unless
enable_px4_sitl>` in the URDF's `<gazebo>` block.

**Hardware abstraction layer:** `gz_ros2_control/GazeboSimSystem` defined in the
`<ros2_control>` block. Command and state interfaces per joint:

| Joint | Command interface | State interfaces |
|---|---|---|
| `rear_left_wheel_joint` | `velocity` | `velocity`, `position` |
| `rear_right_wheel_joint` | `velocity` | `velocity`, `position` |
| `front_left_wheel_joint` | _(none)_ | `velocity`, `position` |
| `front_right_wheel_joint` | _(none)_ | `velocity`, `position` |
| `front_left_wheel_steering_joint` | `position` | `position` |
| `front_right_wheel_steering_joint` | `position` | `position` |

The front wheel spin joints have **state interfaces only**. They are free-rolling;
their angular velocity is determined by physics contact with the ground, not by a
commanded interface.

**Controllers** (`ackermann_controller.yaml`, spawned by `controller_bringup.launch.py`
after `spawn_entity` exits):

- `joint_state_broadcaster` — reads all six interfaces, publishes `/joint_states`
  at 50 Hz.
- `ackermann_steering_controller` — subscribes to `/cmd_vel` (TwistStamped),
  converts to:
  - Rear wheel velocity commands (m/s → rad/s) sent to `rear_left/right_wheel_joint`
  - Front steering position commands (rad) derived from Ackermann geometry sent to
    `front_left/right_wheel_steering_joint`

**`/joint_states` source:** `joint_state_broadcaster` via `ros2_control`.

**Note — Gazebo native Ackermann plugin is intentionally disabled.** The URDF
contains a commented-out `gz-sim-ackermann-steering-system` plugin block. This
native Gazebo plugin bypasses `ros2_control` entirely, provides no
`/joint_states`, and cannot be replaced by a real-hardware driver in the future.
It was evaluated and rejected in favour of `gz_ros2_control`.

---

### Scenario 2 — Gazebo sim + PX4 SITL

**Full signal chain:**

```
/cmd_vel (ROS Twist)
  → rover_speed_steering_mode     (px4_bringup C++ node, px4_ros2_interface_lib)
  → PX4 velocity setpoint         (uORB, internal to PX4 firmware)
  → PX4 rover control loop        (applies RA_STR_RATE_LIM, RO_SPEED_P/I, etc.)
  → actuator allocation           (uORB, internal)
  → gz_bridge GZMixingInterfaceServo / GZMixingInterfaceWheel
      steering: /model/ackermann/servo_0            (Gazebo transport)
      throttle: /model/ackermann/command/motor_speed (Gazebo transport)
  → JointPositionController / JointController plugins  (P-gain, in URDF)
  → Gazebo physics engine         ← simulates joint response with dynamics
  → gz-sim-joint-state-publisher-system  ← reads ACTUAL simulated joint state
  → joint_state_bridge            (ROS, conditional on enable_px4_sitl)
  → /joint_states (ROS)
```

The joint states are **physics-derived**: Gazebo simulates the joint response to
the commanded position/velocity with its dynamics engine. The
`gz-sim-joint-state-publisher-system` then reads back what the physics actually
produced, not what was commanded. This gives realistic joint state feedback
including transient effects from the P-gain controllers.

The Gazebo transport topic names are generated programmatically by `gz_bridge`:
```cpp
// GZMixingInterfaceServo::init()
joint_name  = "servo_" + std::to_string(i);   // → "servo_0", "servo_1", ...
servo_topic = "/model/" + model_name + "/" + joint_name;
// → /model/ackermann/servo_0

// GZMixingInterfaceWheel
wheel_topic = "/model/" + model_name + "/command/motor_speed";
```

These match the `<sub_topic>` values in the URDF's Gazebo plugins exactly.

`enable_px4_sitl:=true` makes two structural changes in the URDF:

1. The `gz_ros2_control` Gazebo plugin is **excluded** (`<xacro:unless
   enable_px4_sitl>`). The `<ros2_control>` hardware block is also excluded by
   the same guard. `controller_bringup.launch.py` is not launched (gated by
   `UnlessCondition(enable_px4_sitl)` in `gazebo_bringup.launch.py`).

2. Six Gazebo joint plugins are **added** (`<xacro:if enable_px4_sitl>`):

**Wheel velocity controllers** — `gz-sim-joint-controller-system`, P-only:

| Joint | Gz topic | Actuator index | p_gain |
|---|---|---|---|
| `front_left_wheel_joint` | `command/motor_speed` | 0 | 10.0 |
| `front_right_wheel_joint` | `command/motor_speed` | 0 | 10.0 |
| `rear_left_wheel_joint` | `command/motor_speed` | 0 | 10.0 |
| `rear_right_wheel_joint` | `command/motor_speed` | 0 | 10.0 |

All four wheels listen to the same actuator index 0. This is a deliberate
simplification: the physical rover is a single-throttle-channel RC car. PX4
sends one throttle value for all drive wheels; there is no per-wheel velocity
differential in the current firmware configuration.

**Steering position controllers** — `gz-sim-joint-position-controller-system`,
P-only:

| Joint | Gz topic | p_gain |
|---|---|---|
| `front_left_wheel_steering_joint` | `servo_0` | 10 |
| `front_right_wheel_steering_joint` | `servo_0` | 10 |

Both steering joints receive the same position command from `servo_0`. PX4's
rover mixer maps the steering channel to a single servo output. Applying it to
both kingpins is geometrically approximate (true Ackermann would require
different angles per side), but the angular difference at small steering angles
is negligible for simulation fidelity at this scale.

**`/joint_states` source:** Gazebo's built-in `gz-sim-joint-state-publisher-system`
(always present in the URDF, publishes on Gazebo transport topic `/joint_states`).
`gazebo_bringup.launch.py` starts a dedicated `joint_state_bridge` node
(conditional on `enable_px4_sitl`) to bridge this into ROS:

```
/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model
```

---

### Scenario 3 — Real hardware

**Full signal chain:**

```
/cmd_vel (ROS Twist)
  → rover_speed_steering_mode     (px4_bringup C++ node)
  → PX4 velocity setpoint         (uORB, internal)
  → PX4 rover control loop
  → actuator allocation           (uORB, internal)
  → PWM/UART to CubePilot outputs
  → physical ESC → brushless motor → rear axle    ← NO feedback to ROS
  → physical servo → front kingpins               ← NO feedback to ROS
```

There is no encoder or resolver on either the drive motor or the steering servo.
Nothing in this chain reports back to ROS, so `/joint_states` has no natural
source. Without it, `robot_state_publisher` silently omits TF frames for all
six non-fixed joints, breaking the TF tree for RTAB-Map, Nav2, and RViz.

**Decision:** `robot_bringup.launch.py` starts `cmd_vel_joint_relay.py`
(conditional on `UnlessCondition(use_gazebo)`), which derives `/joint_states`
from `/cmd_vel` using inverse Ackermann kinematics at 50 Hz.

**`/joint_states` source:** `cmd_vel_joint_relay` (inverse Ackermann from `/cmd_vel`).

Kinematics computed per publish cycle:

```
turn radius:   R = linear.x / angular.z
left  kingpin: δ_L = atan2(L,  R − T/2)     (full per-kingpin, not bicycle approx)
right kingpin: δ_R = atan2(L,  R + T/2)
wheel speed:   ω = linear.x / wheel_radius   (same for all 4 — single throttle channel)
wheel pos:     θ += ω · dt                   (integrated for mesh animation in RViz)
```

**Why `/cmd_vel` rather than `ActuatorServos` / `ActuatorMotors` DDS topics:**

`ActuatorServos` and `ActuatorMotors` are `fmu/in/` topics — they are **inputs
to PX4**, not outputs. They exist so an external ROS node can inject raw actuator
overrides directly into PX4, bypassing the control laws. PX4 does not publish
its internal actuator state outward via these topics; the gz_bridge servo/motor
loop is entirely internal (uORB → `GZMixingInterface*` → Gazebo transport,
no DDS involved).

Even if these were enabled and repurposed as feedback, they would still sit
upstream of the ESC/servo response — "commanded, not measured" — identical in
principle to using `/cmd_vel` but with an additional PX4-DDS dependency and
requiring ESC calibration constants (min/max throttle → rpm, servo range → rad)
that are not tracked in this repository.

`/cmd_vel` is the right proxy because:
- It is already in ROS coordinates with no scaling needed.
- It works when only cameras + Nav2 are running (no PX4 DDS required).
- The only unhandled case is RC/QGC manual control, which bypasses `/cmd_vel`
  entirely; that is not a current navigation scenario.

Camera frames are **fixed joints** in the URDF, so their TF is always correct
regardless of wheel state. SLAM and Nav2 use camera-derived odometry; the wheel
animation in RViz is cosmetic.

---

## Architecture Summary

```
Scenario                     /joint_states source                     Commanded by
─────────────────────────────────────────────────────────────────────────────────────
Gazebo + ros2_control        joint_state_broadcaster (ros2_control)   ackermann_steering_controller ← /cmd_vel
Gazebo + PX4 SITL            Gz JointStatePublisher (bridged)         PX4 gz_bridge (motor_speed + servo_0)
Real hardware                cmd_vel_joint_relay (inv. Ackermann)     PX4 CubePilot → ESC/servo (no ROS feedback)
```

```
Scenario                     enable_px4_sitl    use_gazebo    Gazebo plugin active
──────────────────────────────────────────────────────────────────────────────────
Gazebo + ros2_control        false              true          gz_ros2_control
Gazebo + PX4 SITL            true               true          gz joint controllers
Real hardware                false              false         (none)
```

---

## Consequences

### Accepted trade-offs

- **PX4 SITL uses a single throttle channel for all 4 wheels.** This eliminates
  differential wheel speed (which a real Ackermann rover achieves passively
  through slip). Simulation dynamics are less accurate but acceptable for
  validating navigation algorithms.

- **PX4 SITL applies the same steering angle to both front wheels.** True
  Ackermann geometry requires different left/right angles. At the rover's scale
  (wheelbase = track = 174 mm, max steer ±0.6 rad) the angular error is small.

- **Real hardware has no wheel state in ROS.** Wheel odometry is unavailable.
  All pose estimation relies on visual/depth camera odometry. This is the
  system's intended design — cameras are the primary odometry source.

- **The Gazebo native Ackermann plugin remains commented out.** It should not be
  re-enabled: it lacks `ros2_control` abstraction, provides no `/joint_states`,
  and prevents future hardware-in-the-loop testing with the same controller
  parameters.

### Known limitations

- All PX4 SITL wheel joints share actuator index 0. If future PX4 firmware adds
  per-wheel velocity control (e.g., for traction control), the URDF plugins will
  need separate actuator indices and dedicated `JointController` instances.

---

## Related files

| File | Role |
|---|---|
| [ackermann_rover.urdf](../../src/description_robot/models/ackermann_rover/ackermann_rover.urdf) | Joint definitions, ros2_control block, Gazebo plugins |
| [ackermann_controller.yaml](../../src/description_robot/config/ackermann_controller.yaml) | Controller manager + ackermann_steering_controller params |
| [controller_bringup.launch.py](../../src/description_robot/launch/controller_bringup.launch.py) | Spawns joint_state_broadcaster + ackermann_steering_controller |
| [gazebo_bringup.launch.py](../../src/description_robot/launch/gazebo_bringup.launch.py) | Starts Gazebo, RSP, parameter_bridge, controller callback, joint_state_bridge |
| [robot_bringup.launch.py](../../src/robot_bringup/launch/robot_bringup.launch.py) | Top-level: selects scenario via use_gazebo / enable_px4_sitl |
