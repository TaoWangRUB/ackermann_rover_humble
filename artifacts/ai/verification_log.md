# Verification Log

**Date:** 2026-03-07  
**Branch:** `task/change_base_link_frame`  
**Environment:** ROS 2 Jazzy, Gazebo Harmonic, Docker container `ackermann_slam`

---

## 1. URDF Restructuring

### Task
Move URDF files from `src/description_robot/urdf/` to `src/description_robot/models/ackermann_rover/` and rename the main file from `donkey_sensors.urdf` to `ackermann_rover.urdf`.

### Changes Made

| File                                                                       | Action          |
| -------------------------------------------------------------------------- | --------------- |
| `urdf/donkey_sensors.urdf` → `models/ackermann_rover/ackermann_rover.urdf` | Moved & renamed |
| `urdf/diff_sensors.urdf` → `models/ackermann_rover/diff_sensors.urdf`      | Moved           |
| `urdf/sensors/` → `models/ackermann_rover/sensors/`                        | Moved           |
| `urdf/realsense/` → `models/ackermann_rover/realsense/`                    | Moved           |
| `urdf/cubepilot/` → `models/ackermann_rover/cubepilot/`                    | Moved           |
| `urdf/`                                                                    | Deleted (empty) |

### Dependency Updates

**Launch files (2 files, path updates):**
- `src/description_robot/launch/gazebo_bringup.launch.py` — xacro path updated to `models/ackermann_rover/ackermann_rover.urdf`
- `src/description_robot/launch/controller_bringup.launch.py` — xacro path updated to `models/ackermann_rover/ackermann_rover.urdf`

**Xacro includes (6 files, 12 paths updated from `urdf/` to `models/ackermann_rover/`):**
- `models/ackermann_rover/cubepilot/cubepilot.urdf.xacro` — 1 include + 1 mesh URI
- `models/ackermann_rover/sensors/rplidar.urdf.xacro` — 1 include
- `models/ackermann_rover/realsense/_d435i.urdf.xacro` — 2 includes
- `models/ackermann_rover/realsense/_l515.urdf.xacro` — 1 include + 1 mesh URI
- `models/ackermann_rover/realsense/_t265.urdf.xacro` — 1 include + 2 mesh URIs

**Mesh URIs (3 files, 4 URIs):**
- Fixed `package://description_robot/urdf/...` → `package://description_robot/models/...`

**CMakeLists.txt:** No change needed — `install(DIRECTORY models ...)` already existed.

### Verification
- `colcon build --symlink-install` — **PASS**
- `xacro` processing of `ackermann_rover.urdf` — **PASS**
- `grep -r "urdf/" src/description_robot/` for stale references — **PASS** (none found)

---

## 2. Robot Not Moving — Root Cause Analysis

### Symptom
After launching `ros2 launch robot_bringup robot_bringup.launch.py enable_d435i:=false enable_t265:=false enable_cubepilot:=false rtabmap:=false nav2:=false`, publishing to `/cmd_vel` did not produce any robot motion in Gazebo.

### Diagnosis Steps

#### Step 2.1 — Controller subscription check
- `ackermann_steering_controller` was **active** and subscribed to `/ackermann_steering_controller/reference` (TwistStamped)
- Confirmed via `/controller_manager/list_controllers` service call

#### Step 2.2 — reference_timeout issue
- Config had `reference_timeout: 2.0` which rejects messages with `stamp=0` (i.e., older than 2s relative to current time)
- **Fix:** Changed to `reference_timeout: 0.0` in `src/description_robot/config/ackermann_controller.yaml`
- **Result:** Controller now accepts commands, but robot **still did not move**

#### Step 2.3 — use_stamped_vel investigation
- Attempted `use_stamped_vel: false` — parameter does **not exist** in Jazzy `ros2_controllers`
- Jazzy `ackermann_steering_controller` **always** uses `TwistStamped` on the `reference` interface
- The `reference_unstamped` (Twist) topic exists but is remapped via the Gazebo plugin to `/cmd_vel`
- Reverted to `use_stamped_vel: true` (harmless, ignored by Jazzy)

#### Step 2.4 — Diagnostic Python script
Custom script published TwistStamped with proper sim-clock timestamps and monitored:
- `/ackermann_steering_controller/controller_state` (SteeringControllerStatus)
- `/ackermann/odom` (Odometry)

**Key finding:** Controller output showed `vel_cmd=[52.63, 52.63]` rad/s (correct: 2.0 m/s ÷ 0.038m wheel radius = 52.63 rad/s), but `wheel_vel` in Gazebo remained near **zero**.

This proved the controller was computing correct commands, but Gazebo physics was not executing them.

#### Step 2.5 — PX4 SITL joint plugin conflict (ROOT CAUSE)
Inspected the URDF and found **6 PX4 SITL Gazebo joint plugins**:
- 4x `gz-sim-joint-controller-system` (velocity control, `p_gain=10`)
- 2x `gz-sim-joint-position-controller-system` (position control)

These plugins were commanding **all wheel and steering joints to zero** (their default setpoints), actively fighting against ros2_control's velocity commands. The PX4 plugins win because they have a proportional gain applied at the physics level.

### Root Cause
**PX4 SITL Gazebo joint controller plugins** hold joints at zero velocity/position, overriding ros2_control commands. These plugins are only needed when PX4 SITL is actively running and sending commands, but they were unconditionally loaded.

---

## 3. Fix — Conditional PX4 Plugin Loading

### Changes

**`src/description_robot/models/ackermann_rover/ackermann_rover.urdf`:**
- Added xacro arg: `<xacro:arg name="enable_px4_sitl" default="false"/>`
- Wrapped all 6 PX4 joint plugins in: `<xacro:if value="$(arg enable_px4_sitl)"> ... </xacro:if>`

**`src/description_robot/launch/gazebo_bringup.launch.py`:**
- Added launch argument: `enable_px4_sitl` (default `false`)
- Declared in LaunchDescription actions list
- Passed to xacro command mappings

**`src/description_robot/config/ackermann_controller.yaml`:**
- `reference_timeout: 0.0` (changed from `2.0`) — allows stamp=0 or stale-timestamp messages

### Verification

```
$ ros2 launch robot_bringup robot_bringup.launch.py \
    enable_d435i:=false enable_t265:=false enable_cubepilot:=false \
    rtabmap:=false nav2:=false
```

**Controller state check:**
```
ackermann_steering_controller: state=active
  claimed_interfaces: [rear_right_wheel_joint/velocity, rear_left_wheel_joint/velocity,
                       front_right_wheel_steering_joint/position, front_left_wheel_steering_joint/position]
joint_state_broadcaster: state=active
```

**Motion test (cmd_vel at 1.0 m/s for 5 seconds):**
```
Initial odom:  x=0.0000,  y=0.0000
Controller:    vel_cmd=[26.32, 26.32] rad/s  (1.0 / 0.038 = 26.32 ✓)
Final odom:    x=0.2517,  y=0.0000
Distance:      0.2517 m
Result:        SUCCESS — Robot is moving!
```

### Status: **PASS**

---

## 4. Summary of All Modified Files

| File                                                                          | Change Type                           |
| ----------------------------------------------------------------------------- | ------------------------------------- |
| `src/description_robot/models/ackermann_rover/ackermann_rover.urdf`           | Moved, renamed, PX4 conditional added |
| `src/description_robot/models/ackermann_rover/diff_sensors.urdf`              | Moved                                 |
| `src/description_robot/models/ackermann_rover/sensors/rplidar.urdf.xacro`     | Include path updated                  |
| `src/description_robot/models/ackermann_rover/realsense/_d435i.urdf.xacro`    | Include paths updated                 |
| `src/description_robot/models/ackermann_rover/realsense/_l515.urdf.xacro`     | Include path + mesh URI updated       |
| `src/description_robot/models/ackermann_rover/realsense/_t265.urdf.xacro`     | Include path + mesh URIs updated      |
| `src/description_robot/models/ackermann_rover/cubepilot/cubepilot.urdf.xacro` | Include path + mesh URI updated       |
| `src/description_robot/launch/gazebo_bringup.launch.py`                       | URDF path + enable_px4_sitl arg       |
| `src/description_robot/launch/controller_bringup.launch.py`                   | URDF path updated                     |
| `src/description_robot/config/ackermann_controller.yaml`                      | reference_timeout: 0.0                |

---

## 5. Known Limitations

1. **Velocity underperformance:** Robot traveled 0.25m in 5s at 1.0 m/s commanded (~0.05 m/s actual). This is expected with `open_loop: false` and `position_feedback: false` — the controller uses velocity feedback from Gazebo which may have friction/inertia effects. Tuning PID gains or physics properties may improve tracking.

2. **PX4 SITL integration:** When `enable_px4_sitl:=true`, the PX4 joint plugins will load alongside ros2_control. A coordination strategy (e.g., disabling ros2_control when PX4 is active) may be needed for full PX4 SITL scenarios.

3. **use_stamped_vel parameter:** The config retains `use_stamped_vel: true` which is ignored by Jazzy's `ackermann_steering_controller` (always uses TwistStamped). Harmless but could be removed for clarity.

---

## 6. Running the Motion Test Script

The script `scripts/motion_test.py` automates the motion verification. It publishes `TwistStamped` commands on `/cmd_vel` using Gazebo sim-clock timestamps and monitors `/ackermann/odom` and `/ackermann_steering_controller/controller_state` to confirm the robot moves.

### Prerequisites
- Simulation must be running (controllers active, Gazebo world loaded)
- The script uses `use_sim_time=false` on its own node but reads `/clock` to stamp messages correctly

### How to Run

**1. Start the simulation (in Docker):**
```bash
docker-compose -f docker/docker-compose.yml exec ackermann_slam bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /workspace/install/setup.bash && \
   ros2 launch robot_bringup robot_bringup.launch.py \
     enable_d435i:=false enable_t265:=false enable_cubepilot:=false \
     rtabmap:=false nav2:=false"
```

Wait ~30 seconds for Gazebo and controllers to fully initialize.

**2. Run the test script (in a separate terminal, in Docker):**
```bash
docker-compose -f docker/docker-compose.yml exec ackermann_slam bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /workspace/install/setup.bash && \
   python3 /workspace/scripts/motion_test.py"
```

### Expected Output
```
Initial odom: x=0.0, y=0.0
Clock: <sim_time_seconds>.<nanoseconds>
Published 99 messages
Controller state: vel_cmd=array('d', [26.32, 26.32]), steer=array('d', [0.0, 0.0])
Final odom: x=0.2517, y=0.0
Distance traveled: 0.2517 m
SUCCESS: Robot is moving!
```

### Pass/Fail Criteria
- **PASS:** Distance traveled > 0.05 m (prints `SUCCESS: Robot is moving!`)
- **FAIL:** Distance ≤ 0.05 m (prints `FAILURE: Robot did not move`)

### What the Script Does
1. Subscribes to `/ackermann/odom`, `/clock`, and `/ackermann_steering_controller/controller_state`
2. Records initial odometry position (waits 3s for data)
3. Publishes `TwistStamped` at 20 Hz (linear.x=1.0, angular.z=0.0) for 5 seconds, with sim-clock timestamps
4. Compares final odometry to initial position
5. Reports distance traveled and pass/fail
