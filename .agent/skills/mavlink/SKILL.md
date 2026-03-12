---
name: MAVLink and PX4 Shell Commands
description: PX4 NSH console commands for MAVLink management, parameter tuning, sensor debugging, actuator testing, and DDS bridge operation.
---

# MAVLink and PX4 Shell Commands Skill

This skill covers the PX4 NSH shell (accessible via QGC MAVLink Console or SITL terminal) and the Micro-XRCE-DDS agent commands used to bridge PX4 to ROS 2. Use these commands for hardware bring-up, diagnostics, and debugging.

## 1. MAVLink Instance Management

MAVLink instances handle telemetry streams between PX4 and ground stations (QGC) or companion computers.

```bash
# Check all running MAVLink instances (ports, modes, rates)
mavlink status

# Stop a running instance by its local UDP port
mavlink stop -u <LOCAL_PORT>          # e.g., mavlink stop -u 18570

# Start a new MAVLink instance
mavlink start -m <MODE> -r <BAUDRATE> -x -u <LOCAL_PORT> -o <REMOTE_PORT> -t <TARGET_IP>
```

**Valid modes**: `onboard`, `custom`, `minimal`, `extvision`, `extvisionmin`, `gimbal`, `onboard_l`, `camera`, `osd`, `magic`, `config`, `iridium`

> **WARNING**: `normal` is NOT a valid mode — it will produce `ERROR [mavlink] invalid mode`.

**Example — Redirect QGC to Windows host from WSL2:**
```bash
# Find Windows host IP
ip route | grep default              # IP after "via"

# Stop default instance, start one targeting Windows
mavlink stop -u 18570
mavlink start -m onboard -r 4000000 -x -u 14550 -o 14550 -t 172.22.240.1
```

## 2. Parameter Management

PX4 parameters persist across reboots and control all subsystem behavior.

```bash
# Query a parameter
param show <NAME>                    # e.g., param show EKF2_EV_CTRL

# Set a parameter (takes effect immediately, persists)
param set <NAME> <VALUE>

# Common rover parameters
param show SYS_AUTOSTART             # Airframe number (51000 for ackermann)
param show RA_WHEEL_BASE             # Wheelbase in meters
param show RA_MAX_STR_ANG            # Max steering angle in degrees

# Disable RC requirement (SITL without joystick)
param set COM_RC_IN_MODE 4

# EKF2 vision fusion
param set EKF2_EV_CTRL 15            # Fuse position + velocity + yaw
param set EKF2_HGT_REF 3             # Height reference = vision
param set EKF2_GPS_CTRL 0            # Disable GPS (indoor)

# UXRCE-DDS serial port selection (hardware)
param show UXRCE_DDS_CFG             # 0=disabled, 101=TELEM1, 102=TELEM2, 104=TELEM4, 201=GPS1
param set UXRCE_DDS_CFG 102          # Use TELEM2

# Namespace isolation (hardware vs SITL)
#param set UXRCE_DDS_NS px4_hw        # Topics become /px4_hw/fmu/*

# PWM function mapping
param set PWM_MAIN_FUNC1 101         # MAIN 1 = Throttle
param set PWM_MAIN_FUNC3 201         # MAIN 3 = Steering
```

## 3. Commander — Arming and Mode Control

```bash
# Arm / Disarm
commander arm
commander disarm

# Set flight mode
commander mode auto:loiter            # Hold mode (passes preflight without RC)
commander mode manual
commander mode offboard               # Offboard control via ROS 2

# From SITL binary directory
./px4-commander arm
./px4-commander disarm
./px4-commander mode auto:loiter
```

## 4. Listener — uORB Topic Monitoring

The `listener` command streams internal uORB data. Essential for verifying sensor fusion, actuator output, and estimator health.

```bash
# Syntax: listener <topic> [-n <count>]
listener vehicle_local_position       # EKF position estimate
listener vehicle_attitude             # Roll/pitch/yaw quaternion
listener vehicle_visual_odometry      # External vision data arriving
listener vehicle_status               # Arming state, nav state
listener sensor_accel                 # Raw accelerometer
listener sensor_gyro                  # Raw gyroscope
listener estimator_status_flags       # EKF fusion flags (cs_* and fs_*)
listener estimator_innovations        # Prediction errors (innovation magnitudes)
listener actuator_outputs             # PWM/motor commands
listener battery_status               # Voltage, current, remaining

# Single sample with timeout (from SITL binary dir)
./px4-listener sensor_accel -n 1
./px4-listener vehicle_local_position -n 1
```

**EKF2 Health Checks:**
- `estimator_status_flags`: Look for `cs_ev_hpos=True`, `cs_ev_vpos=True`, `cs_ev_yaw=True`
- `estimator_innovations`: Healthy if `ev_hpos < 0.05m`, `gps_hpos < 0.5m`, `heading < 0.01 rad`

## 5. Actuator Testing

Test motors and servos individually **while disarmed**.

```bash
# IMPORTANT: Vehicle MUST be disarmed for actuator_test
commander disarm

# Motor test: -m <index> -v <value -1..1> -t <duration_sec>
actuator_test set -m 1 -v 0.5 -t 3        # Motor 1 at 50% for 3 sec

# Servo test: -s <index> -v <value -1..1> -t <duration_sec>
actuator_test set -s 1 -v 0.5 -t 3        # Servo 1 at 50% for 3 sec

# Iterate through all motors/servos sequentially
actuator_test iterate-motors
actuator_test iterate-servos

# From SITL binary dir
./px4-actuator_test set -m 1 -v 0.5 -t 3
```

## 6. System Diagnostics

```bash
# System load (CPU, memory, stack usage per task)
top

# uORB topic publication rates
uorb top

# Kernel message log (driver init, errors)
dmesg

# Sensor status summary
sensors status

# Check hardware version
ver all
```

## 7. Micro-XRCE-DDS Agent

The DDS agent bridges PX4's `uxrce_dds_client` module to the ROS 2 DDS network. **Must be started BEFORE PX4** — it does not reliably reconnect if started after.

```bash
# UDP mode (SITL — default)
./scripts/start_microxrce_agent.sh                         # port 8888
./scripts/start_microxrce_agent.sh 8889                    # custom port

# Serial mode (real hardware)
./scripts/start_microxrce_agent.sh --serial                # /dev/ttyUSB0 @ 921600
./scripts/start_microxrce_agent.sh --serial /dev/ttyACM0 115200

# Direct binary invocation (inside Docker)
export LD_LIBRARY_PATH=/opt/microxrce/lib:${LD_LIBRARY_PATH:-}
/opt/microxrce/bin/MicroXRCEAgent udp4 -p 8888
/opt/microxrce/bin/MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

**DDS Topic Conventions:**
- PX4 publications → `/fmu/out/<topic>` (ROS 2 subscribes)
- PX4 subscriptions → `/fmu/in/<topic>` (ROS 2 publishes)
- With namespace: `/px4_hw/fmu/out/<topic>`, `/px4_hw/fmu/in/<topic>`

## 8. Serial Interface Identification (Hardware)

When connecting a companion computer to CubeBlack, identify which TELEM port is used for what:

```bash
# Check MAVLink port assignments
param show MAV_0_CONFIG               # Main MAVLink instance (usually QGC)
param show MAV_1_CONFIG               # Second instance
param show MAV_2_CONFIG               # Third instance

# Check DDS port assignment
param show UXRCE_DDS_CFG              # 102 = TELEM2

# Port number mapping:
# 101 = TELEM1, 102 = TELEM2, 104 = TELEM4, 201 = GPS1

# Verify via mavlink status output
mavlink status                        # Shows "device: /dev/ttyS<N>" per instance
```

## 9. Common Debugging Workflows

### Vision Odometry Not Fusing
```bash
param show EKF2_EV_CTRL               # Should be 15
listener vehicle_visual_odometry       # Is data arriving?
listener estimator_status_flags        # cs_ev_hpos should be True
listener estimator_innovations         # Check ev_hpos magnitude
```

### Robot Not Moving in Offboard
```bash
listener offboard_control_mode         # Are offboard commands arriving?
listener vehicle_status                # Check nav_state, arming_state
mavlink status                         # Is companion link active?
param show COM_RC_IN_MODE              # Should be 4 for no-RC mode
```

### High CPU on CubeBlack
```bash
top                                    # Check uxrce_dds_client CPU usage
uorb top                               # Check topic publication rates
# Solution: Trim dds_topics.yaml to only needed topics (~24 instead of ~45)
```
