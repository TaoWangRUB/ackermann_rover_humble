# PX4 ROS 2 Integration Overview

## Architecture

This project uses **PX4 SITL** running inside **WSL2**, communicating with:

- **MicroXRCEAgent** — bridges PX4 ↔ ROS 2 topics via DDS
- **ROS 2 Humble** — runs custom flight modes and control nodes
- **Gazebo** — provides physics simulation
- **QGroundControl (QGC)** — runs on the **Windows host** for monitoring and manual control

## Running the Stack

### 1. Start PX4 SITL + Gazebo

Launch the full simulation (see `robot_bringup` or run PX4 SITL directly).

### 2. Start MicroXRCEAgent

```bash
MicroXRCEAgent udp4 -p 8888
```

### 3. Run a Custom Mode (e.g. Manual Mode Example)

```bash
ros2 run example_mode_manual_cpp example_mode_manual
```

This registers "My Manual Mode" with PX4 via the PX4 ROS 2 interface library.

---

## QGC Connection from Windows Host (WSL2 Networking)

PX4 SITL inside WSL2 sends MAVLink to `127.0.0.1` by default, which stays inside the WSL VM. QGC on Windows never sees it.

### Fix: Point MAVLink at the Windows Host IP

1. **Get the Windows host IP from WSL:**

   ```bash
   ip route | grep default
   ```

   The IP after `via` is the Windows host (e.g. `172.22.240.1`).

2. **Stop the default MAVLink instance in the PX4 console (`pxh>`):**

   First, find the local port of instance #0 via `mavlink status` (e.g. `18570`), then:

   ```
   mavlink stop -u 18570
   ```

3. **Start a new MAVLink instance targeting the Windows host:**

   ```
   mavlink start -m onboard -r 4000000 -x -u 14550 -o 14550 -t <WIN_IP>
   ```

   Replace `<WIN_IP>` with the IP from step 1 (e.g. `172.22.240.1`).

   > **Note:** The mode must be a valid PX4 mode string. Use `onboard`, `custom`, `minimal`, etc. `normal` is **not** a valid mode and will fail with `ERROR [mavlink] invalid mode`.

4. **Configure QGC on Windows:**

   - Application Settings → Comm Links → Add
   - Type: **UDP**, Port: **14550**
   - Enable "Listen on all interfaces"
   - Ensure Windows Firewall allows QGC on private networks

5. **Verify** with `mavlink status` in the PX4 console — the new instance should show the Windows host IP as the target.

---

## Joystick / RC in QGC (Windows)

QGC only sees joysticks that Windows exposes as a game controller via HID.

- Verify in Windows: `Win+R` → `joy.cpl` → confirm the RC appears and axes respond.
- In QGC: Settings → Joystick → "Enable joystick input".
- Close other apps that may grab the device (Steam, vendor calibration tools).
- Restart QGC after plugging in the RC (some versions only scan at startup).

---

## Custom Mode Arming: `preventArming` Flag

### Problem

When selecting a PX4 ROS 2 custom mode (e.g. "My Manual Mode") and attempting to arm, PX4 denies arming:

```
WARN  [commander] Arming denied: Resolve system health failures first
```

### Cause

All PX4 ROS 2 example modes are created with `preventArming(true)` as a safety guard:

```cpp
// src/px4-ros2-interface-lib/examples/cpp/modes/manual/include/mode.hpp
explicit FlightModeTest(rclcpp::Node& node)
    : ModeBase(node, Settings{kName}.preventArming(true))
```

This flag tells PX4: **do not allow arming while this mode is selected**.

### Solution A: Arm First, Then Switch (Recommended / Safe)

1. Select a standard PX4 mode in QGC (e.g. **Manual** or **Position**).
2. Arm the vehicle.
3. Switch to "My Manual Mode" in the QGC mode selector.

### Solution B: Allow Direct Arming (Development Only)

Change `preventArming(true)` → `preventArming(false)` in the mode header:

```cpp
explicit FlightModeTest(rclcpp::Node& node)
    : ModeBase(node, Settings{kName}.preventArming(false))
```

Then rebuild:

```bash
colcon build --packages-select example_mode_manual_cpp --symlink-install
```

> **Warning:** Only do this in simulation. For real hardware, keep `preventArming(true)` on experimental modes.

---

## PX4 ROS 2 Interface Library — Key Concepts

| Concept | Description |
|---|---|
| `ModeBase` | Base class for custom PX4 flight modes registered via ROS 2 |
| `Settings` | Builder for mode config: name, `preventArming`, `activateEvenWhileDisarmed`, `replaceInternalMode` |
| `ManualControlInput` | Reads RC stick inputs (roll, pitch, yaw, throttle) |
| `RatesSetpointType` | Sends angular rate + thrust commands (experimental) |
| `AttitudeSetpointType` | Sends quaternion attitude + thrust commands (experimental) |
| `PeripheralActuatorControls` | Sends servo/actuator passthrough commands |
| `healthAndArmingChecks()` | Virtual method to report custom arming requirements (default: no-op) |
| Mode requirements | Automatically inferred from which setpoint types are constructed (e.g. needs attitude estimate, needs local position) |

---

## MAVLink Mode Reference

Valid `-m` values for `mavlink start`:

```
custom | camera | onboard | osd | magic | config | iridium | minimal | extvision | extvisionmin | gimbal | onboard_l
```

`normal` is **not** valid and will produce `ERROR [mavlink] invalid mode`.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| QGC not connecting | MAVLink going to WSL loopback | Redirect MAVLink to Windows host IP |
| `invalid mode` on `mavlink start` | Used `normal` instead of valid mode | Use `onboard`, `custom`, etc. |
| `mavlink stop` returns -1 | Wrong argument syntax | Use `-u <local_port>`, not `instance N` |
| Arming denied in custom mode | `preventArming(true)` on the mode | Arm in standard mode first, then switch; or set flag to `false` |
| Joystick not in QGC | Windows doesn't see the controller | Check `joy.cpl`, install driver, restart QGC |
| Hidden modes in QGC | QGC filters unsupported modes per vehicle type | Normal behavior; modes entered programmatically still work |
