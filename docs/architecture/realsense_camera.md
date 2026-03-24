# RealSense Camera System — `realsense_camera_bringup`

Custom ROS 2 wrapper for Intel RealSense D435i, L515, and T265 cameras built on **librealsense2 v2.51.1**.

## Why a Custom Wrapper

The official `realsense-ros` package (v4.51+) depends on librealsense2 2.55+, which **dropped L515 support**. This wrapper uses librealsense2 v2.51.1 installed to `/usr/local`, keeping L515 functional alongside D435i and T265.

## SDK & Build

| Component | Version | Location |
|-----------|---------|----------|
| librealsense2 | 2.51.1 | `/usr/local` (manually installed) |
| ROS 2 | Jazzy | Docker container (`ackermann_slam`) |
| C++ Standard | C++17 | Required by ROS 2 Jazzy |

CMakeLists.txt forces the local SDK:
```cmake
find_package(realsense2 REQUIRED PATHS /usr/local NO_DEFAULT_PATH)
```

ROS dependencies: `rclcpp`, `sensor_msgs`, `nav_msgs`.

## Architecture

### Node Lifecycle

```
Constructor
  declare_parameters()   — parse all params, profile string overrides
  create_publishers()    — conditional on enabled streams
  reset_device()         — hardware_reset() + poll for device reappearance (10s timeout)
  start_pipeline()       — configure streams, start with callback, fallback to auto-profiles
    build CameraInfo     — intrinsics from stream profiles (once at startup)
    init align_to_color_ — rs2::align if depth alignment enabled
    apply_sensor_options()— exposure/gain/auto_exposure per sensor
    start_imu_sensor()   — direct sensor API (NOT via pipeline)

Destructor
  running_ = false
  watchdog thread (5s → std::_Exit)
  stop IMU sensor, then pipeline
```

### Key Design Decisions

**IMU via direct sensor API, not pipeline.** The pipeline's sync module buffers video frames waiting for IMU timestamps, causing latency. IMU is opened independently via `rs2::sensor.open()` + `sensor.start()` with its own callback.

**Hardware reset disabled by default (`enable_hardware_reset=false`).** Matches official realsense-ros behaviour (`initial_reset=false`). Set to `true` only when the device firmware is in a stuck-streaming state from a previous crashed session. For T265, `unload_tracking_module()` is called before reset to release the firmware's USB handle cleanly.

**Retry loop on init (5 attempts, 5s apart).** Transient USB errors (`Unable to open device`, `failed to set power state`, `No device connected`, `device is streaming`) are retried rather than crashing the node.

**`startup_delay_s` parameter for USB sequencing.** When multiple cameras share a USB hub, their simultaneous `hardware_reset()` calls cause USB power-state races. The script automatically applies `d435i_startup_delay_s:=12` when `--t265` is used, so T265 resets and starts streaming before D435i opens.

**Auto-profile fallback.** If the requested resolution/fps isn't supported (e.g. L515 quirks), the pipeline retries with `width=0, height=0, fps=0` so librealsense picks the closest valid profile.

**5-second watchdog in destructor.** Prevents the process from hanging if `pipe_.stop()` blocks on a stuck USB device.

### Frame Callback (`on_frame`)

```
on_frame(rs2::frame)
├─ frameset (synced color+depth from pipeline)
│   ├─ align depth → color (if enabled)
│   ├─ publish color
│   ├─ publish raw depth
│   ├─ publish aligned depth (if enabled)
│   ├─ publish infrared 1/2 (if enabled)
│   └─ publish fisheye 1/2 (T265 + enable_fisheye)
├─ motion_frame (from direct IMU sensor)
│   ├─ accel → cache in last_accel_frame_
│   └─ gyro + cached accel → publish fused IMU
└─ pose_frame (T265)
    └─ publish odometry
```

### Sensor Options

Applied after pipeline start via `apply_sensor_options()`:

1. Identify color vs depth sensor by iterating `dev.query_sensors()` and checking stream types
2. Set `RS2_OPTION_ENABLE_AUTO_EXPOSURE` first (must be set before manual values)
3. If auto_exposure off and value > 0: set `RS2_OPTION_EXPOSURE` and `RS2_OPTION_GAIN`
4. Each call guarded by `sensor.supports()` + try/catch (best-effort)

## Published Topics

All topics prefixed with `{camera_name}/` (e.g. `d435i/color/image_raw`).

| Topic | Type | Condition |
|-------|------|-----------|
| `color/image_raw` | `sensor_msgs/Image` (RGB8) | `enable_color` |
| `color/camera_info` | `sensor_msgs/CameraInfo` | `enable_color` |
| `depth/image_rect_raw` | `sensor_msgs/Image` (16UC1) | `enable_depth` |
| `depth/camera_info` | `sensor_msgs/CameraInfo` | `enable_depth` |
| `aligned_depth_to_color/image_raw` | `sensor_msgs/Image` (16UC1) | `align_depth.enable` + color + depth |
| `aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | (same) |
| `infra1/image_rect_raw` | `sensor_msgs/Image` (MONO8) | `enable_infra1` |
| `infra2/image_rect_raw` | `sensor_msgs/Image` (MONO8) | `enable_infra2` |
| `imu` | `sensor_msgs/Imu` | `enable_imu` or `enable_accel`+`enable_gyro` |
| `odom` | `nav_msgs/Odometry` | T265 only (always) |
| `fisheye1/image_raw` | `sensor_msgs/Image` (MONO8) | T265 + `enable_fisheye` |
| `fisheye2/image_raw` | `sensor_msgs/Image` (MONO8) | T265 + `enable_fisheye` |

CameraInfo topics mirror each image topic. CameraInfo is built once at startup from `rs2::video_stream_profile::get_intrinsics()` (plumb_bob distortion model, 5 coefficients).

## Launch File Structure

**File:** `src/realsense_camera_bringup/launch/realsense_camera.launch.py`

Three `realsense_camera_node` instances gated by `IfCondition`:

```
enable_d435i (default: true)  → D435i node
enable_l515  (default: false) → L515 node
enable_t265  (default: false) → T265 node
```

Each camera has its own prefixed launch arguments (e.g. `d435i_rgb_exposure`, `l515_serial_no`, `t265_enable_fisheye`). Parameters are resolved in order:

1. **Base YAML** (`config/realsense_params.yaml`) — lowest priority
2. **Launch function** (`_depth_camera_params()` / T265 inline dict) — mid priority
3. **CLI launch args** (`:=` syntax) — highest priority

### Global Arguments

| Argument | Default |
|----------|---------|
| `enable_d435i` | `true` |
| `enable_l515` | `false` |
| `enable_t265` | `false` |
| `publish_tf` | `false` |
| `enable_sync` | `true` |
| `unite_imu_method` | `2` |

### D435i Arguments (prefix: `d435i_`)

| Argument | Default | Notes |
|----------|---------|-------|
| `d435i_camera_name` | `d435i` | |
| `d435i_serial_no` | `""` | Empty = first matching device |
| `d435i_device_type` | `""` | Substring match on device name |
| `d435i_enable_color` | `true` | |
| `d435i_enable_depth` | `true` | |
| `d435i_enable_infra1` | `false` | |
| `d435i_enable_infra2` | `false` | |
| `d435i_enable_imu` | `true` | Enables both accel + gyro |
| `d435i_color_width` | `640` | |
| `d435i_color_height` | `480` | |
| `d435i_color_fps` | `30` | |
| `d435i_depth_width` | `640` | |
| `d435i_depth_height` | `480` | |
| `d435i_depth_fps` | `30` | |
| `d435i_color_profile` | `""` | WxHxFPS override (e.g. `640x480x60`) |
| `d435i_depth_profile` | `""` | WxHxFPS override |
| `d435i_rgb_auto_exposure` | `false` | Manual exposure by default |
| `d435i_rgb_exposure` | `200` | Microseconds |
| `d435i_rgb_gain` | `128` | |
| `d435i_depth_auto_exposure` | `false` | |
| `d435i_depth_exposure` | `7500` | Microseconds (7.5ms) |
| `d435i_depth_gain` | `16` | |
| `d435i_align_depth` | `true` | |

### L515 Arguments (prefix: `l515_`)

Same structure as D435i. Key differences:
- `l515_enable_imu`: `false` (L515 has IMU but disabled by default)
- `l515_rgb_auto_exposure`: `true` (auto exposure by default)
- `l515_rgb_exposure`: `0`, `l515_depth_exposure`: `0` (device defaults)

### T265 Arguments (prefix: `t265_`)

| Argument | Default | Notes |
|----------|---------|-------|
| `t265_camera_name` | `t265` | |
| `t265_serial_no` | `""` | |
| `t265_enable_imu` | `true` | |
| `t265_enable_fisheye` | `false` | Odom-only by default |
| `t265_fisheye_fps` | `30` | |

T265 ignores all color/depth/infra/alignment parameters.

## Multi-Camera Launch & USB Sequencing

### The Problem: USB Power-State Races

When multiple RealSense cameras share the same USB hub, simultaneous `hardware_reset()` calls compete for USB power-state control. The symptom is one or both cameras failing with:

```
failed to set power state
Unable to open device interface
```

The root cause is that librealsense resets the USB device during `hardware_reset()`, which briefly interrupts power to all devices on the hub. If two cameras reset simultaneously, each disrupts the other's re-enumeration.

### Solution: Sequential Startup with `startup_delay_s`

Each camera node accepts a `startup_delay_s` parameter (default `0.0`) that sleeps before calling `reset_device()` or `start_pipeline()`. This serialises initialisation without requiring hardware changes.

**Correct startup order for D435i + T265 on a shared USB hub:**

| Time | Camera | Action |
|------|--------|--------|
| 0s | T265 | `hardware_reset()` + wait for re-enumeration (~8s) + `start_pipeline()` |
| 12s | D435i | `start_pipeline()` directly (no reset needed) |

T265 resets first because it most commonly gets stuck in a previous streaming state after a crash. D435i does **not** reset — librealsense re-opens it cleanly when the previous ROS session has exited.

### `enable_hardware_reset` Parameter

| Value | Default for | When to use |
|-------|------------|-------------|
| `false` | D435i, L515 | Normal operation — device opens cleanly after clean shutdown |
| `true` | T265 | T265 often remains in streaming state after a crash; reset clears this |

Matches the official `realsense-ros` default (`initial_reset=false`). Set to `true` only when the device firmware is stuck from a previous crashed session.

### Automatic Delay via `start_cameras.sh`

The shell script automatically sets the 12-second delay when `--t265` is requested:

```bash
# These delays are applied automatically — no manual flags needed:
#   d435i_startup_delay_s:=12.0
#   l515_startup_delay_s:=12.0  (if L515 is also enabled)

./scripts/start_cameras.sh --d435i --t265          # D435i delayed 12s
./scripts/start_cameras.sh --d435i --l515 --t265   # Both D435i and L515 delayed 12s
./scripts/start_cameras.sh --d435i --l515          # No delay (no T265)
```

### Manual Override via Launch Args

```bash
ros2 launch realsense_camera_bringup realsense_camera.launch.py \
  enable_d435i:=true enable_t265:=true \
  d435i_startup_delay_s:=12.0 \
  t265_enable_hardware_reset:=true
```

### Recovery from USB Errors

If cameras fail to open after repeated kills/crashes, the USB power state may be corrupted at the kernel level. Reset it with:

```bash
# Find your device bus/device numbers from: lsusb
python3 -c "import fcntl; fcntl.ioctl(open('/dev/bus/usb/<BUS>/<DEV>','wb'), 0x5514, 0)"
```

Then re-run `start_cameras.sh` — the retry loop (5 attempts, 5s apart) handles transient re-enumeration delays.

---

## `odom_tf_relay` Node

### Purpose

The T265 tracking camera publishes odometry with `child_frame_id = t265_pose_frame` — the pose of the camera itself. For use with `robot_localization` EKF (which expects `child_frame_id = ackermann/base_link`) or Nav2, the odometry must be re-expressed so that the child frame is the robot base link.

`odom_tf_relay` performs this re-expression using the static TF between the sensor and the base link (published by `robot_state_publisher` from the URDF).

### Mathematics

Given:
- `T_WC` — odometry from sensor (world → camera pose, with linear/angular velocity in camera frame)
- `T_CB` — rigid transform from camera to base_link (from static TF tree)

**Pose composition:**
```
pos_WB = pos_WC + R(q_WC) * pos_CB
q_WB   = q_WC ⊗ q_CB
```

**Velocity (lever-arm corrected):**
```
v_B  = R_BC * (v_C + ω_C × pos_CB)
ω_B  = R_BC * ω_C
```
where `R_BC = rotation by conj(q_CB)` maps camera-frame vectors into base-link frame.

**Covariance rotation** (all four 3×3 blocks of the 6×6 pose and twist covariance matrices):
```
Σ_B = R_BC * Σ_C * R_BCᵀ
```
Applied to `[pp, pr, rp, rr]` blocks independently. Passing covariance unchanged when there is a rotation between frames would give incorrect uncertainty ellipsoids.

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `input_topic` | `/odom` | Source odometry topic |
| `output_topic` | `/odom_base` | Remapped output topic |
| `base_frame` | `ackermann/base_link` | Desired `child_frame_id` in output |
| `output_frame` | `""` | Override `frame_id` (empty = keep input `frame_id`) |
| `tf_timeout_s` | `1.0` | TF lookup timeout on first call (seconds) |

### TF Lookup Behaviour

- TF is looked up **once** (on the first received message) and cached for all subsequent messages.
- If TF is unavailable at startup, falls back to identity transform and emits a **single** warning (no spam).
- If `child_frame_id` already equals `base_frame`, the message is relayed unchanged (no transform applied).

### Origin Latching

The T265 VIO starts with its odom origin at the sensor's initial position (0,0,0 in
`t265_odom_frame`). After rigid-body composition, the first output `pos_WB` equals
the static mount offset `pos_CB` (e.g. `[-0.187, 0, -0.210]`), not (0,0,0).

This misaligns the T265 odom origin with other sources like `robot_localization` EKF
(`/odometry/filtered`), which defines origin at `base_link` = (0,0,0).

To fix this, `odom_tf_relay` **latches the first computed pose** (`pos_WB₀`, `q_WB₀`)
and subtracts it from all subsequent outputs:

```
pos_out = conj(q_WB₀) * (pos_WB - pos_WB₀)
q_out   = conj(q_WB₀) ⊗ q_WB
```

This ensures `/t265/odom_base` starts at (0,0,0) regardless of the sensor mount offset.
The latch value is logged at startup:

```
[t265_odom_relay] Origin latched: [-0.187 0.001 -0.210] — subtracting from all outputs
```

See also: [ADR-008](../decisions/ADR-008-px4-odometry-frames.md) for how this aligns
with the PX4 visual odometry origin.

### T265 Integration in Launch File

The relay node is automatically started alongside the T265 camera node, conditioned on `enable_t265`:

```python
t265_odom_relay = Node(
    package='realsense_camera_bringup',
    executable='odom_tf_relay',
    name='t265_odom_relay',
    condition=IfCondition(LaunchConfiguration('enable_t265')),
    parameters=[{
        'input_topic':  '/t265/odom',
        'output_topic': '/t265/odom_base',
        'base_frame':   'ackermann/base_link',
    }],
)
```

### Launch Arguments (T265 relay)

| Argument | Default |
|----------|---------|
| `t265_odom_input_topic` | `/t265/odom` |
| `t265_odom_output_topic` | `/t265/odom_base` |
| `t265_relay_base_frame` | `ackermann/base_link` |

Override at launch:

```bash
ros2 launch realsense_camera_bringup realsense_camera.launch.py \
  enable_t265:=true \
  t265_odom_output_topic:=/odom/t265 \
  t265_relay_base_frame:=base_link
```

### Source

`src/realsense_camera_bringup/src/odom_tf_relay.cpp`

---

## Shell Script (`scripts/start_cameras.sh`)

Convenience wrapper that launches cameras inside the Docker container via `docker-compose exec ackermann_slam`.

### Usage

```bash
./scripts/start_cameras.sh [OPTIONS]
```

### Camera Selection

| Flag | Effect |
|------|--------|
| `--d435i` | Launch D435i (default if none specified) |
| `--l515` | Launch L515 |
| `--t265` | Launch T265 |

### Options

| Flag | Effect | Applies to |
|------|--------|------------|
| `--serial-d435i=SN` | Set D435i serial number | D435i |
| `--serial-l515=SN` | Set L515 serial number | L515 |
| `--serial-t265=SN` | Set T265 serial number | T265 |
| `--imu` / `--no-imu` | Enable/disable D435i IMU | D435i |
| `--align-depth` / `--no-align-depth` | Enable/disable depth alignment | D435i, L515 |
| `--infra` | Enable infrared streams 1 & 2 | D435i, L515 |
| `--exposure-rgb=VAL` | Set RGB exposure (disables auto) | D435i, L515 |
| `--exposure-depth=VAL` | Set depth exposure (disables auto) | D435i, L515 |
| `--gain-rgb=VAL` | Set RGB gain | D435i, L515 |
| `--gain-depth=VAL` | Set depth gain | D435i, L515 |
| `--color-profile=WxHxF` | Color profile override | D435i, L515 |
| `--depth-profile=WxHxF` | Depth profile override | D435i, L515 |
| `--build[=PKG]` | Build before launching | |
| `--build-only[=PKG]` | Build only, skip launch | |

### Examples

```bash
# D435i only (default)
./scripts/start_cameras.sh

# D435i + L515
./scripts/start_cameras.sh --d435i --l515

# All three
./scripts/start_cameras.sh --d435i --l515 --t265

# Custom exposure + infrared
./scripts/start_cameras.sh --d435i --exposure-rgb=100 --infra

# Build then launch
./scripts/start_cameras.sh --build=realsense_camera_bringup --d435i --l515
```

### Script Behavior

1. Kills any existing `realsense_camera_node` processes (uses variable splitting to avoid pkill self-kill)
2. Restarts ROS 2 daemon
3. Invokes single `ros2 launch` with condition flags for all cameras
4. Shared options (exposure, gain, infra, align) applied to both D435i and L515 via `_add_depth_overrides()` helper

## Docker Integration

Container: `ackermann_slam` (ROS 2 Jazzy on Ubuntu 24.04)

Key docker-compose settings for camera access:
- `privileged: true` — required for USB device access
- `/dev:/dev` volume mount — direct USB enumeration
- `ipc: host` + `shm_size: 2g` — large shared memory for image buffers
- `network_mode: host` — DDS communication with host

librealsense2 v2.51.1 is pre-installed in the Docker image at `/usr/local`.

## Stubbed Parameters (Not Yet Implemented)

| Parameter | Purpose |
|-----------|---------|
| `publish_tf` | Broadcast camera TF frames |
| `enable_rgbd` | Combined RGBD topic |
| `pointcloud.enable` | Point cloud generation |
| `camera_namespace` | Topic namespace prefix |
| `unite_imu_method` | IMU interpolation (currently: cache + publish on gyro arrival) |

## Verification

### Quick Start (tmux session)

```bash
# 1. Start Docker container
./scripts/start_docker.sh

# 2. Launch tmux session (XRCE agent + ROS 2 nodes + odom verification)
./scripts/start_camera_px4_test_session.sh --depth-camera=d435i --t265 --rtabmap --no-rviz --bridge --vo-bridge

# 3. Navigate panes: Ctrl+b then arrow keys
# 4. Stop: 
./scripts/stop_all.sh && tmux kill-session -t rover
```

### Manual Verification

With all nodes running inside the container:

```bash
# Check camera topics are publishing
ros2 topic hz /d435i/color/image_raw
ros2 topic hz /d435i/depth/image_rect_raw
ros2 topic hz /t265/odom

# Check odom alignment (stationary robot — X-Y should agree within ~1 cm)
ros2 topic echo /odometry/filtered --once | grep -A3 'position:'
ros2 topic echo /t265/odom_base --once | grep -A3 'position:'
ros2 topic echo /px4_vehicle_odom_base --once | grep -A3 'position:'
```

### Automated Odom Verification

```bash
# One-shot
./scripts/verify_odom.sh

# Continuous (every 5s)
./scripts/verify_odom.sh --loop
```

Checks all three odom sources (`/odometry/filtered`, `/t265/odom_base`, `/px4_vehicle_odom_base`) and prints positions side-by-side. For a stationary robot:

| Check | Expected | Indicates bug if |
|-------|----------|------------------|
| X-Y agreement across sources | within ~1 cm | > 8 cm difference in X |
| T265 origin at (0,0,0) | yes (origin-latched) | X ≈ -0.187 m (mount offset not subtracted) |
| PX4 Z drift | normal (barometric) | N/A — not a frame error |

See [ADR-008](../decisions/ADR-008-px4-odometry-frames.md) for detailed verification results and frame alignment analysis.

## File Locations

| File | Path |
|------|------|
| Camera node source | `src/realsense_camera_bringup/src/realsense_camera_node.cpp` |
| Camera node header | `src/realsense_camera_bringup/include/realsense_camera_bringup/realsense_camera_node.hpp` |
| Odom relay source | `src/realsense_camera_bringup/src/odom_tf_relay.cpp` |
| CMakeLists | `src/realsense_camera_bringup/CMakeLists.txt` |
| Launch file | `src/realsense_camera_bringup/launch/realsense_camera.launch.py` |
| Config YAML | `src/realsense_camera_bringup/config/realsense_params.yaml` |
| Shell script | `scripts/start_cameras.sh` |
