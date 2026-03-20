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

**Hardware reset on startup.** Clears stuck USB states. Polls for the specific device serial (not just any RealSense) to avoid false-positive detection of other connected cameras.

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

## File Locations

| File | Path |
|------|------|
| Node source | `src/realsense_camera_bringup/src/realsense_camera_node.cpp` |
| Header | `src/realsense_camera_bringup/include/realsense_camera_bringup/realsense_camera_node.hpp` |
| CMakeLists | `src/realsense_camera_bringup/CMakeLists.txt` |
| Launch file | `src/realsense_camera_bringup/launch/realsense_camera.launch.py` |
| Config YAML | `src/realsense_camera_bringup/config/realsense_params.yaml` |
| Shell script | `scripts/start_cameras.sh` |
