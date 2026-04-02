# Hardware Performance Notes

This page tracks ad hoc hardware performance measurements for the real-camera
RTAB-Map/VIO pipeline.

## Test Command

Unless noted otherwise, the stack was launched with:

```bash
./scripts/start_ros2_nodes.sh --hw --depth-camera=d435i --t265 --rtabmap
```

## Jetson Xavier

Measurement date: 2026-04-02  
Measurement method: live Jetson run of the command above, then
`HZ_WINDOW=5 ECHO_TIMEOUT=3 ./scripts/debug_vio.sh` from the Jetson host.

### Topic Rates

| Topic | Rate / Result |
| --- | --- |
| `/d435i/color/image_raw` | 23.012 Hz |
| `/d435i/aligned_depth_to_color/image_raw` | 22.378 Hz |
| `/d435i/imu` | 199.682 Hz |
| `/t265/odom` | 201.044 Hz |
| `/t265/odom_base` | 200.299 Hz |
| `/imu/raw_transformed` | 198.539 Hz |
| `/imu/data` | 196.453 Hz |
| `/rgbd_image` | 4.475 Hz |
| `/vo_odom` | 1.830 Hz |
| `/odometry/filtered` | 29.971 Hz |
| `/map` | no samples observed in the 5 s probe window |

### Notes

- D435i IMU and T265 odometry were healthy and close to 200 Hz.
- EKF output was close to 30 Hz.
- The RGB-D/VO path was the bottleneck:
  - `/rgbd_image` was only about 4.5 Hz.
  - `/vo_odom` was only about 1.8 Hz.
- During the same run, RTAB-Map stayed at 1.00 Hz and launch logs showed:
  - typical RTAB-Map compute around 0.35 s to 0.58 s
  - frequent end-to-end delay around 0.7 s to 1.4 s
  - occasional delay spikes up to about 2.46 s
  - recurring `ekf_filter_node` update-rate overruns

## x86 Host

Last successful full-stack snapshot: earlier successful x86 hardware run in this
investigation, with both D435i and T265 visible inside the container.

### Successful Snapshot

| Metric | Result |
| --- | --- |
| D435i USB | 3.2 |
| T265 USB | 3.1 |
| D435i depth alignment | CUDA GPU-accelerated |
| RTAB-Map detection rate | 1.00 Hz |
| RTAB-Map compute time | about 0.036 s to 0.054 s |
| End-to-end delay | about 0.12 s to 0.17 s |

### Supporting Checks From That x86 Session

| Check | Result |
| --- | --- |
| Native librealsense D435i IMU probe | `ACCEL_COUNT 285`, `GYRO_COUNT 914` |
| ROS-level D435i launch sample | `IMU_COUNT=1000`, `COLOR_COUNT=150` |

### Current Status

A fresh x86 rerun on 2026-04-02 could not reproduce the full-stack measurement,
because both RealSense nodes reported `No device connected` during startup.
That means the current x86 section above is the last successful snapshot, not a
same-minute rerun like the Jetson Xavier numbers.

## Quick Takeaway

- Jetson Xavier currently has healthy camera/IMU inputs, but the RTAB-Map VO
  path is the limiting stage.
- The last successful x86 full-stack run was materially faster on RTAB-Map
  compute and end-to-end delay than the Jetson Xavier run.
