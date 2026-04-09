## Why

The rover currently supports RTAB-Map RGB-D odometry, ICP odometry, and the T265's built-in VIO, but it does not offer a software-controlled stereo fisheye + IMU VIO path that can be tuned and debugged inside the ROS stack. Adding VINS-Fusion with optional GPU-accelerated feature tracking gives the system a selectable alternative to Intel's discontinued proprietary T265 estimator while keeping the existing odometry sources available.

## What Changes

- Add a selectable VINS-Fusion-based VIO source that consumes T265 fisheye stereo images and IMU data.
- Add bringup/configuration support for launching VINS-Fusion, relaying its output into the rover's standard `odom -> ackermann/base_link` frame contract, and selecting it as the SLAM/EKF odometry input.
- Extend top-level bringup so enabling VINS automatically enables the required T265 hardware path and fisheye streams in hardware mode, while keeping fisheye streams disabled when VINS is not selected.
- Add container dependency support and validation steps for the chosen VINS-Fusion fork and its OpenCV/Ceres requirements, with Jazzy compatibility treated as an integration constraint rather than assumed upstream support.
- Preserve existing odometry options (`/vo_odom`, `/icp_odom`, `/t265/odom_base`) so VINS can be introduced without removing or regressing current workflows.

## Capabilities

### New Capabilities
- `vins-fusion-vio`: GPU-capable stereo fisheye + IMU visual-inertial odometry using VINS-Fusion, including configuration, launch, topic contracts, and output adaptation into the rover frame conventions.

### Modified Capabilities
- `robot-bringup`: top-level launch behavior changes to expose and orchestrate VINS-Fusion as another odometry source.
- `slam`: SLAM and EKF odometry-source selection changes to support VINS-Fusion alongside VO, ICP, and T265 built-in odometry.
- `realsense-camera`: T265 hardware requirements change to support guaranteed fisheye stream availability for VINS mode.

## Impact

- Affected code: `.gitmodules`, `docker/docker-compose.yml`, `docker/install_vins_gpu_deps.sh`, `scripts/build_vins_gpu.sh`, new `src/vins_fusion_bringup/` package, `src/robot_bringup/launch/robot_bringup.launch.py`, `src/rtabmap_bringup/launch/rtabmap_slam.launch.py`, and RealSense launch/config wiring.
- Affected systems: Docker image build, hardware camera launch flow, SLAM/EKF odometry input selection, T265 hardware workflows, and operator scripts used to launch/debug the stack.
- New dependency surface: VINS-Fusion fork selection, Ceres compatibility management, and CUDA-enabled OpenCV for GPU feature tracking.
