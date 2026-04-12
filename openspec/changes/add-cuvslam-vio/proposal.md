## Why

The rover now has two practical external VIO paths: Intel T265 built-in odometry and VINS-Fusion running on the T265 fisheye stereo pair. On Jetson Xavier NX, VINS-Fusion is usable but still compute-limited, while cuVSLAM offers a GPU-first stereo VIO path that can eventually better exploit CUDA on both desktop and Jetson platforms while RTAB-Map continues to own mapping.

The platform story is not symmetric, though: x86_64 host development is closer to cuVSLAM's upstream CUDA 12/13 expectations, while Jetson Xavier uses CUDA 11.4 and a different OpenCV/CUDA compatibility path. The change therefore needs an explicit x86-first rollout and a second Jetson-specific phase instead of treating both targets as one integration step.

## What Changes

- Add a selectable cuVSLAM-based stereo visual-inertial odometry path that consumes the T265 fisheye cameras and IMU.
- Add a thin ROS 2 C++ wrapper package that builds a cuVSLAM rig from CameraInfo, forwards stereo frames and IMU measurements, and publishes rover-compatible odometry.
- Stage the implementation so x86_64 host integration and validation land first, then Jetson Xavier enablement follows with its own CUDA/OpenCV-specific dependency path.
- Extend top-level bringup with `use_cuvslam_odom` orchestration and reuse the existing T265 fisheye enablement path in hardware mode.
- Extend SLAM/EKF odometry-source selection so cuVSLAM becomes the highest-priority external VIO source when enabled, ahead of VINS-Fusion and T265 built-in odometry.
- Add Docker and build-tooling support for source-building cuVSLAM on both x86_64 and Jetson Xavier, reusing the repo's CUDA compatibility patterns where possible.
- Add explicit rollout gates so x86_64 host support must be working before Jetson-specific compilation and validation are treated as in-scope completion criteria.

## Capabilities

### New Capabilities
- `cuvslam-vio`: GPU-accelerated stereo fisheye + IMU visual-inertial odometry using cuVSLAM, including source-build validation, ROS 2 wrapper behavior, topic contracts, and rover-frame odometry output.

### Modified Capabilities
- `robot-bringup`: add `use_cuvslam_odom` launch orchestration and propagate cuVSLAM mode into the existing hardware and SLAM bringup flow.
- `slam`: extend external odometry selection and EKF behavior so cuVSLAM can be selected ahead of VINS-Fusion, T265 built-in odometry, RGB-D VO, and ICP.
- `realsense-camera`: confirm that top-level T265 fisheye enablement semantics cover cuVSLAM as well as VINS-Fusion consumers.

## Impact

- Affected code: new `src/cuvslam_bringup/` package, new `docker/install_cuvslam_deps.sh`, new `scripts/build_cuvslam.sh`, `docker/Dockerfile`, `src/robot_bringup/launch/robot_bringup.launch.py`, `src/rtabmap_bringup/launch/rtabmap_slam.launch.py`, and a new cuVSLAM submodule under `src/`.
- Affected systems: Docker image build, x86_64 host and Jetson Xavier CUDA/OpenCV toolchain handling, T265 fisheye hardware workflows, EKF odometry-source selection, RTAB-Map input selection, and downstream Nav2/PX4 consumers of `/odometry/filtered`.
- Dependency surface: cuVSLAM source tree, `liblmdb-dev`, CUDA host-compiler selection, shared-vs-static CUDA runtime linkage, and reuse of the repo's existing glibc/CUDA compatibility header for Jetson builds.
