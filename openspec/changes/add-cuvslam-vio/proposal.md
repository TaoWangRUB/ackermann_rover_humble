## Why

The rover now has two practical external VIO paths: Intel T265 built-in odometry and VINS-Fusion running on the T265 fisheye stereo pair. On Jetson Xavier NX, VINS-Fusion is usable but still compute-limited, while cuVSLAM offers a GPU-first stereo VIO path that is more aligned with the Xavier's CUDA hardware and can run in odometry-only mode while RTAB-Map continues to own mapping.

The main uncertainty is build viability on Jetson Xavier's CUDA 11.4 toolchain because cuVSLAM's README calls out CUDA 12/13 for prebuilt binaries. The source-level review and the existing VINS CUDA setup in this repo suggest the Xavier path is still plausible, so the change needs a formal validation gate before deeper integration work starts.

## What Changes

- Add a selectable cuVSLAM-based stereo visual-inertial odometry path that consumes the T265 fisheye cameras and IMU.
- Add a thin ROS 2 C++ wrapper package that builds a cuVSLAM rig from CameraInfo, forwards stereo frames and IMU measurements, and publishes rover-compatible odometry.
- Extend top-level bringup with `use_cuvslam_odom` orchestration and reuse the existing T265 fisheye enablement path in hardware mode.
- Extend SLAM/EKF odometry-source selection so cuVSLAM becomes the highest-priority external VIO source when enabled, ahead of VINS-Fusion and T265 built-in odometry.
- Add Docker and build-tooling support for source-building cuVSLAM on both x86_64 and Jetson Xavier, reusing the repo's CUDA compatibility patterns where possible.
- Add an explicit phase-0 stop/go validation step so Xavier source compilation and smoke-linking are proven before the rest of the integration is treated as implementation-ready.

## Capabilities

### New Capabilities
- `cuvslam-vio`: GPU-accelerated stereo fisheye + IMU visual-inertial odometry using cuVSLAM, including source-build validation, ROS 2 wrapper behavior, topic contracts, and rover-frame odometry output.

### Modified Capabilities
- `robot-bringup`: add `use_cuvslam_odom` launch orchestration and propagate cuVSLAM mode into the existing hardware and SLAM bringup flow.
- `slam`: extend external odometry selection and EKF behavior so cuVSLAM can be selected ahead of VINS-Fusion, T265 built-in odometry, RGB-D VO, and ICP.
- `realsense-camera`: confirm that top-level T265 fisheye enablement semantics cover cuVSLAM as well as VINS-Fusion consumers.

## Impact

- Affected code: new `src/cuvslam_bringup/` package, new `docker/install_cuvslam_deps.sh`, new `scripts/build_cuvslam.sh`, `docker/Dockerfile`, `src/robot_bringup/launch/robot_bringup.launch.py`, `src/rtabmap_bringup/launch/rtabmap_slam.launch.py`, and a new cuVSLAM submodule under `src/`.
- Affected systems: Docker image build, x86_64 and Jetson Xavier CUDA toolchain handling, T265 fisheye hardware workflows, EKF odometry-source selection, RTAB-Map input selection, and downstream Nav2/PX4 consumers of `/odometry/filtered`.
- Dependency surface: cuVSLAM source tree, `liblmdb-dev`, CUDA host-compiler selection, shared-vs-static CUDA runtime linkage, and reuse of the repo's existing glibc/CUDA compatibility header for Jetson builds.
