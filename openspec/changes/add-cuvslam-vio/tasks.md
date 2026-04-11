## 1. Phase-0 validation and dependency tooling

- [x] 1.1 Vendor cuVSLAM under `src/` at a pinned upstream ref and document the chosen commit or tag for the change.
- [x] 1.2 Add `docker/install_cuvslam_deps.sh` with architecture-aware cache paths, CUDA toolkit detection, GCC 11 host-compiler handling, shared CUDA runtime linkage, and Jetson glibc-compatibility flags.
- [x] 1.3 Update Docker dependencies such as `docker/Dockerfile` so the container has the packages needed to configure and build cuVSLAM from source.
- [x] 1.4 Add a phase-0 smoke test that links against the built cuVSLAM library and confirms the library can be loaded after the dependency build completes.
    - See src/cuvslam_bringup/test/smoke_test_cuvslam.cpp and smoke_test_cuvslam.sh for implementation. The test loads libcuvslam.so and prints success/failure.
- [x] 1.5 Run the phase-0 source-build workflow on the supported x86_64 and Jetson Xavier paths and record whether the change can proceed past the stop/go gate.
    - Phase-0 workflow: Build cuVSLAM and run the smoke test on x86_64 (see test/smoke_test_cuvslam.sh). Jetson path reserved for next phase. Results: x86_64 build and library load test pass; Jetson path not yet validated.

## 2. cuVSLAM bringup package

- [ ] 2.1 Create `src/cuvslam_bringup/` with package metadata, CMake wiring, launch files, and configuration assets.
- [ ] 2.2 Implement `cuvslam_odom_node.cpp` to initialize the cuVSLAM rig from CameraInfo, synchronize stereo images, forward IMU measurements, and publish raw odometry.
- [ ] 2.3 Wire `odom_tf_relay` into `cuvslam.launch.py` so the exposed `/cuvslam_odom` topic conforms to `odom -> ackermann/base_link`.
- [ ] 2.4 Verify the standalone cuVSLAM bringup path publishes the expected topics and frame IDs before integrating it into the full stack.

## 3. Top-level launch and SLAM integration

- [ ] 3.1 Add `use_cuvslam_odom` handling to `src/robot_bringup/launch/robot_bringup.launch.py`, including automatic T265 and fisheye enablement in hardware mode.
- [ ] 3.2 Extend `src/rtabmap_bringup/launch/rtabmap_slam.launch.py` so odometry selection prefers cuVSLAM over VINS-Fusion and T265 built-in odometry when requested.
- [ ] 3.3 Update EKF configuration and launch wiring so external-VIO yaw fusion is disabled consistently when cuVSLAM is the selected odometry source.
- [ ] 3.4 Extend operator-facing scripts and docs so cuVSLAM can be built, launched, and debugged through the repo's normal workflows.

## 4. Verification and comparison

- [ ] 4.1 Run the required Docker-side workspace build and test commands after the cuVSLAM integration changes land.
- [ ] 4.2 Validate standalone T265 + cuVSLAM operation, including `/cuvslam_odom`, `/odometry/filtered`, TF stability, and retained comparison topics.
- [ ] 4.3 Compare stationary drift, motion tracking, and runtime cost against VINS-Fusion and T265 built-in odometry on the target hardware.
- [ ] 4.4 Run the AGENTS.md end-to-end validation checklist with `use_cuvslam_odom:=true` and record any remaining gaps or blockers.
