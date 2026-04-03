## 1. Dependency and fork validation

- [x] 1.1 Select the canonical VINS-Fusion fork for this repo and add it as a submodule under `src/`.
- [x] 1.2 Update the Docker dependency tooling (`docker/install_vins_gpu_deps.sh`, related compose mounts, and launch/build defaults) with the VINS-compatible CUDA-enabled OpenCV path needed for the selected fork.
- [x] 1.3 Rebuild the Docker image and validate that the chosen fork can be configured and built in the Jazzy container, patching the vendored fork locally if required.

## 2. VINS bringup package and odometry adaptation

- [x] 2.1 Create `src/vins_fusion_bringup/` with package metadata, launch files, and versioned T265 fisheye + IMU calibration/config files.
- [x] 2.2 Add the VINS estimator launch and topic wiring for `/t265/fisheye1/image_raw`, `/t265/fisheye2/image_raw`, and `/t265/imu`.
- [x] 2.3 Add or reuse an odometry adapter so the VINS output exposed as `/vins_odom` conforms to `odom -> ackermann/base_link`.
- [x] 2.4 Validate standalone VINS bringup and confirm `/vins_odom` publishes with the expected frame contract.

## 3. Top-level launch and SLAM integration

- [x] 3.1 Add `use_vins_odom` orchestration to `robot_bringup.launch.py`, including automatic T265 and fisheye enablement in hardware mode.
- [x] 3.2 Add VINS odometry-source selection to `rtabmap_slam.launch.py`, including EKF handling for external VIO heading data.
- [x] 3.3 Extend operator-facing scripts and docs so VINS can be launched and debugged through the repo’s normal workflows.

## 4. End-to-end verification

- [x] 4.1 Run the required workspace build and test commands in the Docker environment after the integration is complete.
- [x] 4.2 Verify topic flow, TF stability, and lifecycle readiness with VINS selected as the odometry source.
- [ ] 4.3 Validate the hardware or simulation launch path end to end, including `/vins_odom`, `/odometry/filtered`, RTAB-Map, and downstream PX4/Nav2 consumers. (blocked: D435i not connected during verification; RTAB-Map + Nav2 untested)
- [x] 4.4 Compare VINS against the existing odometry sources and document any remaining performance, compatibility, or GPU-usage findings. (T265 hardware VIO ~7mm drift stationary; VINS diverges due to stereo sync issue — tracked in follow-up task)
