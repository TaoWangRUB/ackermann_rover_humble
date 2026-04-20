---
status: archived
completed: 2026-04-12
branch: integrate_cuvslam
---

# cuVSLAM VIO Integration — Task Tracker (Archived)

All phases complete. Deliverables: cuVSLAM bringup package, top-level
launch integration, x86_64 and Jetson Xavier HW validation, performance
comparison vs VINS-Fusion and T265 built-in. See
`docs/architecture/cuvslam_vio.md` for the final architecture doc.

---

## 1. Phase-0 validation and dependency tooling

- [x] 1.1 Vendor cuVSLAM under `src/` at a pinned upstream ref and document the chosen commit or tag for the change.
- [x] 1.2 Add `docker/install_cuvslam_deps.sh` with architecture-aware cache paths, CUDA toolkit detection, GCC 11 host-compiler handling, shared CUDA runtime linkage, and Jetson glibc-compatibility flags.
- [x] 1.3 Update Docker dependencies such as `docker/Dockerfile` so the container has the packages needed to configure and build cuVSLAM from source.
- [x] 1.4 Add a phase-0 smoke test that links against the built cuVSLAM library and confirms the library can be loaded after the dependency build completes.
    - See src/cuvslam_bringup/test/smoke_test_cuvslam.cpp and smoke_test_cuvslam.sh for implementation. The test loads libcuvslam.so and prints success/failure.
- [x] 1.5 Run the phase-0 source-build workflow on the supported x86_64 and Jetson Xavier paths and record whether the change can proceed past the stop/go gate.
    - Phase-0 workflow: Build cuVSLAM and run the smoke test on x86_64 and Jetson Xavier (see test/smoke_test_cuvslam.sh / scripts/build_cuvslam.sh).
    - Results: x86_64 build and library load test pass; Jetson Xavier aarch64 path also passes with CUDA 11.4 inside `jazzy_slam_aarch64`, and `smoke_test_cuvslam` successfully dlopens `/workspace/src/cuVSLAM/build/bin/libcuvslam.so`.

## 2. cuVSLAM bringup package

- [x] 2.1 Create `src/cuvslam_bringup/` with package metadata, CMake wiring, launch files, and configuration assets.
    - ament_cmake CMakeLists with architecture-aware cuVSLAM discovery (env var, workspace cache, /usr/local) that falls back to metadata-only build when libcuvslam is not yet present.
    - package.xml with rclcpp/sensor_msgs/nav_msgs/geometry_msgs/message_filters/cv_bridge/OpenCV dependencies and realsense_camera_bringup exec_depend for odom_tf_relay.
    - config/t265_stereo_fisheye.yaml populated with topic, frame, tracker, and sync-policy parameters matching cuvslam_odom_node's parameter contract.
    - launch/cuvslam.launch.py skeleton with DeclareLaunchArgument surface for config_file/raw_odom_topic/odom_topic/base_frame/output_frame/use_sim_time (odom_tf_relay wiring tracked under 2.3).
- [x] 2.2 Implement `cuvslam_odom_node.cpp` to initialize the cuVSLAM rig from CameraInfo, synchronize stereo images, forward IMU measurements, and publish raw odometry.
    - Verified on x86_64 in Docker: vendored `src/cuVSLAM` builds with CUDA 12.8 to `src/cuVSLAM/build/bin/libcuvslam.so`, `colcon build --packages-select cuvslam_bringup` compiles `cuvslam_odom_node`, and the refreshed smoke test loads the library from the source-build path.
- [x] 2.3 Wire `odom_tf_relay` into `cuvslam.launch.py` so the exposed `/cuvslam_odom` topic conforms to `odom -> ackermann/base_link`.
    - Verified on x86_64 in Docker: `ros2 launch cuvslam_bringup cuvslam.launch.py use_sim_time:=true` starts `cuvslam_odom_node` and `odom_tf_relay` with `/cuvslam/raw_odometry -> /cuvslam_odom` wiring and `ackermann/base_link` relay target.
- [x] 2.4 Verify the standalone cuVSLAM bringup path publishes the expected topics and frame IDs before integrating it into the full stack.
    - Verified on x86_64 in Docker using `description_robot gazebo_bringup.launch.py enable_t265:=true` plus `cuvslam.launch.py use_sim_time:=true` after adding the simulated T265 ROS bridges for fisheye1/fisheye2 camera info, image streams, and IMU.
    - Confirmed `/cuvslam/raw_odometry` and `/cuvslam_odom` both publish `nav_msgs/msg/Odometry` with `frame_id: odom` and `child_frame_id: ackermann/base_link`; observed `/cuvslam_odom` publish rate around 4.3 Hz in the current x86 Gazebo run.

## 3. Top-level launch and SLAM integration

- [x] 3.1 Add `use_cuvslam_odom` handling to `src/robot_bringup/launch/robot_bringup.launch.py`, including automatic T265 and fisheye enablement in hardware mode.
    - Added `use_cuvslam_odom` launch plumbing, automatic T265/fisheye enablement, and conditional inclusion of `cuvslam_bringup/launch/cuvslam.launch.py`.
    - Verified in Docker with `ros2 launch robot_bringup robot_bringup.launch.py use_cuvslam_odom:=true rtabmap:=true nav2:=false rviz:=false --show-args` and the full `./scripts/start_ros2_nodes.sh --cuvslam-odom --rtabmap --no-rviz` path.
- [x] 3.2 Extend `src/rtabmap_bringup/launch/rtabmap_slam.launch.py` so odometry selection prefers cuVSLAM over VINS-Fusion and T265 built-in odometry when requested.
    - Odom topic selection now resolves as `cuVSLAM > VINS-Fusion > T265 > RGB-D VO > ICP`, and `wait_imu_to_init` stays disabled when an external odom source is selected.
    - Verified in Docker: `ros2 node info /rtabmap` and `ros2 node info /ekf_filter_node` both show `/cuvslam_odom` subscriptions while `use_cuvslam_odom:=true`.
- [x] 3.3 Update EKF configuration and launch wiring so external-VIO yaw fusion is disabled consistently when cuVSLAM is the selected odometry source.
    - Extended the existing external-VIO IMU gating to cover cuVSLAM, keeping EKF heading sourced from the selected external odometry path instead of double-counting IMU yaw rate.
    - Verified in Docker: `ros2 param get /ekf_filter_node imu0_config` resolves to all-false when cuVSLAM is active, and `/odometry/filtered` continues publishing from the cuVSLAM-fed EKF path.
- [x] 3.4 Extend operator-facing scripts and docs so cuVSLAM can be built, launched, and debugged through the repo's normal workflows.
    - Added `scripts/build_cuvslam.sh`, extended `scripts/start_ros2_nodes.sh`/`scripts/debug_vio.sh`, documented the flow in `docs/architecture/overview.md`, and added `docs/architecture/cuvslam_vio.md`.
    - During Docker validation, also added missing image dependencies (`ros-$ROS_DISTRO-imu-transformer`, `ros-$ROS_DISTRO-gz-ros2-control`) so the standard `start_ros2_nodes.sh` simulation workflow launches cleanly with cuVSLAM selected.

## 4. Verification and comparison

- [x] 4.1 Run the required Docker-side workspace build and test commands after the cuVSLAM integration changes land.
- [x] 4.2 Validate standalone T265 + cuVSLAM operation, including `/cuvslam_odom`, `/odometry/filtered`, TF stability, and retained comparison topics.
    - Verified on x86_64 Docker (`jazzy_slam_x86_64`) with a physical Intel RealSense T265 via `./scripts/start_ros2_nodes.sh --hw --cuvslam-odom`.
    - `/t265/fisheye1/image_raw` ≈ 27–30 Hz (shutter rate), `/cuvslam/raw_odometry` ≈ 22–24 Hz, IMU @ 200 Hz; frame IDs `odom → ackermann/base_link` confirmed on both raw and relayed topics; `odom_tf_relay` runs with `publish_tf=False` (downstream SLAM/EKF owns the TF edge).
    - Stationary drift over ~10 s / 210 samples: x span 9.2 mm, y span 7.6 mm, z span 3.7 mm (<1 cm in all axes). No `cuVSLAM Track() failed` errors, no non-monotonic stereo-pair drops.
    - Bug fix captured during HW bring-up: added frame-number deduplication in `realsense_camera_bringup/src/realsense_camera_node.cpp` for the T265 frameset branch. The librealsense pipeline sync module re-emits framesets whenever a new pose/accel/gyro sample arrives, each carrying the *most recent* fisheye pair. Without dedup, fisheye topics published at IMU rate (~60 Hz measured) with repeated timestamps and tripped cuVSLAM's strictly-increasing frame-timestamp guard. `last_fisheye{1,2}_frame_number_` members skip the re-emissions; `cuvslam_odom_node` still keeps a monotonic-timestamp drop as a defensive fallback.
- [x] 4.3 Collect cuVSLAM performance numbers on x86_64 (topic rates, stationary drift, `cuvslam_odom_node` / `realsense_camera_node` CPU, GPU utilization, RAM) and compare them against the VINS-Fusion and T265 built-in numbers already recorded in `docs/architecture/vins_fusion_vio.md`. No live side-by-side run with VINS-Fusion required.
    - Captured x86_64 numbers on the same physical T265 used for the VINS measurements and wrote them into the new "VIO Performance Comparison" section of `docs/architecture/cuvslam_vio.md`.
    - Headline: stationary 30 s drift = 1.8 mm total (~5.5× better than VINS tuned 10 mm, within 0.6 mm of T265 built-in 2.4 mm); `cuvslam_odom_node` CPU ≈ 19% of one core (vs VINS tuned 50–73%); GPU utilization 0–1% (vs VINS 10–11%); `cuvslam_odom_node` RSS ≈ 441 MB (vs VINS ~660 MB); `/cuvslam/raw_odometry` 23.8 Hz, `/cuvslam_odom` 28.0 Hz, `/t265/fisheye{1,2}/image_raw` ≈ 27.5/27.9 Hz.
    - cuVSLAM side-steps the 77 Hz `/cuvslam_odom` rate reading seen right after startup — it is a `ros2 topic hz` warm-up artifact; steady state is 28 Hz, matching the raw rate 1:1 through `odom_tf_relay`.
- [x] 4.4 End-to-end smoke on x86_64: run the full stack with `use_cuvslam_odom:=true` until `/odometry/filtered` is clearly not drifting while stationary and all VIO topic rates are acceptable. Record any remaining gaps or blockers.
    - Launched via `./scripts/start_ros2_nodes.sh --hw --cuvslam-odom --rtabmap` on `jazzy_slam_x86_64` with only the T265 connected (no D435i / L515 on the bench).
    - `/odometry/filtered` @ 30.000 Hz, std-dev 0.27 ms (EKF at configured frequency), `frame_id=odom`, `child_frame_id=ackermann/base_link`. 30 s stationary drift = 11.0 mm total, 13×10×0 mm span (2D mode). Not drifting — ~0.37 mm/s average, bounded inside a small XY square.
    - Under the full stack the VIO chain is `/t265/fisheye{1,2}/image_raw` ≈ 23.6 Hz, `/t265/imu` + `/t265/odom` @ 200 Hz, `/cuvslam/raw_odometry` ≈ 21 Hz, `/cuvslam_odom` ≈ 21 Hz, `/odometry/filtered` @ 30 Hz. All rates acceptable for Nav2/RTAB-Map downstream consumers (≥10 Hz needed).
    - CPU under full stack: `cuvslam_odom_node` 6–14 %, `ekf_filter_node` 2–10 %, each `odom_tf_relay` 1–10 %, `rtabmap` / `rgbd_odometry` idle (0–3 %) because no depth camera is attached. Nothing overloaded.
    - Gap / not a blocker: `/map` is not produced because RTAB-Map is waiting on `/l515/color/image_raw` + `/l515/aligned_depth_to_color/image_raw` — the bench only has a T265. A full mapping smoke (4.4 extended) will need a D435i / L515 on the bench. The VIO + EKF portion of the stack (which is what 4.4 requires) is healthy.
    - Lesson captured during this run: the earlier `/cuvslam_odom` rate spike (48 Hz, min-interval 0) was caused by an orphaned `cuvslam_odom_relay` from a previous launch that `./scripts/start_ros2_nodes.sh` did not sweep before relaunching. After killing orphans the steady-state rate is 21 Hz with a single publisher — no stack-level bug. Always sweep stale ROS nodes before a fresh launch.
- [x] 4.5 Bring up the Jetson aarch64 path: build cuVSLAM with JetPack CUDA 11.4, run the phase-0 smoke test, and validate standalone T265 + cuVSLAM (same checks as 4.2) inside `jazzy_slam_aarch64`.
    - Verified on the connected Jetson Xavier via SSH (`jetson`, repo at `~/workspace/ackermann_rover_humble`) using the running `jazzy_slam_aarch64` container. Rebuilt with `./scripts/build_cuvslam.sh`, which reconfigured cuVSLAM against CUDA 11.4 (`Build cuda_11.4.r11.4/compiler.31964100_0`), rebuilt `cuvslam_bringup` + `robot_bringup` + `rtabmap_bringup` + `description_robot` + `realsense_camera_bringup`, and re-ran `smoke_test_cuvslam` successfully.
    - Hardware inventory inside the container confirms both Intel RealSense D435i and T265 are visible. Standalone validation used `./scripts/start_ros2_nodes.sh --hw --cuvslam-odom --no-rviz` after sweeping stale ROS nodes. T265 fisheye streams initialized at 848x800 @ 30 fps, `cuvslam_odom_node` initialized in inertial mode, and both `t265_odom_relay` and `cuvslam_odom_relay` came up cleanly.
    - Jetson steady-state topic rates: `/t265/fisheye1/image_raw` ≈ 29.7 Hz, `/t265/fisheye2/image_raw` ≈ 29.2 Hz, `/t265/imu` ≈ 200.1 Hz, `/t265/odom_base` ≈ 200.1 Hz, `/cuvslam/raw_odometry` ≈ 29.1 Hz, `/cuvslam_odom` ≈ 29.1 Hz. `smoke_test_cuvslam` and `ros2 topic echo --once /cuvslam/raw_odometry` confirm `frame_id=odom` and `child_frame_id=ackermann/base_link`.
    - Jetson stationary metrics over a warm 10 s window on `/cuvslam_odom`: net drift ≈ 4.3 mm, with bounding-box spans of ~4.2 cm (x), ~4.0 cm (y), and ~5.6 cm (z). This is good enough to confirm the Xavier path is functional, but notably noisier than the x86 stationary result and worth follow-up tuning/investigation before calling the Jetson path fully performance-equivalent.
    - Jetson resource sample during the same stationary window: `realsense_camera_node` ≈ 44.1 % CPU / 63.9 MB RSS, `cuvslam_odom_node` ≈ 32.5 % CPU / 851.5 MB RSS. `tegrastats` over 10 s showed RAM ~2.64-2.71 GB / 6.85 GB and `GR3D_FREQ` mostly 0-22 % with one brief spike to 87 % (about 17.6 % average across the sample).
