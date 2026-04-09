## Context

The rover stack currently offers three odometry paths relevant to SLAM and navigation: RTAB-Map RGB-D visual odometry, ICP odometry, and the T265's built-in VIO relayed into the rover's `odom -> ackermann/base_link` frame contract. The proposed change adds a fourth option: VINS-Fusion using the T265 stereo fisheye cameras and IMU, with GPU-accelerated feature tracking when compatible CUDA-enabled OpenCV is available.

This is a cross-cutting change because it touches Docker dependencies, hardware bringup, launch orchestration, SLAM/EKF odometry selection, and frame-contract enforcement. It also introduces compatibility uncertainty: the selected upstream fork documents Foxy-era prerequisites, Jazzy support is not confirmed upstream, and Ubuntu 24.04's packaged Ceres is too new for at least some downstream forks.

## Goals / Non-Goals

**Goals:**
- Add a selectable VINS-Fusion odometry source without removing existing odometry options.
- Preserve the rover's TF and odometry contracts by adapting VINS output into `odom -> ackermann/base_link`.
- Reuse the existing T265 hardware path and auto-enable required fisheye streams when VINS mode is selected.
- Keep downstream consumers unchanged by continuing to publish `/odometry/filtered` as the fused odometry source for SLAM, Nav2, and PX4 bridges.
- Make GPU acceleration available when Docker image dependencies and hardware support it, while keeping a viable CPU/debug path during integration.

**Non-Goals:**
- Replacing RTAB-Map as the SLAM system.
- Removing T265 built-in odometry, RTAB-Map VO, or ICP odometry.
- Reworking PX4 bridge topics, Nav2 contracts, or the global TF tree.
- Treating upstream Jazzy compatibility as guaranteed before local build validation proves it.

## Architecture / Data Flow

```
T265 Fisheye (848x800 @30Hz) + IMU
  → realsense_camera_node (fisheye auto-enabled when use_vins_odom:=true)
  → /t265/fisheye1/image_raw, /t265/fisheye2/image_raw, /t265/imu
  → [vins_node] (GPU feature tracking via CUDA OpenCV, LD_LIBRARY_PATH injected)
  → /vins/raw_odometry (t265_pose_frame)
  → [odom_tf_relay] (frame adaptation: t265_pose_frame → ackermann/base_link)
  → /vins_odom (odom → ackermann/base_link)
  → EKF (odom0, IMU yaw fusion disabled for external VIO)
  → /odometry/filtered → RTAB-Map / Nav2 / PX4
```

Odometry priority when multiple sources enabled: **VINS-Fusion > T265 built-in > RGB-D VO > ICP**

All existing odometry topics (`/vo_odom`, `/icp_odom`, `/t265/odom_base`) remain published for debugging/comparison when their producers are enabled.

## Decisions

### Use a dedicated VINS capability and bringup package

VINS-Fusion will be integrated through a dedicated bringup package and a distinct OpenSpec capability instead of being folded into existing RTAB-Map launch logic alone.

Rationale:
- Keeps VINS-specific config files, launch behavior, and future patches isolated.
- Makes fork-specific compatibility work easier to localize.
- Preserves the current pattern where `robot_bringup` orchestrates subsystems rather than embedding all node details itself.

Alternatives considered:
- Embedding VINS directly in `robot_bringup` without a dedicated package. Rejected because it would mix fork-specific config and topic adaptation into the top-level launcher.

### Always adapt VINS output to the rover odometry contract

VINS output will not be consumed directly by downstream components. It will be relayed or adapted so the published odometry presented to the rest of the stack uses `frame_id=odom` and `child_frame_id=ackermann/base_link`.

Rationale:
- The repo already standardizes on `map -> odom -> ackermann/base_link`.
- Downstream EKF, RTAB-Map, Nav2, and PX4 bridging already depend on this contract.
- Upstream VINS output conventions do not match the rover's frame ownership and sensor mount conventions.

Alternatives considered:
- Letting VINS publish its native frames directly. Rejected because it would leak fork-specific frame conventions into the rest of the system and risk duplicate TF ownership.

### Treat T265 fisheye enablement as a gated bringup concern

Selecting VINS odometry in hardware mode will automatically require T265 hardware enablement and T265 fisheye stream enablement from top-level bringup. When VINS is not selected, fisheye streams will remain disabled to avoid unnecessary camera bandwidth, CPU work, and message traffic.

Rationale:
- VINS cannot operate without the T265 stereo fisheye feeds and IMU.
- The current RealSense launch already supports optional fisheye publication, so the missing behavior is orchestration rather than a new driver feature.

Alternatives considered:
- Requiring operators to manually set independent fisheye arguments. Rejected because it is error-prone and makes the launch contract harder to use.
- Leaving fisheye streams enabled whenever the T265 is present. Rejected because it spends resources in the common non-VINS cases without helping downstream consumers.

### Keep `/odometry/filtered` as the stable downstream interface

VINS will become another candidate source for EKF `odom0`, but the fused `/odometry/filtered` topic will remain the odometry interface consumed by downstream systems.

Rationale:
- This preserves Nav2 and PX4 expectations.
- It allows VINS integration without rewriting downstream specs and tooling.
- It keeps the repo's comparison/debugging model intact by allowing `/vo_odom`, `/icp_odom`, `/t265/odom_base`, and `/vins_odom` to coexist.

Alternatives considered:
- Bypassing EKF and feeding VINS directly to all downstream consumers. Rejected because it would change established system contracts and reduce comparability with the other odometry paths.

### Use fork validation gates rather than assuming upstream Jazzy support

The design treats `zinuok/VINS-Fusion-ROS2` as the primary candidate fork, but compatibility will be validated locally and a fallback fork remains part of the migration path.

Rationale:
- Upstream documentation still targets Foxy-era dependencies.
- The referenced Jazzy issue does not confirm mainline support; it only contains a report that a separate fork built on Jazzy with source-built Ceres 2.0.0.
- This reduces the risk of encoding unsupported assumptions into the spec.

Alternatives considered:
- Declaring Jazzy support as an accepted fact in the design. Rejected because the available evidence is weaker than that.

### Separate VINS calibration files from live RealSense CameraInfo

The VINS configuration will use dedicated calibration files for each fisheye camera plus a main VINS config, instead of relying on the live CameraInfo messages as the final source of truth.

Rationale:
- Upstream VINS configuration expects separate camera calibration files.
- The local RealSense driver publishes CameraInfo using a generic ROS distortion model, while VINS camera models and calibration layout are fork-specific.
- Dedicated files make calibration reviewable, versioned, and portable between runs.

Alternatives considered:
- Deriving all calibration only from runtime CameraInfo. Rejected because it is not a stable or sufficiently explicit contract for VINS.

## Risks / Trade-offs

- [Fork build breaks on ROS 2 Jazzy or Ubuntu 24.04] -> Validate the selected fork inside Docker first, keep fallback-fork criteria explicit, and avoid assuming issue #23 proves compatibility.
- [CUDA OpenCV conflicts with apt-linked RTAB-Map/OpenCV consumers] -> Prefer scoped dependency resolution for VINS first; only allow global precedence if compatibility is demonstrated.
- [VINS output uses incompatible frames or sensor-origin semantics] -> Always adapt through a relay/adapter before exposing the odometry to EKF and the rest of the stack.
- [T265 calibration or IMU timing is insufficient for stable VINS performance] -> Start with conservative extrinsic/time-offset estimation settings and validate on standalone bringup before integrating with the full stack.
- [GPU acceleration increases build time and image complexity] -> Keep GPU support optional at integration time and maintain a CPU/debug path while the stack stabilizes.
- [Operator workflow becomes harder to understand] -> Surface VINS as one more top-level odometry mode and extend existing scripts/docs rather than creating a separate launch flow.

## Migration Plan

1. Validate the selected VINS fork and dependency versions inside the Docker environment before wiring top-level launch behavior.
2. Add the new VINS bringup package and standalone launch/config so VINS can be exercised against T265 topics in isolation.
3. Add odometry adaptation into the rover frame contract and verify `/vins_odom` independently.
4. Extend `robot_bringup` and `rtabmap_slam` to select VINS as an odometry source while preserving existing source selection behavior.
5. Extend scripts and documentation so operators can launch and debug VINS with the same workflows used for other odometry modes.
6. Roll back by disabling `use_vins_odom` and returning to existing odometry sources if validation fails; no downstream interface rollback is required because `/odometry/filtered` remains unchanged.

## Open Questions

- Which fork becomes the canonical submodule after local Jazzy validation: `zinuok` mainline or a maintained fallback fork?
- Should GPU enablement be controlled by a local patch/CMake option rather than a source edit inside the vendored fork?
- What exact calibration source will be treated as authoritative for T265 fisheye intrinsics and body-to-camera extrinsics in this repo?
- Does the final VINS integration need a dedicated operator flag in `scripts/start_ros2_nodes.sh` and extra probes in `scripts/debug_vio.sh` as part of the initial change, or can that land as a follow-up?
