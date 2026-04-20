## Context

The rover already supports several odometry paths that feed RTAB-Map, Nav2, and PX4 through the shared `/odometry/filtered` interface: RTAB-Map RGB-D visual odometry, ICP odometry, Intel T265 built-in VIO, and VINS-Fusion using the T265 fisheye stereo pair plus IMU. The repo's current CUDA story is split by architecture: x86_64 builds lean on a mounted desktop CUDA 12.x toolchain and a newer CUDA-enabled OpenCV cache, while Jetson Xavier builds use CUDA 11.4, GCC 11, shared CUDA runtime linkage, a different OpenCV version, and a glibc compatibility header to keep Ubuntu 24.04 workable.

cuVSLAM is attractive because it is open source, CUDA-first, and already structured as a native library rather than a monolithic ROS application. At the same time, the repo wants to stay lean: adding Isaac ROS Visual SLAM would bring a large dependency tree that does not fit the existing Docker image or bringup pattern. The change is therefore cross-cutting across Docker, dependency tooling, top-level launch orchestration, SLAM odometry selection, and T265 hardware behavior.

The largest uncertainty is whether cuVSLAM source builds really remain viable on Jetson Xavier's CUDA 11.4 stack even though the upstream README calls out CUDA 12/13 for prebuilt binaries. Because x86_64 host development is closer to the upstream toolchain expectations and is easier to iterate on, the integration needs to be staged rather than attempting both platforms in parallel.

## Goals / Non-Goals

**Goals:**
- Add cuVSLAM as a selectable stereo fisheye + IMU odometry source alongside VINS-Fusion, T265 built-in odometry, RTAB-Map VO, and ICP.
- Establish a working x86_64 host integration path before Jetson Xavier enablement is treated as the next milestone.
- Reuse the existing T265 fisheye and IMU path so no new camera hardware or topic families are required.
- Keep cuVSLAM in odometry-only mode so RTAB-Map continues to own mapping and `map -> odom`.
- Preserve the rover's odometry and TF contracts by relaying cuVSLAM output into `odom -> ackermann/base_link`.
- Define an architecture-aware build path for x86_64 and Jetson Xavier that follows the repo's existing CUDA toolchain conventions.
- Insert explicit rollout gates so Jetson work begins only after the x86_64 host path is green and Jetson completion is validated independently.

**Non-Goals:**
- Replacing RTAB-Map as the SLAM or mapping system.
- Removing or deprecating VINS-Fusion, T265 built-in odometry, RTAB-Map VO, or ICP.
- Pulling in Isaac ROS NITROS/GXF/GEM dependency chains as part of this change.
- Using Python bindings as the primary cuVSLAM execution path.
- Treating prebuilt cuVSLAM binaries as a requirement on Jetson Xavier.

## Decisions

### Stage the rollout x86_64 first, then Jetson Xavier

The change will establish a working x86_64 host integration first and only then move to Jetson Xavier-specific enablement.

Rationale:
- x86_64 is closer to cuVSLAM's upstream CUDA expectations, so it is the lowest-risk place to validate the wrapper, launch integration, and odometry contracts.
- Jetson Xavier has an additional compatibility matrix: CUDA 11.4, GCC 11, and an older CUDA-enabled OpenCV path. That deserves its own follow-on phase instead of being hidden inside the initial bringup work.
- Separating the milestones makes it easier to land useful cuVSLAM work even if Jetson-specific enablement takes longer.

Alternatives considered:
- Integrate x86_64 and Jetson in parallel. Rejected because it mixes platform-specific toolchain failures into the first integration pass.
- Start with Jetson because it is the target hardware. Rejected because it is the higher-risk environment and would slow down basic wrapper/launch iteration.

### Use a thin ROS 2 C++ wrapper around `cuvslam2.h`

The repo will integrate cuVSLAM by building the library from source and adding a small native ROS 2 C++ node in `src/cuvslam_bringup/`.

Rationale:
- This matches the repo's current bringup style better than Isaac ROS packages.
- It keeps the dependency graph narrow and localizes ownership of ROS topics, frame conversion, and lifecycle behavior inside one package.
- It avoids Python per-frame overhead and GIL contention on Xavier's limited CPU budget.

Alternatives considered:
- Isaac ROS Visual SLAM wrapper. Rejected because the dependency chain is too heavy for the current Docker image and workflow.
- Python bindings. Rejected because 30 Hz stereo VIO on Xavier leaves little CPU headroom for interpreter overhead.

### Reuse T265 fisheye and IMU topics and build the rig from live CameraInfo

cuVSLAM will subscribe to the existing `/t265/fisheye1/image_raw`, `/t265/fisheye2/image_raw`, `/t265/fisheye1/camera_info`, `/t265/fisheye2/camera_info`, and `/t265/imu` topics. The wrapper will construct the cuVSLAM rig from the first valid CameraInfo pair and use YAML configuration only for wrapper behavior, topic names, and launch-time policy.

Rationale:
- It avoids maintaining a second source of truth for the T265 intrinsics when the RealSense driver already publishes them.
- It follows the user's desired wrapper lifecycle and keeps the integration closer to the live sensor contract.
- It lets VINS-Fusion keep its own static calibration files without forcing cuVSLAM into the same configuration model.

Alternatives considered:
- Hard-code all camera intrinsics/extrinsics in YAML as VINS-Fusion does. Rejected because it duplicates calibration data and makes drift between driver output and wrapper assumptions more likely.

### Keep cuVSLAM odometry-only and preserve `/odometry/filtered`

cuVSLAM will act only as an external odometry source. RTAB-Map will remain the mapping owner, `robot_localization` will continue to publish `/odometry/filtered`, and Nav2/PX4 will keep consuming that filtered output.

Rationale:
- This keeps the rest of the stack unchanged.
- It preserves the repo's current comparison model where multiple raw odometry topics can coexist while downstream systems stay wired to `/odometry/filtered`.
- It aligns with Xavier's memory constraints by avoiding additional mapping or loop-closure work inside the cuVSLAM path.

Alternatives considered:
- Feed cuVSLAM directly to downstream consumers. Rejected because it would change stable system contracts.
- Let cuVSLAM own mapping. Rejected because RTAB-Map already fills that role and the requested mode is odometry-only.

### Mirror the repo's architecture-aware CUDA toolchain conventions, but keep platform phases distinct

The cuVSLAM dependency tooling will follow the same architecture split documented for VINS-Fusion:
- x86_64 builds use the mounted host CUDA toolkit under `/usr/local/cuda` and can target the local desktop GPU architecture.
- Jetson Xavier builds force GCC 11 as the C, C++, and CUDA host compiler, use `CUDA_RUNTIME_LIBRARY=Shared`, inject `realsense_camera_bringup/cuda_glibc_compat.h` during CUDA compilation, and keep parallelism conservative (`make -j2`).
- Built artifacts are cached under an architecture-qualified prefix such as `/workspace/docker_cache/cuvslam/<arch>/current` so x86_64 and aarch64 outputs never collide.
- x86_64 and Jetson can use different OpenCV/CUDA combinations as long as each path is validated independently and the rollout order stays x86 first, Jetson second.

Rationale:
- These patterns are already proven elsewhere in the repo.
- They directly address the known Xavier failure modes: static CUDA runtime link errors, Ubuntu 24.04 glibc parsing issues, and memory pressure during native builds.

Alternatives considered:
- Use a single unqualified build cache. Rejected because the binaries and generated CMake metadata are architecture-specific.
- Treat x86_64 and Xavier as separate one-off scripts. Rejected because the repo already has a reusable architecture-aware pattern.

### Give cuVSLAM highest priority among external VIO modes

When multiple external-VIO launch flags are enabled, the SLAM stack will choose cuVSLAM ahead of VINS-Fusion and T265 built-in odometry. VINS-Fusion will remain ahead of T265 built-in odometry.

Rationale:
- The user's intent is to integrate cuVSLAM as the preferred GPU-accelerated alternative while still preserving the older modes for debugging and fallback.
- Making the priority explicit avoids ambiguous behavior if multiple flags are accidentally enabled.

Alternatives considered:
- Reject launches with more than one external-VIO flag. Rejected because the repo often keeps non-selected odometry producers available for comparison, and explicit priority is simpler operationally.

## Risks / Trade-offs

- [cuVSLAM source build still fails on Xavier CUDA 11.4] -> Keep Jetson as phase 2, after x86_64 is already working, and validate Jetson independently.
- [x86_64 and Jetson require different CUDA/OpenCV dependency combinations] -> Treat the dependency tooling and caches as architecture-specific from the start and avoid implying one binary/dependency stack serves both.
- [T265 fisheye model or extreme edge FOV degrades cuVSLAM tracking] -> Start with full-frame support but validate stationary drift and motion paths against VINS-Fusion and T265 built-in odometry.
- [Jetson memory pressure during native compilation] -> Use conservative parallelism, cache built artifacts, and keep cuVSLAM in odometry-only mode.
- [Static CUDA linkage pulls incompatible system libraries on Ubuntu 24.04] -> Force shared CUDA runtime linkage in the cuVSLAM build path.
- [Frame or coordinate conventions differ from ROS ENU/FLU expectations] -> Relay raw cuVSLAM output through the existing odom adapter before exposing it to EKF and the rest of the stack.
- [Keeping VINS-Fusion and cuVSLAM side by side complicates launch logic] -> Centralize source selection in `robot_bringup` and `rtabmap_slam` with explicit priority rules.

## Migration Plan

1. Vendor cuVSLAM under `src/` and implement the x86_64 host build, smoke-link, wrapper, and launch workflow first.
2. Validate the x86_64 host path end to end so the ROS integration, odometry contract, and operator workflow are stable before any Jetson-specific work begins.
3. Extend the dependency tooling for Jetson Xavier's CUDA 11.4 and OpenCV requirements, then validate Jetson source compilation as a separate phase.
4. Reuse the x86_64 wrapper and launch integration on Jetson, patching only the Jetson-specific toolchain or runtime issues that remain.
5. Roll back by disabling `use_cuvslam_odom`; downstream systems remain unchanged because `/odometry/filtered` stays the stable interface.

## Open Questions

- Which upstream cuVSLAM commit or tag becomes the pinned submodule reference after the phase-0 build passes?
- Does cuVSLAM need image masking or central-region cropping for the T265's widest fisheye edges, or is the default fisheye model sufficient?
- What covariance mapping and failure-state behavior should the wrapper publish when cuVSLAM tracking is initializing, degraded, or lost?
- Should the x86_64 path cache only the final cuVSLAM install tree, or also preserve an intermediate build directory for faster local iteration?
