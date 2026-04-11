## MODIFIED Requirements

### Requirement: Top-level fisheye enablement compatibility
The RealSense bringup SHALL accept top-level launch control for T265 fisheye stream enablement so higher-level bringup can activate VINS-Fusion or cuVSLAM prerequisites without manual per-camera overrides.

#### Scenario: Top-level launch requests stereo-fisheye VIO prerequisites
- **WHEN** higher-level bringup selects a VINS-Fusion-based or cuVSLAM-based odometry mode
- **THEN** the RealSense launch SHALL honor the passed T265 fisheye enablement argument and start the required fisheye streams

#### Scenario: Stereo-fisheye VIO mode not selected
- **WHEN** the T265 hardware path is enabled but higher-level bringup has not selected a VINS-Fusion-based or cuVSLAM-based odometry mode
- **THEN** the RealSense launch SHALL keep T265 fisheye stream publishing disabled
