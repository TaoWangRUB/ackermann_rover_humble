## ADDED Requirements

### Requirement: cuVSLAM odometry mode orchestration
The top-level bringup SHALL expose a `use_cuvslam_odom` launch mode that orchestrates the required hardware and subsystem configuration for cuVSLAM odometry.

#### Scenario: Hardware bringup with cuVSLAM odometry
- **WHEN** `use_gazebo:=false` and `use_cuvslam_odom:=true`
- **THEN** `robot_bringup.launch.py` SHALL enable the T265 hardware path, enable the T265 fisheye streams needed by cuVSLAM, and include the cuVSLAM bringup launch

### Requirement: cuVSLAM mode propagation to SLAM bringup
The top-level bringup SHALL pass cuVSLAM odometry selection into the SLAM bringup so downstream odometry-source selection is consistent.

#### Scenario: RTAB-Map launched with cuVSLAM mode
- **WHEN** `rtabmap:=true` and `use_cuvslam_odom:=true`
- **THEN** the RTAB-Map bringup include SHALL receive launch arguments that select cuVSLAM-derived odometry as the preferred external odometry source
