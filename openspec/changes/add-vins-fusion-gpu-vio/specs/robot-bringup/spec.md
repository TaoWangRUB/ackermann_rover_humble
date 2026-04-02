## ADDED Requirements

### Requirement: VINS odometry mode orchestration
The top-level bringup SHALL expose a `use_vins_odom` launch mode that orchestrates the required hardware and subsystem configuration for VINS-Fusion odometry.

#### Scenario: Hardware bringup with VINS odometry
- **WHEN** `use_gazebo:=false` and `use_vins_odom:=true`
- **THEN** `robot_bringup.launch.py` SHALL enable the T265 hardware path, enable the T265 fisheye streams needed by VINS, and include the VINS-Fusion bringup launch

### Requirement: VINS mode propagation to SLAM bringup
The top-level bringup SHALL pass VINS odometry selection into the SLAM bringup so downstream odometry-source selection is consistent.

#### Scenario: RTAB-Map launched with VINS mode
- **WHEN** `rtabmap:=true` and `use_vins_odom:=true`
- **THEN** the RTAB-Map bringup include SHALL receive launch arguments that select VINS-derived odometry as the preferred external odometry source
