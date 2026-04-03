## ADDED Requirements

### Requirement: VINS odometry mode orchestration
The top-level bringup SHALL expose a `use_vins_odom` launch mode that orchestrates the required hardware and subsystem configuration for VINS-Fusion odometry.

#### Scenario: Hardware bringup with VINS odometry
- **WHEN** `use_gazebo:=false` and `use_vins_odom:=true`
- **THEN** `robot_bringup.launch.py` SHALL enable the T265 hardware path, enable the T265 fisheye streams needed by VINS, and include the VINS-Fusion bringup launch

#### Scenario: Hardware bringup without VINS odometry
- **WHEN** `use_gazebo:=false`, the T265 hardware path is enabled, and `use_vins_odom:=false`
- **THEN** `robot_bringup.launch.py` SHALL keep T265 fisheye stream publishing disabled unless another explicitly selected mode requires it

### Requirement: VINS mode propagation to SLAM bringup
The top-level bringup SHALL pass VINS odometry selection into the SLAM bringup so downstream odometry-source selection is consistent.

#### Scenario: RTAB-Map launched with VINS mode
- **WHEN** `rtabmap:=true` and `use_vins_odom:=true`
- **THEN** the RTAB-Map bringup include SHALL receive launch arguments that select VINS-derived odometry as the preferred external odometry source
