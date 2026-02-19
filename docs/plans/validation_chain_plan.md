## Plan: Validate Simulation Chain

Verify TF/data flow end-to-end: first `description_robot`, then RTAB-Map, then sensor bridges, capturing results before resuming the Nav2 plan.

### Steps
1. Launch [description_robot/launch/gazebo_bringup.launch.py](src/description_robot/launch/gazebo_bringup.launch.py) by itself and run `ros2 run tf2_tools view_frames` to inspect the generated frames graph, confirming all frames from [donkey_sensors.urdf](src/description_robot/urdf/donkey_sensors.urdf) exist with the expected parents.
2. Still in Gazebo-only mode, run `ros2 run tf2_ros tf2_echo odom ackermann/base_footprint` and `ros2 topic echo /joint_states --once` to verify transforms and joint states are publishing correctly without RTAB-Map.
3. Add RTAB-Map via [rtabmap_bringup/launch/rtabmap_slam.launch.py](src/rtabmap_bringup/launch/rtabmap_slam.launch.py) and repeat `tf2_echo` for `map→odom` and `odom→ackermann/base_footprint`, plus `ros2 topic hz /odometry/filtered`, ensuring the EKF maintains a continuous TF chain when both visual/ICP odom sources are active.
4. Compare Gazebo-bridged topics from [gazebo_bringup.launch.py#L116-L131](src/description_robot/launch/gazebo_bringup.launch.py#L116-L131) with RTAB-Map remaps by running `ros2 topic info/echo` on `/ackermann/depth_camera/*`, `/l515/imu/raw`, `/rplidar/scan`, and `/ackermann/odom`, checking frame IDs and timestamps match what [rtabmap_slam.launch.py](src/rtabmap_bringup/launch/rtabmap_slam.launch.py) expects.
5. Log results (commands, pass/fail notes) in this plan and propagate any required changes into [docs/plans/nav2_bringup_plan.md](docs/plans/nav2_bringup_plan.md) before continuing Nav2 integration work.

### Progress Log
- 2026-02-19: Step 1 PASS. `ros2 run tf2_tools view_frames` produced [frames_2026-02-19_13.31.08.pdf](frames_2026-02-19_13.31.08.pdf) showing the expected `ackermann/base_footprint → base_link → sensors/wheels` hierarchy with no missing frames.
- 2026-02-19: Step 2 PASS. `ros2 launch rtabmap_bringup rtabmap_slam.launch.py` plus `ros2 run tf2_ros tf2_echo map odom`, `tf2_echo odom ackermann/base_footprint`, and `ros2 topic hz /odometry/filtered` confirmed the full TF chain, and driving with `ros2 topic pub -r 1 /ackermann/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'ackermann/base_footprint'}, twist: {linear: {x: 1.0}, angular: {z: 0.5}}}"` produced stable odometry despite `/odometry/filtered` running ~6 Hz. Latest TF snapshot: [frames_2026-02-19_13.42.08.pdf](frames_2026-02-19_13.42.08.pdf).
- 2026-02-19: Step 3 PASS. With Gazebo + RTAB-Map running, `ros2 topic info`/`ros2 topic echo --once` on `/ackermann/depth_camera/image`, `/ackermann/depth_camera/camera_info`, `/ackermann/depth_camera/depth_image`, `/l515/imu/raw`, `/rplidar/scan`, and `/ackermann/odom` showed matching frame IDs/timestamps against RTAB-Map remaps.
- 2026-02-19: Step 4 PASS. Captured findings here, refreshed the architecture docs with `rosgraph.png` and TF snapshots (see [docs/architecture/node_graph.md](../architecture/node_graph.md)), then pivoted focus back to [docs/plans/nav2_bringup_plan.md](nav2_bringup_plan.md) for the next validation phase.

### Further Considerations
1. Headless tooling only: Option A (`tf2_tools view_frames`) and Option C (`tf2_echo`) can run alongside launches; RViz remains optional.