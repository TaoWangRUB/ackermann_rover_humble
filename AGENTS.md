# AGENTS.md

This repository supports autonomous feature implementation and validation
for a navigation stack consisting of:

RTAB-Map + Gazebo + Nav2 + ros2_control + PX4

The agent MUST execute the following workflow before declaring a task complete.

---

# 0. Prepare Docker env

Use the compose stack under `docker/` for all container management. Before starting any work:

1. Check whether the development container is already running:
   - `docker compose -f docker/docker-compose.yml ps`
   - If it is running, attach with `docker compose -f docker/docker-compose.yml exec ackermann_slam bash`.

2. If the container is not running, verify whether the image exists:
   - `docker images | grep ${IMAGE_NAME:-ackermann_rover}`
   - Inspect configuration via `cat docker/.env` (adjust ROS/Gazebo/Ubuntu versions if needed).

3. Build or rebuild the image (captures Dockerfile failures early):
   - `docker compose -f docker/docker-compose.yml build ackermann_slam`
   - If this fails, fix `docker/Dockerfile`, supporting scripts, or dependencies, then rebuild until it succeeds.

4. Start the environment:
   - `docker compose -f docker/docker-compose.yml up -d ackermann_slam`
   - Follow with `docker compose -f docker/docker-compose.yml logs -f ackermann_slam` to ensure entrypoint completes without errors.

5. Debugging tips if compose fails:
   - `docker compose -f docker/docker-compose.yml config` (verify resolved configuration).
   - `docker compose -f docker/docker-compose.yml ps -a` (check exit codes).
   - `docker compose -f docker/docker-compose.yml logs ackermann_slam` (inspect stack traces).
   - If necessary, tear down stale resources: `docker compose -f docker/docker-compose.yml down --remove-orphans`.

6. When the container is healthy, enter the shell with `docker compose -f docker/docker-compose.yml exec ackermann_slam bash`. Stay inside this shell for all subsequent steps.

7. Check Nividia driver is also loaded correctly and verify via run `docker compose -f docker/docker-compose.yml exec ackermann_slam bash` and then run `ros2 launch robot_bringup robot_bringup.launch.py` so that there's no nvidia driver related error/warning.

No build or startup errors are allowed. Fix issues immediately before moving on.

# 1. Build

    colcon build --symlink-install

If build fails:
- Fix compilation errors
- Rebuild until success

No build errors allowed.

---

# 2. Unit Tests

    colcon test --event-handlers console_direct+
    colcon test-result --verbose

All tests must pass.

If any fail:
- Diagnose
- Patch
- Re-run tests
- Repeat until green

---

# 3. Launch Full Simulation Stack

Launch full system using the project bringup entrypoint with rtabmap and nav2 set to true:

    ros2 launch robot_bringup robot_bringup.launch.py rtabmap:=true nav2:=true

Launch gazebo only

    ros2 launch robot_bringup robot_bringup.launch.py rtabmap:=false nav2:=false

Launch gazebo + rtabmap

    ros2 launch robot_bringup robot_bringup.launch.py rtabmap:=true nav2:=false

This launch must include:
- Gazebo world
- RTAB-Map SLAM
- Nav2 stack
- ros2_control
- PX4 SITL

Wait until:
- /map is publishing from RTAB-Map
- /odometry/filtered is publishing with frame `odom`
- Nav2 lifecycle nodes are active (planner, controller, behavior server)
- TF tree is stable with `map  odom  ackermann/base_footprint` available
- /ackermann/odom and /joint_states are publishing

Fail if:
- Any node crashes
- Lifecycle node stuck inactive
- Missing TF transforms
- ERROR logs appear

When all above checks in this section pass:
- Stop the simulation launch (Ctrl+C in the bringup terminal).
- Detach the running Docker container to return to the host shell.

---

# 4. SLAM Validation

## 4.1 Map Availability
Verify:
- /map topic publishes OccupancyGrid
- Map resolution matches config
- Map update rate > 1 Hz
- use `ros2 topic pub -r 1 /ackermann/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'ackermann/base_footprint'}, twist: {linear: {x: 1.0}, angular: {z: 0.5}}}"` to check robot is moving

## 4.2 Localization Stability
Robot stationary for 10 seconds:
- Pose drift < 0.1 m
- Yaw drift < 3 degrees

If drift exceeds threshold:
- Treat as failure

---

# 5. Navigation Validation

## 5.1 Global Planning Test

Send goal 5 meters away:

    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose ...

Validation criteria:
- Global plan generated
- No planner timeout
- Planning time < 2 seconds

Fail if:
- Planner retries > 3
- Planner aborts

---

## 5.2 Path Execution Test

Track execution metrics:

- Final pose error < 0.25 m
- Final yaw error < 5 degrees
- No oscillation recovery triggered more than once
- No collision in Gazebo

If collision detected → immediate failure.

---

# 6. Controller Tracking Quality

Measure:

- RMS lateral error < 0.2 m
- Max steering saturation < 95%
- No velocity spikes > configured limit

If using Ackermann:
- Steering angle must stay within hardware limits
- Command frequency ≥ 10 Hz

---

# 7. PX4 Command Validation

Verify:

- /cmd_vel is published
- PX4 receives commands (SITL confirmation)
- No offboard timeout
- No mode drop from OFFBOARD

Fail if:
- Offboard mode drops
- Failsafe triggered
- Command rate < required threshold

---

# 8. ros2_control Validation

Verify:

- Controller manager active (`/controller_manager/list_controllers` available)
- `joint_state_broadcaster` and `ackermann_steering_controller` in `active` state
- /joint_states publishing at the expected rate and wheel TFs present
- No hardware interface errors
- No controller switch failures

Command latency must remain < 50 ms.

---

# 9. Regression Check (Optional but Recommended)

Replay reference scenario:

    ros2 bag play reference_navigation.bag

Compare:

- Final pose error deviation < 5%
- Path length deviation < 5%

If deviation exceeds threshold:
- Treat as regression

---

# 10. Definition of Done

Task is complete only if:

✔ Build succeeds  
✔ All tests pass  
✔ SLAM stable  
✔ Planner generates path  
✔ Controller tracks within tolerance  
✔ No Gazebo collisions  
✔ PX4 remains in OFFBOARD  
✔ No lifecycle failures  
✔ No new ERROR logs  

Agent must not stop until all conditions satisfied.