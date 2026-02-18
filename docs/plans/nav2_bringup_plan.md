## Plan: Integrate Nav2 Bringup

Introduce a dedicated Nav2 bringup package/config wired into `robot_bringup`, keeping `/cmd_vel_nav` and `/odometry/filtered` as the control/localization contracts while reusing the stock Nav2 sample behavior-tree XML for the first iteration.

### Steps
1. Scaffold `nav2_bringup` package under [src/navigation](src/navigation) with package.xml/CMake tying in `nav2_bringup`, `nav2_controller`, `nav2_planner`, `bt_navigator`, `behavior_tree`, `map_server`, `waypoint_follower`, and install `launch/` plus [config/nav2_ackermann.yaml](src/navigation/config/nav2_ackermann.yaml).
2. Expand [nav2_ackermann.yaml](src/navigation/config/nav2_ackermann.yaml) to define controller, planner, global/local costmaps, smoother, recoveries, lifecycle manager, and reference `/odometry/filtered` + `/cmd_vel_nav` while pointing `bt_navigator` to the reused sample Nav2 XML (e.g., `bt_navigator.xml` from `nav2_bt_navigator` share dir).
3. Add `nav2_bringup.launch.py` that loads the config, starts the Nav2 lifecycle stack (controller/planner/bt_navigator/behavior_server/map_server/waypoint_follower), and remaps outputs to `/cmd_vel_nav`; include args for `bt_xml`, `use_sim_time`, and config overrides.
4. Update [robot_bringup/launch/robot_bringup.launch.py](src/robot_bringup/launch/robot_bringup.launch.py) with a `nav2` boolean argument gating inclusion of the new launch, ensuring sequence: Gazebo → RTAB-Map → Nav2 once `/odometry/filtered` is available.
5. Document the Nav2 bringup in README + architecture docs, including instructions to launch (`ros2 launch robot_bringup robot_bringup.launch.py nav2:=true`) and note reliance on the sample behavior tree.

### Further Considerations
1. Confirm topic alignment: keep `/cmd_vel_nav` and `/odometry/filtered` naming or adopt Nav2 defaults?
2. Behavior tree source: reuse Nav2 sample XML or craft rover-specific tree with safety hooks?
