---
name: ROS 2 Development
description: Essential commands and workflows for building, testing, and debugging ROS 2 workspaces.
---

# ROS 2 Development Skill

This skill provides you with instructions on how to interact with, build, test, and debug ROS 2 code. Use these practices whenever you encounter a ROS 2 package or workspace.

## 1. Environment Setup
Before running ROS 2 commands or building code locally (if not inside the pre-configured Docker container), always source the workspace:
```bash
# Source the ROS 2 installation (Adjust 'humble' to the correct distro if needed)
source /opt/ros/humble/setup.bash
# Source the local workspace
source install/setup.bash
```

If you are using the project's Docker container, the entrypoint usually sources these automatically, but when you `exec` into a running container you may need to source the workspace manually.

## 2. Building the Workspace
ROS 2 uses `colcon` as its build tool. 
- **Build all packages**:
  ```bash
  colcon build --symlink-install
  ```
- **Build specific packages** (faster during iterative development):
  ```bash
  colcon build --symlink-install --packages-select <package_name1> <package_name2>
  ```
- **Build packages up to a specific package** (builds dependencies too):
  ```bash
  colcon build --symlink-install --packages-up-to <package_name>
  ```

## 3. Testing
ROS 2 uses `colcon test` to run tests (gtest/pytest).
- **Run all tests**:
  ```bash
  colcon test --event-handlers console_direct+
  ```
- **Run tests for a specific package**:
  ```bash
  colcon test --event-handlers console_direct+ --packages-select <package_name>
  ```
- **Check test results**:
  After running tests, you MUST examine the results:
  ```bash
  colcon test-result --verbose
  ```

## 4. Debugging & Introspection
To understand the running ROS graph, use the CLI tools:
- **Topics**: Data streams
  - `ros2 topic list`
  - `ros2 topic info /topic_name`
  - `ros2 topic echo /topic_name` (use `--once` if you only need one message)
  - `ros2 topic hz /topic_name`
- **Nodes**: Running executables
  - `ros2 node list`
  - `ros2 node info /node_name`
- **Parameters**: Node configuration
  - `ros2 param list`
  - `ros2 param get /node_name param_name`
  - `ros2 param set /node_name param_name value`
- **Services/Actions**: RPCs and long-running tasks
  - `ros2 service list` & `ros2 service call ...`
  - `ros2 action list` & `ros2 action send_goal ...`

## 5. Lifecycle Nodes
Nav2 and hardware interfaces heavily use Managed (Lifecycle) nodes.
- **Check State**:
  ```bash
  ros2 lifecycle get /node_name
  ```
- **Change State** (Usually Nav2 handles this, but for manual debugging):
  ```bash
  ros2 lifecycle set /node_name configure
  ros2 lifecycle set /node_name activate
  ```

## 6. Launch Files
ROS 2 uses Python, XML, or YAML launch files. The standard way to bring up a system:
```bash
ros2 launch <package_name> <launch_file_name> [arguments...]
```
Example:
```bash
ros2 launch robot_bringup robot_bringup.launch.py rtabmap:=true nav2:=false
```

When changing launch files, pay attention to the `Node(..., parameters=[...], remappings=[...])` structures.
