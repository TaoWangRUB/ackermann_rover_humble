cmake_minimum_required(VERSION 3.8)
project(px4_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(px4_msgs REQUIRED)

include_directories(include)

# --- C++ mode executables ---

# 1. Offboard Trajectory Mode
add_executable(offboard_trajectory_mode
  src/offboard_trajectory_main.cpp
)
ament_target_dependencies(offboard_trajectory_mode
  rclcpp geometry_msgs tf2_ros tf2_geometry_msgs px4_ros2_cpp px4_msgs
)

# 2. Rover Speed Steering Mode
add_executable(rover_speed_steering_mode
  src/rover_speed_steering_main.cpp
)
ament_target_dependencies(rover_speed_steering_mode
  rclcpp geometry_msgs px4_ros2_cpp px4_msgs
)

# 3. Rover Speed Attitude Mode
add_executable(rover_speed_attitude_mode
  src/rover_speed_attitude_main.cpp
)
ament_target_dependencies(rover_speed_attitude_mode
  rclcpp geometry_msgs px4_ros2_cpp px4_msgs
)

# 4. Rover Manual Mode (open-loop throttle + steering)
add_executable(rover_manual_mode
  src/rover_manual_main.cpp
)
ament_target_dependencies(rover_manual_mode
  rclcpp geometry_msgs px4_ros2_cpp px4_msgs
)

# --- Install targets ---

install(TARGETS
  offboard_trajectory_mode
  rover_speed_steering_mode
  rover_speed_attitude_mode
  rover_manual_mode
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/px4_offboard_control.py
  scripts/px4_vision_odom.py
  scripts/px4_vehicle_odometry.py
  scripts/px4_mavlink_vpe.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
