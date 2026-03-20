cmake_minimum_required(VERSION 3.8)
project(realsense_camera_bingup)

# Default to C++17 for ROS 2 Jazzy
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Standard compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Use manually installed librealsense from /usr/local (v2.51) — avoids the ROS-bundled
# librealsense2 2.55+ which dropped L515 support.
find_package(realsense2 REQUIRED PATHS /usr/local NO_DEFAULT_PATH)

include_directories(
  include
  ${realsense2_INCLUDE_DIRS}
)

# Define the node executable
add_executable(realsense_camera_node
  src/realsense_camera_node.cpp
)

ament_target_dependencies(realsense_camera_node
  rclcpp
  sensor_msgs
  nav_msgs
)

target_link_libraries(realsense_camera_node
  ${realsense2_LIBRARY}
)

# Install rules
install(TARGETS
  realsense_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()