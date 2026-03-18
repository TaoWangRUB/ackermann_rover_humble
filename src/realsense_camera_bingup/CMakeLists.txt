cmake_minimum_required(VERSION 3.10)
project(realsense_camera_bingup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# librealsense2 installed to /usr/local by the Dockerfile build step
find_package(realsense2 REQUIRED)

add_executable(realsense_camera_node
  src/realsense_camera_node.cpp
)

target_include_directories(realsense_camera_node
  PRIVATE include
)

ament_target_dependencies(realsense_camera_node
  rclcpp
  sensor_msgs
  nav_msgs
  cv_bridge
)

target_link_libraries(realsense_camera_node
  realsense2::realsense2
)

install(TARGETS realsense_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
