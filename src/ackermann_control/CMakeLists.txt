cmake_minimum_required(VERSION 3.8)
project(ackermann_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(ackermann_controller src/ackermann_controller.cpp)
ament_target_dependencies(ackermann_controller
  rclcpp rclcpp_lifecycle geometry_msgs ackermann_msgs)

install(TARGETS ackermann_controller DESTINATION lib/${PROJECT_NAME})
ament_package()
