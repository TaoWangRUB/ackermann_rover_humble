cmake_minimum_required(VERSION 3.8)
project(safety)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(watchdog_node src/watchdog_node.cpp)
ament_target_dependencies(watchdog_node rclcpp std_msgs ackermann_msgs)

install(TARGETS watchdog_node DESTINATION lib/${PROJECT_NAME})

ament_package()
