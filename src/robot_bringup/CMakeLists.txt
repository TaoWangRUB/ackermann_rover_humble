cmake_minimum_required(VERSION 3.10)
project(robot_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

# C++ port of the Python relay — same kinematics, much lower CPU.
add_executable(cmd_vel_joint_relay src/cmd_vel_joint_relay.cpp)
ament_target_dependencies(cmd_vel_joint_relay
  rclcpp
  geometry_msgs
  sensor_msgs
)

install(TARGETS cmd_vel_joint_relay
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/t265_sim_camera_info.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
