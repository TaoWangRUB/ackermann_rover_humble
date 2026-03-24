cmake_minimum_required(VERSION 3.10)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cmd_vel_joint_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
