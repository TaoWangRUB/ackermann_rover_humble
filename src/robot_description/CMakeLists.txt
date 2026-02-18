cmake_minimum_required(VERSION 3.10)
project(robot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
