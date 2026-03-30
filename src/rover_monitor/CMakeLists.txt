cmake_minimum_required(VERSION 3.8)
project(rover_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Protobuf REQUIRED)

# Optional: px4_msgs may not be available in all build environments
find_package(px4_msgs REQUIRED)

# paho-mqtt-cpp
find_package(PahoMqttCpp REQUIRED)

# --- Custom messages ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CamStatus.msg"
  "msg/Px4Status.msg"
  "msg/JetsonStatus.msg"
  "msg/RoverHealth.msg"
  DEPENDENCIES std_msgs
)

# --- Protobuf code generation ---
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/rover_health.proto)

# --- Shared library for all components ---
add_library(${PROJECT_NAME}_components SHARED
  src/cam_probe.cpp
  src/px4_probe.cpp
  src/jetson_probe.cpp
  src/system_metrics_provider.cpp
  src/jetson_metrics_provider.cpp
  src/x86_mock_metrics_provider.cpp
  src/aggregator.cpp
  src/telemetry_publisher.cpp
  ${PROTO_SRCS}
)

target_include_directories(${PROJECT_NAME}_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Protobuf_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}  # for generated protobuf headers
)

ament_target_dependencies(${PROJECT_NAME}_components
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  tf2_ros
  px4_msgs
)

target_link_libraries(${PROJECT_NAME}_components
  ${Protobuf_LIBRARIES}
  PahoMqttCpp::paho-mqttpp3
)

# Use typesupport from generated messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_components ${cpp_typesupport_target})

# --- Register components ---
rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "rover_monitor::CamProbe"
  EXECUTABLE cam_probe
)
rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "rover_monitor::Px4Probe"
  EXECUTABLE px4_probe
)
rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "rover_monitor::JetsonProbe"
  EXECUTABLE jetson_probe
)
rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "rover_monitor::Aggregator"
  EXECUTABLE monitor_aggregator
)
rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "rover_monitor::TelemetryPublisher"
  EXECUTABLE telemetry_publisher
)

# --- Install ---
install(TARGETS ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY proto/
  DESTINATION share/${PROJECT_NAME}/proto
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_components)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  tf2_ros
  px4_msgs
)

ament_package()
