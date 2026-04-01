cmake_minimum_required(VERSION 3.8)
project(realsense_camera_bringup LANGUAGES CXX)

# Default to C++17 for ROS 2 Jazzy
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Standard compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- CUDA (optional, for GPU-accelerated depth alignment on Jetson) ----------
# On Jetson (aarch64): prefer the host-mounted CUDA 11.4 toolkit (JetPack 5)
# because the host driver is 11.4 and the runtime must match.
# CUDA 11.4 nvcc only supports up to GCC 11, so we use gcc-11 as host compiler.
# On x86_64: fall back to whatever /usr/local/cuda provides (12.x in container).
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" AND EXISTS "/usr/local/cuda-11.4/bin/nvcc")
  set(CMAKE_CUDA_COMPILER "/usr/local/cuda-11.4/bin/nvcc" CACHE FILEPATH "")
  set(CUDAToolkit_ROOT    "/usr/local/cuda-11.4"           CACHE PATH     "")
  # CUDA 11.4 nvcc can't parse GCC 13+ headers — use GCC 11 for host compilation
  find_program(_GCC11 gcc-11)
  if(_GCC11)
    set(CMAKE_CUDA_HOST_COMPILER "${_GCC11}" CACHE FILEPATH "")
  endif()
elseif(EXISTS "/usr/local/cuda/bin/nvcc")
  set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc" CACHE FILEPATH "")
  set(CUDAToolkit_ROOT    "/usr/local/cuda"           CACHE PATH     "")
endif()
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  set(HAVE_CUDA TRUE)
  message(STATUS "CUDA found — GPU-accelerated depth alignment enabled")
  message(STATUS "  nvcc: ${CMAKE_CUDA_COMPILER}")
  message(STATUS "  toolkit root: ${CUDAToolkit_ROOT}")
  if(CMAKE_CUDA_HOST_COMPILER)
    message(STATUS "  host compiler: ${CMAKE_CUDA_HOST_COMPILER}")
  endif()
else()
  set(HAVE_CUDA FALSE)
  message(STATUS "CUDA not found — falling back to CPU depth alignment (rs2::align)")
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

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

if(HAVE_CUDA)
  find_package(CUDAToolkit REQUIRED)

  # Build CUDA alignment as a static library and link it into the node.
  # Statically link cudart so the binary never picks up the wrong libcudart.so
  # from ldconfig at runtime.
  add_library(cuda_align STATIC src/cuda_align.cu)
  target_include_directories(cuda_align PUBLIC include)
  set_target_properties(cuda_align PROPERTIES
    CUDA_STANDARD 14
    CUDA_SEPARABLE_COMPILATION ON
    POSITION_INDEPENDENT_CODE ON
    # Xavier = sm_72, Orin = sm_87
    CUDA_ARCHITECTURES "72;87"
  )
  target_link_libraries(cuda_align
    CUDA::cudart_static      # portable: finds correct lib path for x86/aarch64
    pthread dl rt
  )
  target_link_libraries(realsense_camera_node cuda_align)
  target_compile_definitions(realsense_camera_node PRIVATE HAVE_CUDA)
endif()

ament_target_dependencies(realsense_camera_node
  rclcpp
  sensor_msgs
  nav_msgs
  tf2_ros
  geometry_msgs
)

target_link_libraries(realsense_camera_node
  ${realsense2_LIBRARY}
)

add_executable(odom_tf_relay
  src/odom_tf_relay.cpp
)
ament_target_dependencies(odom_tf_relay
  rclcpp nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs
)
target_include_directories(odom_tf_relay PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)

# Install rules
install(TARGETS
  realsense_camera_node
  odom_tf_relay
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