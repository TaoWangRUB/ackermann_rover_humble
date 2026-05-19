cmake_minimum_required(VERSION 3.8)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0104)
  cmake_policy(SET CMP0104 NEW)
endif()

execute_process(
  COMMAND uname -m
  OUTPUT_VARIABLE _HOST_ARCH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Jetson builds use Ubuntu 24.04 plus the CUDA 11.x toolkit exposed inside the
# running container, which still requires GCC 11 as the host compiler. Set
# these before project()/enable_language(CUDA) so CMake's compiler detection
# and the actual build use the same toolchain.
if(_HOST_ARCH STREQUAL "aarch64")
  find_program(_JETSON_GCC11 gcc-11)
  find_program(_JETSON_GPP11 g++-11)
  if(_JETSON_GCC11 AND _JETSON_GPP11)
    set(CMAKE_C_COMPILER "${_JETSON_GCC11}" CACHE FILEPATH "" FORCE)
    set(CMAKE_CXX_COMPILER "${_JETSON_GPP11}" CACHE FILEPATH "" FORCE)
    set(CMAKE_CUDA_HOST_COMPILER "${_JETSON_GPP11}" CACHE FILEPATH "" FORCE)
    set(ENV{CUDAHOSTCXX} "${_JETSON_GPP11}")
  endif()
endif()

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
# On Jetson, use whichever CUDA toolkit the running container exposes.
# On x86_64, keep using the default /usr/local/cuda layout.
unset(_CUDA_ROOT)
foreach(_cuda_env IN ITEMS CUDA_HOME CUDA_PATH)
  if(DEFINED ENV{${_cuda_env}} AND EXISTS "$ENV{${_cuda_env}}/bin/nvcc")
    set(_CUDA_ROOT "$ENV{${_cuda_env}}")
    break()
  endif()
endforeach()
if(NOT _CUDA_ROOT AND CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  if(EXISTS "/usr/local/cuda/bin/nvcc")
    set(_CUDA_ROOT "/usr/local/cuda")
  else()
    foreach(_candidate IN ITEMS /usr/local/cuda-12.6 /usr/local/cuda-12.4 /usr/local/cuda-11.8 /usr/local/cuda-11.6 /usr/local/cuda-11.4)
      if(EXISTS "${_candidate}/bin/nvcc")
        set(_CUDA_ROOT "${_candidate}")
        break()
      endif()
    endforeach()
  endif()
elseif(NOT _CUDA_ROOT AND EXISTS "/usr/local/cuda/bin/nvcc")
  set(_CUDA_ROOT "/usr/local/cuda")
elseif(NOT _CUDA_ROOT)
  foreach(_candidate IN ITEMS /usr/local/cuda-12.8 /usr/local/cuda-12.6 /usr/local/cuda-12.4 /usr/local/cuda-12.2 /usr/local/cuda-12.0 /usr/local/cuda-11.8 /usr/local/cuda-11.6 /usr/local/cuda-11.4)
    if(EXISTS "${_candidate}/bin/nvcc")
      set(_CUDA_ROOT "${_candidate}")
      break()
    endif()
  endforeach()
endif()

if(_CUDA_ROOT)
  # Force-refresh these cache entries so moving between Jetson and x86, or
  # switching from a versioned CUDA mount to /usr/local/cuda, doesn't leave
  # CMake stuck on a stale compiler path from an older build tree.
  set(CMAKE_CUDA_COMPILER "${_CUDA_ROOT}/bin/nvcc" CACHE FILEPATH "" FORCE)
  set(CUDAToolkit_ROOT    "${_CUDA_ROOT}"          CACHE PATH     "" FORCE)
  set(CMAKE_CUDA_RUNTIME_LIBRARY Shared)
  # Ubuntu 24.04 glibc 2.39 exposes vector typedefs through
  # bits/math-vector.h that CUDA 11.4's cicc cannot parse on Jetson. Include
  # a tiny compat header before cuda_runtime.h so the compiler-id probe and
  # actual .cu builds see a tamed __GNUC_PREREQ macro.
  if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(_CUDA_GLIBC_COMPAT_HEADER
      "${CMAKE_CURRENT_SOURCE_DIR}/include/realsense_camera_bringup/cuda_glibc_compat.h")
    string(APPEND CMAKE_CUDA_FLAGS_INIT
      " -Xcompiler=-include -Xcompiler=${_CUDA_GLIBC_COMPAT_HEADER}")
  endif()
else()
  unset(CMAKE_CUDA_COMPILER CACHE)
  unset(CUDAToolkit_ROOT CACHE)
endif()
if(CMAKE_CUDA_COMPILER AND EXISTS "${CMAKE_CUDA_COMPILER}")
  enable_language(CUDA)
  set(HAVE_CUDA TRUE)
  if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(_CUDA_ARCHITECTURES "72;87")
  else()
    # Match the local desktop GPU on x86 instead of hard-wiring Jetson SMs.
    set(_CUDA_ARCHITECTURES "native")
  endif()
  message(STATUS "CUDA found — GPU-accelerated depth alignment enabled")
  message(STATUS "  nvcc: ${CMAKE_CUDA_COMPILER}")
  message(STATUS "  toolkit root: ${CUDAToolkit_ROOT}")
  message(STATUS "  architectures: ${_CUDA_ARCHITECTURES}")
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

# Use manually installed librealsense from /usr/local (v2.51) when available.
# CI runners do not provide the hardware SDK, so fall back to a stub node
# there instead of aborting the whole workspace configure step.
find_package(realsense2 QUIET PATHS /usr/local NO_DEFAULT_PATH)
if(realsense2_FOUND)
  set(HAVE_LIBREALSENSE TRUE)
  message(STATUS "librealsense2 found — building RealSense hardware node")
else()
  set(HAVE_LIBREALSENSE FALSE)
  message(WARNING
    "librealsense2 was not found under /usr/local. "
    "Building a stub realsense_camera_node so CI and non-hardware builds can proceed. "
    "Hardware RealSense launches will require librealsense2 to be installed before rebuilding.")
endif()

include_directories(include)
if(HAVE_LIBREALSENSE)
  include_directories(${realsense2_INCLUDE_DIRS})
endif()

if(HAVE_LIBREALSENSE)
  set(REALSENSE_CAMERA_NODE_SOURCES src/realsense_camera_node.cpp)
else()
  set(REALSENSE_CAMERA_NODE_SOURCES src/realsense_camera_node_stub.cpp)
endif()

add_executable(realsense_camera_node ${REALSENSE_CAMERA_NODE_SOURCES})

if(HAVE_CUDA AND HAVE_LIBREALSENSE)
  find_package(CUDAToolkit REQUIRED)

  # Build CUDA alignment as a static library and link it into the node.
  # Use the toolkit's shared cudart inside the running container. Jetson's
  # CUDA 11.4 static cudart pulls in librt.a during device link, which is not
  # available on this Ubuntu 24.04 image.
  add_library(cuda_align STATIC src/cuda_align.cu)
  target_include_directories(cuda_align PUBLIC include)
  set_target_properties(cuda_align PROPERTIES
    CUDA_STANDARD 14
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RUNTIME_LIBRARY Shared
    POSITION_INDEPENDENT_CODE ON
    CUDA_ARCHITECTURES "${_CUDA_ARCHITECTURES}"
  )
  target_link_libraries(cuda_align
    CUDA::cudart             # portable: finds correct lib path for x86/aarch64
    pthread dl
  )
  set_target_properties(realsense_camera_node PROPERTIES
    CUDA_ARCHITECTURES "${_CUDA_ARCHITECTURES}"
    CUDA_RUNTIME_LIBRARY Shared
  )
  target_link_libraries(realsense_camera_node cuda_align)
  target_compile_definitions(realsense_camera_node PRIVATE HAVE_CUDA)
elseif(HAVE_CUDA AND NOT HAVE_LIBREALSENSE)
  message(STATUS "CUDA found but librealsense2 is unavailable — skipping CUDA depth alignment build")
endif()

set(REALSENSE_CAMERA_NODE_DEPS rclcpp)
if(HAVE_LIBREALSENSE)
  list(APPEND REALSENSE_CAMERA_NODE_DEPS
    rcl_interfaces
    sensor_msgs
    nav_msgs
    tf2_ros
    geometry_msgs
  )
endif()
ament_target_dependencies(realsense_camera_node ${REALSENSE_CAMERA_NODE_DEPS})

if(HAVE_LIBREALSENSE)
  target_link_libraries(realsense_camera_node ${realsense2_LIBRARY})
endif()

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

if(HAVE_LIBREALSENSE)
  install(
    DIRECTORY include/
    DESTINATION include
  )
endif()

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
