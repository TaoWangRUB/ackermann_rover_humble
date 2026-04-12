cmake_minimum_required(VERSION 3.10)
project(cuvslam_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# ─── cuVSLAM library discovery ─────────────────────────────────────────────
#
# Strategy:
#
#   1. explicit -DCUVSLAM_PREFIX=/path/to/install           (CMake override)
#   2. CUVSLAM_PREFIX environment variable
#   3. docker cache install prefixes under /docker_cache or /workspace/docker_cache
#   4. vendored source/build tree under src/cuVSLAM (or CUVSLAM_SRC_DIR/CUVSLAM_DST_DIR)
#   5. /usr/local                                          (system install)
#
# If the library is not yet built, CUVSLAM_FOUND stays FALSE and we skip the
# node target so downstream colcon builds succeed.  Phase-0 smoke test still
# compiles because its own target uses dlopen() and only needs libdl.
# ───────────────────────────────────────────────────────────────────────────
if(NOT DEFINED CUVSLAM_PREFIX AND DEFINED ENV{CUVSLAM_PREFIX})
  set(CUVSLAM_PREFIX "$ENV{CUVSLAM_PREFIX}")
endif()

set(CUVSLAM_PREFIX_HINTS "")
if(CUVSLAM_PREFIX)
  list(APPEND CUVSLAM_PREFIX_HINTS "${CUVSLAM_PREFIX}")
endif()

file(GLOB CUVSLAM_DOCKER_CACHE_PREFIXES
  "/docker_cache/cuvslam/*/install"
  "/docker_cache/cuvslam/*/current/install"
  "/workspace/docker_cache/cuvslam/*/install"
  "/workspace/docker_cache/cuvslam/*/current/install"
)
list(APPEND CUVSLAM_PREFIX_HINTS ${CUVSLAM_DOCKER_CACHE_PREFIXES})
list(APPEND CUVSLAM_PREFIX_HINTS "/usr/local")

set(CUVSLAM_SOURCE_HINTS "")
if(DEFINED ENV{CUVSLAM_SRC_DIR})
  list(APPEND CUVSLAM_SOURCE_HINTS "$ENV{CUVSLAM_SRC_DIR}")
endif()
list(APPEND CUVSLAM_SOURCE_HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../cuVSLAM")

set(CUVSLAM_BUILD_HINTS "")
if(DEFINED ENV{CUVSLAM_DST_DIR})
  list(APPEND CUVSLAM_BUILD_HINTS "$ENV{CUVSLAM_DST_DIR}")
endif()
foreach(CUVSLAM_SOURCE_HINT IN LISTS CUVSLAM_SOURCE_HINTS)
  list(APPEND CUVSLAM_BUILD_HINTS "${CUVSLAM_SOURCE_HINT}/build")
endforeach()

find_path(CUVSLAM_INCLUDE_DIR
  NAMES cuvslam/cuvslam2.h cuvslam2.h
  PATHS
    ${CUVSLAM_PREFIX_HINTS}
    ${CUVSLAM_SOURCE_HINTS}
  NO_DEFAULT_PATH
  PATH_SUFFIXES
    include
    libs
)

find_library(CUVSLAM_LIBRARY
  NAMES cuvslam
  PATHS
    ${CUVSLAM_PREFIX_HINTS}
    ${CUVSLAM_BUILD_HINTS}
  NO_DEFAULT_PATH
  PATH_SUFFIXES
    lib
    lib64
    bin
)

if(CUVSLAM_INCLUDE_DIR AND CUVSLAM_LIBRARY)
  set(CUVSLAM_FOUND TRUE)
  message(STATUS "cuvslam_bringup: CUVSLAM_INCLUDE_DIR=${CUVSLAM_INCLUDE_DIR}")
  message(STATUS "cuvslam_bringup: CUVSLAM_LIBRARY=${CUVSLAM_LIBRARY}")
else()
  set(CUVSLAM_FOUND FALSE)
  message(WARNING
    "cuvslam_bringup: libcuvslam not found. "
    "Building package metadata only; cuvslam_odom_node target will be "
    "skipped. Set -DCUVSLAM_PREFIX=<install-root>, export "
    "CUVSLAM_SRC_DIR/CUVSLAM_DST_DIR for the vendored source tree, or run "
    "docker/install_cuvslam_deps.sh to populate the cache."
  )
endif()

# ─── cuvslam_odom_node ─────────────────────────────────────────────────────
if(CUVSLAM_FOUND)
  get_filename_component(CUVSLAM_LIBRARY_DIR "${CUVSLAM_LIBRARY}" DIRECTORY)
  add_executable(cuvslam_odom_node
    src/cuvslam_odom_node.cpp
  )
  target_include_directories(cuvslam_odom_node PRIVATE
    ${CUVSLAM_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
  )
  set_target_properties(cuvslam_odom_node PROPERTIES
    BUILD_RPATH "${CUVSLAM_LIBRARY_DIR}"
    INSTALL_RPATH "${CUVSLAM_LIBRARY_DIR}"
  )
  target_link_libraries(cuvslam_odom_node
    ${CUVSLAM_LIBRARY}
    ${OpenCV_LIBRARIES}
  )
  ament_target_dependencies(cuvslam_odom_node
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    tf2
    tf2_ros
    message_filters
    cv_bridge
  )
  install(TARGETS cuvslam_odom_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# ─── phase-0 smoke test (optional, x86_64 dev only) ────────────────────────
if(BUILD_TESTING OR NOT DEFINED BUILD_TESTING)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/smoke_test_cuvslam.cpp")
    add_executable(smoke_test_cuvslam test/smoke_test_cuvslam.cpp)
    target_link_libraries(smoke_test_cuvslam dl)
    install(TARGETS smoke_test_cuvslam
      DESTINATION lib/${PROJECT_NAME}
    )
  endif()
endif()

# ─── install non-code assets ───────────────────────────────────────────────
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
