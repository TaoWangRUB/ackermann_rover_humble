cmake_minimum_required(VERSION 3.10)
project(cuvslam_smoke_test)

add_executable(smoke_test_cuvslam test/smoke_test_cuvslam.cpp)

target_link_libraries(smoke_test_cuvslam dl)

# Only link cuVSLAM if present (for CI/dev safety)
find_library(CUVSLAM_LIB cuvslam PATHS /workspace/docker_cache/cuvslam/x86_64/current/install/lib NO_DEFAULT_PATH)
if(CUVSLAM_LIB)
    target_link_libraries(smoke_test_cuvslam ${CUVSLAM_LIB})
endif()
