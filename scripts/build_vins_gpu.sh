#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
_ARCH="$(uname -m)"
OPENCV_PREFIX="${OPENCV_PREFIX:-/workspace/docker_cache/opencv-cuda/${_ARCH}/current}"
OPENCV_DIR="${OPENCV_DIR:-${OPENCV_PREFIX}/lib/cmake/opencv4}"

if [[ ! -f "${OPENCV_DIR}/OpenCVConfig.cmake" ]]; then
    echo "CUDA OpenCV is missing at ${OPENCV_DIR}. Run docker/install_vins_gpu_deps.sh first." >&2
    exit 1
fi

set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
set -u

cd "${REPO_ROOT}"
./scripts/apply_vins_fusion_patch.sh

colcon build --symlink-install \
    --packages-select camera_models vins loop_fusion global_fusion \
    --event-handlers console_direct+ \
    --cmake-args -DVINS_GPU_MODE=ON -DOpenCV_DIR="${OPENCV_DIR}"

colcon build --symlink-install \
    --packages-select vins_fusion_bringup robot_bringup rtabmap_bringup realsense_camera_bringup \
    --event-handlers console_direct+

cat <<EOF
Built VINS GPU packages against ${OPENCV_DIR}
Launch with:
  ros2 launch robot_bringup robot_bringup.launch.py use_gazebo:=false hw_enable_t265:=true use_vins_odom:=true depth_camera:=none rtabmap:=false nav2:=false rviz:=false
EOF
