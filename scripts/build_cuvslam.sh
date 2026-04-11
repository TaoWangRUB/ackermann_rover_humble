#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${SCRIPT_DIR}" != "/workspace/scripts" ]]; then
    # Host-side entrypoint: mirror the other operator workflows and execute the
    # actual build inside the Docker dev container.
    # shellcheck source=/dev/null
    source "${SCRIPT_DIR}/lib/dc.sh"
    INNER_CMD="source /opt/ros/\$ROS_DISTRO/setup.bash && "
    INNER_CMD+="if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi && "
    INNER_CMD+="bash /workspace/scripts/build_cuvslam.sh"
    xdcomp exec -T ackermann_slam bash -lc "${INNER_CMD}"
fi

REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CUVSLAM_SRC_DIR="${CUVSLAM_SRC_DIR:-${REPO_ROOT}/src/cuVSLAM}"
CUVSLAM_DST_DIR="${CUVSLAM_DST_DIR:-${CUVSLAM_SRC_DIR}/build}"
ARCH="$(uname -m)"

if [[ ! -d "${CUVSLAM_SRC_DIR}" ]]; then
    echo "cuVSLAM source tree is missing at ${CUVSLAM_SRC_DIR}" >&2
    exit 1
fi

if ! command -v gcc-11 >/dev/null 2>&1 || ! command -v g++-11 >/dev/null 2>&1; then
    echo "gcc-11/g++-11 are required inside the container to build cuVSLAM." >&2
    exit 1
fi

if [[ ! -x /usr/local/cuda/bin/nvcc ]]; then
    echo "/usr/local/cuda/bin/nvcc is missing. Ensure the CUDA toolkit is mounted into the container." >&2
    exit 1
fi

if [[ "${ARCH}" == "aarch64" ]]; then
    BUILD_JOBS="${BUILD_JOBS:-2}"
else
    BUILD_JOBS="${BUILD_JOBS:-$(nproc)}"
fi

echo "Building cuVSLAM from ${CUVSLAM_SRC_DIR}"
echo "  build dir: ${CUVSLAM_DST_DIR}"
echo "  arch:      ${ARCH}"
echo "  jobs:      ${BUILD_JOBS}"

cmake -S "${CUVSLAM_SRC_DIR}" -B "${CUVSLAM_DST_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER=/usr/bin/gcc-11 \
    -DCMAKE_CXX_COMPILER=/usr/bin/g++-11 \
    -DCMAKE_CUDA_HOST_COMPILER=/usr/bin/g++-11 \
    -DUSE_RERUN=OFF \
    -DUSE_CERES=OFF \
    -DUSE_NVTX=OFF

cmake --build "${CUVSLAM_DST_DIR}" -j"${BUILD_JOBS}" --target cuvslam

cd "${REPO_ROOT}"
colcon build --symlink-install \
    --packages-select cuvslam_bringup robot_bringup rtabmap_bringup description_robot realsense_camera_bringup \
    --event-handlers console_direct+

"${REPO_ROOT}/install/cuvslam_bringup/lib/cuvslam_bringup/smoke_test_cuvslam"

cat <<EOF
Built cuVSLAM and related bringup packages.
Launch with:
  ./scripts/start_ros2_nodes.sh --hw --depth-camera=d435i --cuvslam-odom --rtabmap
Debug with:
  ./scripts/debug_vio.sh
EOF
