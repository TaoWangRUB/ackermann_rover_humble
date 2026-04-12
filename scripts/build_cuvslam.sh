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

# ---------------------------------------------------------------------------
# Jetson (aarch64) workarounds
# ---------------------------------------------------------------------------
if [[ "${ARCH}" == "aarch64" ]]; then
    CUDA_MAJOR=$(/usr/local/cuda/bin/nvcc --version | sed -n 's/.*release \([0-9]*\)\.\([0-9]*\).*/\1/p')
    CUDA_MINOR=$(/usr/local/cuda/bin/nvcc --version | sed -n 's/.*release \([0-9]*\)\.\([0-9]*\).*/\2/p')

    # 1) -arch=all is only supported from CUDA 11.5+. On JetPack 5.x (CUDA 11.4)
    #    we must replace it with an explicit SM target. Xavier NX/AGX = sm_72,
    #    Orin = sm_87. Auto-detect from /proc/device-tree/compatible.
    if (( CUDA_MAJOR < 12 && CUDA_MINOR < 5 )); then
        if grep -q "tegra234" /proc/device-tree/compatible 2>/dev/null; then
            SM_ARCH="sm_87"   # Orin
        else
            SM_ARCH="sm_72"   # Xavier NX / AGX Xavier
        fi
        CUDA_KERNELS_CMAKE="${CUVSLAM_SRC_DIR}/libs/cuda_modules/cuda_kernels/CMakeLists.txt"
        if grep -q '\-arch=all' "${CUDA_KERNELS_CMAKE}"; then
            echo "[aarch64] Patching -arch=all -> -arch=${SM_ARCH} for CUDA ${CUDA_MAJOR}.${CUDA_MINOR}"
            sed -i "s|-arch=all|-arch=${SM_ARCH}|g" "${CUDA_KERNELS_CMAKE}"
        fi
    fi

    # 2) glibc 2.34+ merged librt into libc, leaving an empty 8-byte librt.a
    #    stub that nvlink 11.4 cannot open. Replace with a valid dummy archive.
    LIBRT_A="/usr/lib/aarch64-linux-gnu/librt.a"
    if [[ -f "${LIBRT_A}" ]]; then
        LIBRT_SIZE=$(stat -c%s "${LIBRT_A}")
        if (( LIBRT_SIZE < 64 )); then
            echo "[aarch64] Replacing empty librt.a (${LIBRT_SIZE} bytes) with a stub archive"
            TMP_STUB=$(mktemp -d)
            echo 'void __cuvslam_librt_stub(void) {}' > "${TMP_STUB}/stub.c"
            gcc -c "${TMP_STUB}/stub.c" -o "${TMP_STUB}/stub.o"
            ar rcs "${LIBRT_A}" "${TMP_STUB}/stub.o"
            rm -rf "${TMP_STUB}"
        fi
    fi

    # 3) liblmdb-dev is required by cuVSLAM but not always installed.
    if [[ ! -f /usr/include/lmdb.h ]]; then
        echo "[aarch64] Installing liblmdb-dev"
        apt-get update -qq && apt-get install -y liblmdb-dev
    fi
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
