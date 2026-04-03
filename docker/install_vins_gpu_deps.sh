#!/usr/bin/env bash
set -euo pipefail

OPENCV_VERSION="${OPENCV_VERSION:-4.6.0}"
OPENCV_PREFIX_VERSIONED="${OPENCV_PREFIX_VERSIONED:-/opt/opencv-cuda-${OPENCV_VERSION}}"
OPENCV_PREFIX_LINK="${OPENCV_PREFIX_LINK:-/opt/opencv-cuda}"
OPENCV_BUILD_ROOT="${OPENCV_BUILD_ROOT:-/tmp/opencv-cuda-build}"

detect_cuda_arch_bin() {
    if [[ -n "${CUDA_ARCH_BIN:-}" ]]; then
        printf '%s\n' "${CUDA_ARCH_BIN}"
        return
    fi

    if command -v nvidia-smi >/dev/null 2>&1; then
        local detected
        detected="$(
            nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null \
                | sed 's/ //g' \
                | sort -u \
                | paste -sd';' -
        )"
        if [[ -n "${detected}" ]]; then
            printf '%s\n' "${detected}"
            return
        fi
    fi

    # Covers Xavier, desktop Ampere/Ada, and Orin as a portable fallback.
    printf '%s\n' "7.2;8.6;8.7;8.9"
}

CUDA_ARCH_BIN="$(detect_cuda_arch_bin)"

if [[ ! -x /usr/local/cuda/bin/nvcc ]]; then
    echo "CUDA toolkit is not available at /usr/local/cuda. Start the container with the host CUDA mount first." >&2
    exit 1
fi

if [[ -f "${OPENCV_PREFIX_LINK}/lib/cmake/opencv4/OpenCVConfig.cmake" && "${FORCE_REBUILD:-0}" != "1" ]]; then
    echo "CUDA OpenCV already installed at ${OPENCV_PREFIX_LINK}"
else
    rm -rf "${OPENCV_BUILD_ROOT}"
    mkdir -p "${OPENCV_BUILD_ROOT}"

    git clone --depth 1 --branch "${OPENCV_VERSION}" https://github.com/opencv/opencv.git "${OPENCV_BUILD_ROOT}/opencv"
    git clone --depth 1 --branch "${OPENCV_VERSION}" https://github.com/opencv/opencv_contrib.git "${OPENCV_BUILD_ROOT}/opencv_contrib"

    mkdir -p "${OPENCV_BUILD_ROOT}/opencv/build"
    pushd "${OPENCV_BUILD_ROOT}/opencv/build" >/dev/null
    CC=gcc-11 CXX=g++-11 cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${OPENCV_PREFIX_VERSIONED}" \
        -DOPENCV_EXTRA_MODULES_PATH="${OPENCV_BUILD_ROOT}/opencv_contrib/modules" \
        -DBUILD_LIST=core,imgproc,features2d,calib3d,highgui,imgcodecs,cudaarithm,cudaimgproc,cudafeatures2d,cudaoptflow,cudev \
        -DWITH_CUDA=ON \
        -DCUDA_ARCH_BIN="${CUDA_ARCH_BIN}" \
        -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
        -DCMAKE_CUDA_HOST_COMPILER=/usr/bin/g++-11 \
        -DWITH_CUDNN=OFF \
        -DOPENCV_DNN_CUDA=OFF \
        -DENABLE_FAST_MATH=ON \
        -DCUDA_FAST_MATH=ON \
        -DWITH_TBB=ON \
        -DOPENCV_GENERATE_PKGCONFIG=ON \
        -DBUILD_TESTS=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_opencv_apps=OFF \
        -DBUILD_DOCS=OFF
    make -j"$(nproc)"
    sudo make install
    sudo ln -sfn "${OPENCV_PREFIX_VERSIONED}" "${OPENCV_PREFIX_LINK}"
    popd >/dev/null
fi

cat >/tmp/opencv_cuda_probe.cpp <<'EOF'
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/cudaoptflow.hpp>
#include <iostream>

int main() {
  const int device_count = cv::cuda::getCudaEnabledDeviceCount();
  auto flow = cv::cuda::SparsePyrLKOpticalFlow::create();
  auto detector = cv::cuda::createGoodFeaturesToTrackDetector(CV_8UC1, 200, 0.01, 10.0);
  std::cout << "OpenCV CUDA devices: " << device_count << std::endl;
  std::cout << "Created CUDA tracker and detector successfully." << std::endl;
  return device_count > 0 ? 0 : 2;
}
EOF

export PKG_CONFIG_PATH="${OPENCV_PREFIX_LINK}/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
g++ -std=c++17 /tmp/opencv_cuda_probe.cpp -o /tmp/opencv_cuda_probe $(pkg-config --cflags --libs opencv4)
LD_LIBRARY_PATH="${OPENCV_PREFIX_LINK}/lib:${LD_LIBRARY_PATH:-}" /tmp/opencv_cuda_probe

cat <<EOF
CUDA OpenCV is ready.
  prefix: ${OPENCV_PREFIX_LINK}
  versioned prefix: ${OPENCV_PREFIX_VERSIONED}
  CUDA_ARCH_BIN: ${CUDA_ARCH_BIN}

Build VINS in GPU mode with:
  ./scripts/build_vins_gpu.sh
EOF
