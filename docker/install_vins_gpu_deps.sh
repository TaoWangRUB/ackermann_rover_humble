#!/usr/bin/env bash
set -euo pipefail

OPENCV_VERSION="${OPENCV_VERSION:-4.10.0}"
OPENCV_CACHE_ROOT="${OPENCV_CACHE_ROOT:-/workspace/docker_cache/opencv-cuda}"
OPENCV_PREFIX_VERSIONED="${OPENCV_PREFIX_VERSIONED:-${OPENCV_CACHE_ROOT}/${OPENCV_VERSION}}"
OPENCV_PREFIX_LINK="${OPENCV_PREFIX_LINK:-${OPENCV_CACHE_ROOT}/current}"
OPENCV_BUILD_ROOT="${OPENCV_BUILD_ROOT:-/tmp/opencv-cuda-build}"

repo_matches_version() {
    local repo_dir="$1"
    local version="$2"
    local marker_file="${repo_dir}/.source-version"

    if [[ -f "${marker_file}" ]]; then
        [[ "$(<"${marker_file}")" == "${version}" ]]
        return
    fi

    [[ -d "${repo_dir}/.git" ]] || return 1
    [[ "$(git -C "${repo_dir}" describe --tags --exact-match 2>/dev/null || true)" == "${version}" ]]
}

download_tarball_retry() {
    local tarball_url="$1"
    local version="$2"
    local dest="$3"
    local attempts="${4:-3}"
    local attempt

    for attempt in $(seq 1 "${attempts}"); do
        local archive
        local extract_root
        local temp_root

        archive="$(mktemp "/tmp/$(basename "${dest}")-${version}-XXXXXX.tar.gz")"
        temp_root="$(mktemp -d "/tmp/$(basename "${dest}")-${version}-XXXXXX")"

        rm -rf "${dest}"
        mkdir -p "${dest}"

        if curl --http1.1 -L --fail --retry 3 --retry-all-errors --connect-timeout 30 "${tarball_url}" -o "${archive}" \
            && tar -xzf "${archive}" -C "${temp_root}" \
            && extract_root="$(find "${temp_root}" -mindepth 1 -maxdepth 1 -type d | head -n 1)" \
            && [[ -n "${extract_root}" ]]; then
            rm -rf "${dest}"
            mv "${extract_root}" "${dest}"
            printf '%s\n' "${version}" > "${dest}/.source-version"
            rm -rf "${temp_root}" "${archive}"
            return 0
        fi

        rm -rf "${dest}" "${temp_root}" "${archive}"
        echo "tarball download failed for ${tarball_url} (attempt ${attempt}/${attempts})" >&2
        if [[ "${attempt}" -lt "${attempts}" ]]; then
            sleep 5
        fi
    done

    echo "Failed to fetch ${tarball_url} after ${attempts} attempts." >&2
    return 1
}

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
    mkdir -p "${OPENCV_BUILD_ROOT}"
    mkdir -p "$(dirname "${OPENCV_PREFIX_VERSIONED}")" "$(dirname "${OPENCV_PREFIX_LINK}")"

    if ! repo_matches_version "${OPENCV_BUILD_ROOT}/opencv" "${OPENCV_VERSION}"; then
        download_tarball_retry \
            "https://github.com/opencv/opencv/archive/refs/tags/${OPENCV_VERSION}.tar.gz" \
            "${OPENCV_VERSION}" \
            "${OPENCV_BUILD_ROOT}/opencv"
    else
        echo "Reusing existing OpenCV checkout at ${OPENCV_BUILD_ROOT}/opencv"
    fi

    if ! repo_matches_version "${OPENCV_BUILD_ROOT}/opencv_contrib" "${OPENCV_VERSION}"; then
        download_tarball_retry \
            "https://github.com/opencv/opencv_contrib/archive/refs/tags/${OPENCV_VERSION}.tar.gz" \
            "${OPENCV_VERSION}" \
            "${OPENCV_BUILD_ROOT}/opencv_contrib"
    else
        echo "Reusing existing OpenCV contrib checkout at ${OPENCV_BUILD_ROOT}/opencv_contrib"
    fi

    rm -rf "${OPENCV_BUILD_ROOT}/opencv/build"
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
        -DWITH_IPP=OFF \
        -DWITH_VTK=OFF \
        -DWITH_OPENJPEG=OFF \
        -DWITH_CUDNN=OFF \
        -DOPENCV_DNN_CUDA=OFF \
        -DENABLE_FAST_MATH=ON \
        -DCUDA_FAST_MATH=ON \
        -DWITH_TBB=OFF \
        -DOPENCV_GENERATE_PKGCONFIG=ON \
        -DBUILD_opencv_gapi=OFF \
        -DBUILD_opencv_python2=OFF \
        -DBUILD_opencv_python3=OFF \
        -DBUILD_JAVA=OFF \
        -DBUILD_TESTS=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_opencv_apps=OFF \
        -DBUILD_DOCS=OFF
    make -j"$(nproc)"
    if [[ -w "$(dirname "${OPENCV_PREFIX_VERSIONED}")" ]]; then
        make install
        ln -sfn "${OPENCV_PREFIX_VERSIONED}" "${OPENCV_PREFIX_LINK}"
    else
        sudo make install
        sudo ln -sfn "${OPENCV_PREFIX_VERSIONED}" "${OPENCV_PREFIX_LINK}"
    fi
    popd >/dev/null
fi

cat >/tmp/opencv_cuda_probe.cpp <<'EOF'
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
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
