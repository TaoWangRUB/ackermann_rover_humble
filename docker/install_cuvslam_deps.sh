#!/usr/bin/env bash
# docker/install_cuvslam_deps.sh
# Resolves dependencies and sets up the build environment for cuVSLAM.
# Implements architecture-aware caching to isolate x86_64 and Jetson builds.

set -euo pipefail

ARCH="$(uname -m)"
if [[ $ARCH == "x86_64" || $ARCH == "aarch64" ]]; then
    apt-get update && apt-get install -y gcc-11 g++-11 liblmdb-dev
    export CC=/usr/bin/gcc-11
    export CXX=/usr/bin/g++-11
fi

# --- CUDA TOOLKIT DETECTION ---
if command -v nvcc &>/dev/null; then
  CUDA_VER=$(nvcc --version | grep release | sed 's/.*release \([0-9]*\.[0-9]*\).*/\1/')
else
  CUDA_VER="unknown"
fi

# --- SHARED CUDA RUNTIME LINKAGE ---
export CU_VSLAM_CUDA_LINKAGE="shared"

# --- JETSON GLIBC COMPATIBILITY FLAGS ---
if [[ $ARCH == "aarch64" ]]; then
    export CFLAGS="${CFLAGS:-} -D_FORCE_INLINES -D__GLIBC_USE_LIB_EXT2=1"
    export LDFLAGS="${LDFLAGS:-} -Wl,--unresolved-symbols=ignore-in-object-files"
fi

# --- ARCHITECTURE-AWARE CACHE PATHS ---
if [[ $ARCH == "x86_64" ]]; then
    export CUVSLAM_CACHE_DIR="/docker_cache/cuvslam/x86_64-cuda${CUDA_VER}"
elif [[ $ARCH == "aarch64" ]]; then
    export CUVSLAM_CACHE_DIR="/docker_cache/cuvslam/jetson-cuda${CUDA_VER}"
fi

echo "[install_cuvslam_deps.sh] ARCH=$ARCH, CUDA_VER=$CUDA_VER, CC=$CC, CUVSLAM_CACHE_DIR=$CUVSLAM_CACHE_DIR"

echo "========================================================"
echo " Setting up cuVSLAM dependencies for architecture: ${ARCH}"
echo "========================================================"

mkdir -p "${CUVSLAM_CACHE_DIR}"

if [[ "${ARCH}" == "x86_64" ]]; then
    # Set up x86_64-specific cache paths
    CUVSLAM_BUILD_DIR="${CUVSLAM_SRC}/build"
    CUVSLAM_INSTALL_DIR="${CUVSLAM_CACHE_DIR}/install"
    mkdir -p "${CUVSLAM_BUILD_DIR}" "${CUVSLAM_INSTALL_DIR}"

    echo "[x86_64] Initializing host CUDA integration path..."
    
    if [[ ! -x "/usr/local/cuda/bin/nvcc" ]]; then
        echo "WARNING: /usr/local/cuda/bin/nvcc not found."
        echo "Ensure the host CUDA toolkit is mounted into the container via docker-compose."
    else
        echo "CUDA Toolkit found:"
        /usr/local/cuda/bin/nvcc --version
    fi
    
    # Note: OpenCV with CUDA for x86_64 is handled by existing VINS-Fusion dependency scripts
    # or can be fetched here if cuVSLAM requires a strictly different version.
    
    echo "[x86_64] Building cuVSLAM from source..."
    cd "${CUVSLAM_BUILD_DIR}"
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${CUVSLAM_INSTALL_DIR}"
    make -j$(nproc)
    make install
    echo "[x86_64] cuVSLAM build complete. Installed to ${CUVSLAM_INSTALL_DIR}"
    
elif [[ "${ARCH}" == "aarch64" ]]; then
    CUVSLAM_SRC="${CUVSLAM_SRC:-/workspace/src/cuVSLAM}"
    CUVSLAM_BUILD_DIR="${CUVSLAM_SRC}/build"
    CUVSLAM_INSTALL_DIR="${CUVSLAM_CACHE_DIR}/install"
    mkdir -p "${CUVSLAM_BUILD_DIR}" "${CUVSLAM_INSTALL_DIR}"

    echo "[aarch64] Initializing Jetson CUDA integration path..."

    if [[ ! -x "/usr/local/cuda/bin/nvcc" ]]; then
        echo "WARNING: /usr/local/cuda/bin/nvcc not found."
        echo "Ensure the JetPack CUDA toolkit is mounted into the container via docker-compose."
    else
        echo "CUDA Toolkit found:"
        /usr/local/cuda/bin/nvcc --version
    fi

    # The operator build script (scripts/build_cuvslam.sh) handles the three
    # Jetson-specific workarounds at build time:
    #   1) -arch=all -> -arch=sm_72 (or sm_87) for CUDA < 11.5
    #   2) Empty librt.a stub replacement for glibc 2.34+ / nvlink 11.4
    #   3) liblmdb-dev installation if missing
    # This dependency script only ensures the base toolchain is present.

    echo "[aarch64] Jetson dependency setup complete."
    echo "Run ./scripts/build_cuvslam.sh to build cuVSLAM from source."
else
    echo "Unsupported architecture: ${ARCH}"
    exit 1
fi

echo "cuVSLAM dependency path scaffolded. Cache ready at: ${CUVSLAM_CACHE_DIR}"