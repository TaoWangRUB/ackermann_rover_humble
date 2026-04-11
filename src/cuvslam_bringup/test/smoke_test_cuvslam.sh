# cuvslam_bringup/test/smoke_test_cuvslam.sh
# Builds and runs the cuVSLAM smoke test (x86_64 only)
set -e

cd /workspace/src/cuvslam_bringup
mkdir -p build && cd build
cmake ..
make -j$(nproc)
./smoke_test_cuvslam
