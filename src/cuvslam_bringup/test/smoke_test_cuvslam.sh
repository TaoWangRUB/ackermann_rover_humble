# cuvslam_bringup/test/smoke_test_cuvslam.sh
# Builds and runs the cuVSLAM smoke test (x86_64 only)
set -eo pipefail

source /opt/ros/jazzy/setup.bash
set -u

cd /workspace
colcon build --symlink-install --packages-select cuvslam_bringup --event-handlers console_direct+

export CUVSLAM_DST_DIR="${CUVSLAM_DST_DIR:-/workspace/src/cuVSLAM/build}"
export LD_LIBRARY_PATH="${CUVSLAM_DST_DIR}/bin:${LD_LIBRARY_PATH:-}"

./install/cuvslam_bringup/lib/cuvslam_bringup/smoke_test_cuvslam
