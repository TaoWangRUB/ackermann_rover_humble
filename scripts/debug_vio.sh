#!/usr/bin/env bash
# Debug RTAB-Map / VIO pipeline alignment issues for D435i, L515, T265, VINS, and cuVSLAM.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HZ_WINDOW="${HZ_WINDOW:-5}"
ECHO_TIMEOUT="${ECHO_TIMEOUT:-5}"
KEEP_TF_ARTIFACTS="${KEEP_TF_ARTIFACTS:-0}"

cleanup_tf_artifacts() {
    if [[ "${KEEP_TF_ARTIFACTS}" != "1" ]]; then
        rm -f /tmp/frames.pdf /tmp/frames.gv /tmp/frames.yaml /tmp/frames.xml
    fi
}

show_help() {
    cat <<'EOF'
Usage: ./scripts/debug_vio.sh

Runs the VIO / RTAB-Map debug probe. When launched from the host, it will
automatically execute inside the `ackermann_slam` container, similar to
`start_ros2_nodes.sh`.

Optional environment variables:
  HZ_WINDOW=5           Seconds for each `ros2 topic hz` probe
  ECHO_TIMEOUT=5        Seconds to wait for one message sample
  KEEP_TF_ARTIFACTS=1   Keep /tmp/frames.* files instead of cleaning them up
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    show_help
    exit 0
fi

if [[ "${SCRIPT_DIR}" != "/workspace/scripts" ]]; then
    # Host-side entrypoint: mirror start_ros2_nodes.sh and run inside the dev container.
    # shellcheck source=/dev/null
    source "${SCRIPT_DIR}/lib/dc.sh"
    INNER_CMD="source /opt/ros/\$ROS_DISTRO/setup.bash && "
    INNER_CMD+="source /workspace/install/setup.bash && "
    INNER_CMD+="export HZ_WINDOW=$(printf '%q' "${HZ_WINDOW}") && "
    INNER_CMD+="export ECHO_TIMEOUT=$(printf '%q' "${ECHO_TIMEOUT}") && "
    INNER_CMD+="export KEEP_TF_ARTIFACTS=$(printf '%q' "${KEEP_TF_ARTIFACTS}") && "
    INNER_CMD+="bash /workspace/scripts/debug_vio.sh"
    xdcomp exec -T ackermann_slam bash -lc "${INNER_CMD}"
fi

trap cleanup_tf_artifacts EXIT

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 is not on PATH. Source the ROS and workspace setup files first."
    exit 1
fi

declare -A LABELS=(
    [d435i_color]="D435i color"
    [d435i_depth]="D435i aligned depth"
    [d435i_color_info]="D435i color camera_info"
    [d435i_depth_info]="D435i depth camera_info"
    [d435i_imu]="D435i IMU"
    [l515_color]="L515 color"
    [l515_depth]="L515 aligned depth"
    [l515_color_info]="L515 color camera_info"
    [l515_depth_info]="L515 depth camera_info"
    [l515_imu]="L515 IMU"
    [t265_fisheye1]="T265 fisheye1"
    [t265_fisheye2]="T265 fisheye2"
    [t265_fisheye1_info]="T265 fisheye1 camera_info"
    [t265_fisheye2_info]="T265 fisheye2 camera_info"
    [t265_odom]="T265 odom"
    [t265_odom_base]="T265 odom base"
    [vins_raw_odom]="VINS raw odom"
    [vins_odom]="VINS adapted odom"
    [cuvslam_raw_odom]="cuVSLAM raw odom"
    [cuvslam_odom]="cuVSLAM adapted odom"
    [imu_raw_transformed]="IMU transformed"
    [imu_data]="IMU filtered"
    [rgbd_image]="RGBD image"
    [vo_odom]="VO odom"
    [ekf_odom]="EKF odom"
    [map]="Map"
    [rtabmap_info]="RTAB-Map info"
    [d435i_extrinsics]="D435i depth->color extrinsics"
    [l515_extrinsics]="L515 depth->color extrinsics"
)

TOPIC_KEYS=(
    d435i_color d435i_depth d435i_color_info d435i_depth_info d435i_imu
    l515_color l515_depth l515_color_info l515_depth_info l515_imu
    t265_fisheye1 t265_fisheye2 t265_fisheye1_info t265_fisheye2_info
    t265_odom t265_odom_base vins_raw_odom vins_odom cuvslam_raw_odom cuvslam_odom imu_raw_transformed imu_data
    rgbd_image vo_odom ekf_odom map rtabmap_info
    d435i_extrinsics l515_extrinsics
)

STAMP_KEYS=(
    d435i_color d435i_depth d435i_imu
    l515_color l515_depth l515_imu
    t265_fisheye1 t265_fisheye2 t265_odom t265_odom_base vins_raw_odom vins_odom cuvslam_raw_odom cuvslam_odom
    imu_raw_transformed imu_data vo_odom ekf_odom rgbd_image
)

STAT_KEYS=(
    d435i_color d435i_depth d435i_imu
    l515_color l515_depth l515_imu
    t265_fisheye1 t265_fisheye2 t265_odom t265_odom_base vins_raw_odom vins_odom cuvslam_raw_odom cuvslam_odom
    imu_raw_transformed imu_data
    rgbd_image vo_odom ekf_odom map
)

declare -A RESOLVED_TOPICS
declare -A TOPIC_TYPES
declare -A SUMMARY

TOPIC_LIST="$(ros2 topic list 2>/dev/null || true)"
NODE_LIST="$(ros2 node list 2>/dev/null || true)"

topic_exists() {
    local topic="$1"
    [[ -n "${topic}" ]] && grep -Fxq "${topic}" <<<"${TOPIC_LIST}"
}

resolve_topic() {
    local key="$1"
    shift
    local candidate
    for candidate in "$@"; do
        if topic_exists "${candidate}"; then
            RESOLVED_TOPICS["${key}"]="${candidate}"
            return 0
        fi
    done
    RESOLVED_TOPICS["${key}"]=""
    return 1
}

resolve_topic d435i_color /d435i/color/image_raw /d435i/image
resolve_topic d435i_depth /d435i/aligned_depth_to_color/image_raw /d435i/depth_image /d435i/depth/image_rect_raw
resolve_topic d435i_color_info /d435i/color/camera_info /d435i/camera_info
resolve_topic d435i_depth_info /d435i/aligned_depth_to_color/camera_info /d435i/camera_info
resolve_topic d435i_imu /d435i/imu /d435i/imu/raw
resolve_topic l515_color /l515/color/image_raw /l515/image
resolve_topic l515_depth /l515/aligned_depth_to_color/image_raw /l515/depth_image /l515/depth/image_rect_raw
resolve_topic l515_color_info /l515/color/camera_info /l515/camera_info
resolve_topic l515_depth_info /l515/aligned_depth_to_color/camera_info /l515/camera_info
resolve_topic l515_imu /l515/imu /l515/imu/raw
resolve_topic t265_fisheye1 /t265/fisheye1/image_raw
resolve_topic t265_fisheye2 /t265/fisheye2/image_raw
resolve_topic t265_fisheye1_info /t265/fisheye1/camera_info
resolve_topic t265_fisheye2_info /t265/fisheye2/camera_info
resolve_topic t265_odom /t265/odom /t265/odom/sample
resolve_topic t265_odom_base /t265/odom_base /t265_odom_base
resolve_topic vins_raw_odom /vins/raw_odometry /vins/odometry
resolve_topic vins_odom /vins_odom /vins/odom_base
resolve_topic cuvslam_raw_odom /cuvslam/raw_odometry
resolve_topic cuvslam_odom /cuvslam_odom
resolve_topic imu_raw_transformed /imu/raw_transformed
resolve_topic imu_data /imu/data
resolve_topic rgbd_image /rgbd_image
resolve_topic vo_odom /vo_odom
resolve_topic ekf_odom /odometry/filtered
resolve_topic map /map
resolve_topic rtabmap_info /rtabmap/info
resolve_topic d435i_extrinsics /d435i/extrinsics/depth_to_color /camera/extrinsics/depth_to_color
resolve_topic l515_extrinsics /l515/extrinsics/depth_to_color

for key in "${TOPIC_KEYS[@]}"; do
    if [[ -n "${RESOLVED_TOPICS[$key]}" ]]; then
        TOPIC_TYPES["${key}"]="$(ros2 topic type "${RESOLVED_TOPICS[$key]}" 2>/dev/null || true)"
    else
        TOPIC_TYPES["${key}"]=""
    fi
done

RTABMAP_NODE=""
if grep -Fxq "/rtabmap" <<<"${NODE_LIST}"; then
    RTABMAP_NODE="/rtabmap"
elif grep -Fxq "/rtabmap/rtabmap" <<<"${NODE_LIST}"; then
    RTABMAP_NODE="/rtabmap/rtabmap"
fi

section() {
    printf '\n=== %s ===\n' "$1"
}

print_topic_map() {
    local key topic
    for key in "${TOPIC_KEYS[@]}"; do
        topic="${RESOLVED_TOPICS[$key]}"
        if [[ -n "${topic}" ]]; then
            printf '%-28s %s\n' "${LABELS[$key]}:" "${topic}"
        else
            printf '%-28s %s\n' "${LABELS[$key]}:" "(not published)"
        fi
    done
}

print_topic_info() {
    local key="$1"
    local topic="${RESOLVED_TOPICS[$key]}"
    echo "${LABELS[$key]}:"
    if [[ -z "${topic}" ]]; then
        echo "  not published"
        return
    fi
    ros2 topic info "${topic}" 2>/dev/null | sed 's/^/  /' | head -5 || echo "  topic info unavailable"
}

print_stamp_sample() {
    local key="$1"
    local topic="${RESOLVED_TOPICS[$key]}"
    local topic_type="${TOPIC_TYPES[$key]}"
    echo "${LABELS[$key]}:"
    if [[ -z "${topic}" ]]; then
        echo "  not published"
        return
    fi
    local sample
    sample="$(
        python3 - "${topic}" "${topic_type}" "${ECHO_TIMEOUT}" <<'PY'
import importlib
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

topic = sys.argv[1]
type_name = sys.argv[2]
timeout = float(sys.argv[3])

pkg, _, iface = type_name.partition('/msg/')
if not pkg or not iface:
    sys.exit(0)

msg_module = importlib.import_module(f'{pkg}.msg')
msg_type = getattr(msg_module, iface)

qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

rclpy.init()


class Probe(Node):
    def __init__(self) -> None:
        super().__init__('debug_vio_stamp_probe')
        self.msg = None
        self.create_subscription(msg_type, topic, self._cb, qos)

    def _cb(self, msg) -> None:
        if self.msg is None:
            self.msg = msg


node = Probe()
deadline = time.monotonic() + timeout
while rclpy.ok() and time.monotonic() < deadline and node.msg is None:
    rclpy.spin_once(node, timeout_sec=0.2)

msg = node.msg
node.destroy_node()
rclpy.shutdown()

if msg is None:
    sys.exit(0)

lines = []
header = getattr(msg, 'header', None)
if header is not None:
    lines.append('stamp:')
    lines.append(f'  sec: {header.stamp.sec}')
    lines.append(f'  nanosec: {header.stamp.nanosec}')
    lines.append(f'frame_id: {header.frame_id}')

child_frame_id = getattr(msg, 'child_frame_id', None)
if child_frame_id is not None:
    lines.append(f'child_frame_id: {child_frame_id}')

if lines:
    print('\n'.join(lines))
PY
    )"
    if [[ -n "${sample}" ]]; then
        sed 's/^/  /' <<<"${sample}"
    else
        echo "  no sample received within ${ECHO_TIMEOUT}s"
    fi
}

format_hz_stats() {
    local topic="$1"
    local topic_type="$2"
    python3 - "${topic}" "${topic_type}" "${HZ_WINDOW}" <<'PY'
import importlib
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

topic = sys.argv[1]
type_name = sys.argv[2]
duration = float(sys.argv[3])

pkg, _, iface = type_name.partition('/msg/')
if not pkg or not iface:
    print(f'no samples in {duration:g}s')
    sys.exit(0)

msg_module = importlib.import_module(f'{pkg}.msg')
msg_type = getattr(msg_module, iface)

qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

rclpy.init()


class Probe(Node):
    def __init__(self) -> None:
        super().__init__('debug_vio_rate_probe')
        self.arrivals = []
        self.create_subscription(msg_type, topic, self._cb, qos)

    def _cb(self, _msg) -> None:
        self.arrivals.append(time.monotonic())


node = Probe()
deadline = time.monotonic() + duration
while rclpy.ok() and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)

arrivals = node.arrivals
node.destroy_node()
rclpy.shutdown()

if len(arrivals) < 2:
    print(f'no samples in {duration:g}s')
    sys.exit(0)

intervals = [b - a for a, b in zip(arrivals, arrivals[1:]) if b > a]
if not intervals:
    print(f'no samples in {duration:g}s')
    sys.exit(0)

avg_rate = len(intervals) / sum(intervals)
min_period = min(intervals)
max_period = max(intervals)
std_period = statistics.pstdev(intervals) if len(intervals) > 1 else 0.0

print(
    f'{avg_rate:.3f} Hz (min {min_period:.4f}, max {max_period:.4f}, '
    f'std {std_period:.4f}, window {len(arrivals)})'
)
PY
}

capture_stats() {
    local key="$1"
    local topic="${RESOLVED_TOPICS[$key]}"
    if [[ -z "${topic}" ]]; then
        SUMMARY["$key"]="not published"
        return
    fi
    SUMMARY["$key"]="$(format_hz_stats "${topic}" "${TOPIC_TYPES[$key]}")"
}

section "1. Frame Transform Tree"
if ros2 run tf2_tools view_frames.py >/dev/null 2>&1; then
    echo "TF tree saved to /tmp/frames.pdf"
else
    echo "Unable to generate TF tree right now"
fi

section "2. Topic Discovery"
print_topic_map

section "3. Topic Info"
for key in \
    d435i_color d435i_depth d435i_imu \
    l515_color l515_depth l515_imu \
    t265_odom t265_odom_base \
    imu_raw_transformed imu_data \
    vo_odom ekf_odom map rtabmap_info; do
    print_topic_info "${key}"
    echo
done

section "4. Message Timestamp Samples"
for key in "${STAMP_KEYS[@]}"; do
    print_stamp_sample "${key}"
    echo
done

section "5. Camera Extrinsics"
for key in d435i_extrinsics l515_extrinsics; do
    echo "${LABELS[$key]}:"
    if [[ -z "${RESOLVED_TOPICS[$key]}" ]]; then
        echo "  not published"
        continue
    fi
    timeout "${ECHO_TIMEOUT}s" ros2 topic echo "${RESOLVED_TOPICS[$key]}" --once 2>/dev/null \
        | sed 's/^/  /' || echo "  no sample received within ${ECHO_TIMEOUT}s"
done

section "6. RTAB-Map Status"
if [[ -n "${RTABMAP_NODE}" ]]; then
    echo "RTAB-Map node: ${RTABMAP_NODE}"
    ros2 param get "${RTABMAP_NODE}" Mem/BadSignaturesIgnored 2>/dev/null | sed 's/^/  /' || true
    ros2 param get "${RTABMAP_NODE}" Kp/DetectorStrategy 2>/dev/null | sed 's/^/  /' || true
    ros2 param get "${RTABMAP_NODE}" Vis/MaxFeatures 2>/dev/null | sed 's/^/  /' || true
else
    echo "RTAB-Map node not found"
fi

echo
if [[ -n "${RESOLVED_TOPICS[rtabmap_info]}" ]]; then
    timeout "${ECHO_TIMEOUT}s" ros2 topic echo "${RESOLVED_TOPICS[rtabmap_info]}" --once 2>/dev/null \
        | head -20 | sed 's/^/  /' || echo "  no /rtabmap/info sample received within ${ECHO_TIMEOUT}s"
else
    echo "/rtabmap/info not published"
fi

section "7. Statistics Summary"
echo "Measured over ${HZ_WINDOW}s per topic"
for key in "${STAT_KEYS[@]}"; do
    capture_stats "${key}"
    printf '%-28s %s' "${LABELS[$key]}:"
    if [[ -n "${RESOLVED_TOPICS[$key]}" ]]; then
        printf ' %s\n' "${SUMMARY[$key]}"
        printf '  topic: %s\n' "${RESOLVED_TOPICS[$key]}"
    else
        printf ' %s\n' "${SUMMARY[$key]}"
    fi
done
