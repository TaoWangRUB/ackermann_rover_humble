#!/usr/bin/env python3
"""Extract /tf_static from a rosbag2 mcap and republish it as a latched topic.

When ros2 bag play uses --start-offset >0, it skips messages before the
offset including the bag's tf_static (typically published at t=0). This
helper extracts the first tf_static message from the bag and publishes
it on /tf_static with transient_local + reliable QoS so any RTAB-Map / TF
listener gets the static transforms regardless of start offset.

Usage:
  python3 publish_bag_tf_static.py /workspace/bags/run_xxx
"""
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import serialize_message
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time as RosTime


def main():
    if len(sys.argv) < 2:
        print("usage: publish_bag_tf_static.py <bag_dir>")
        sys.exit(2)
    bag_dir = Path(sys.argv[1])
    mcap_path = next(bag_dir.glob("*.mcap"), None)
    if mcap_path is None:
        print(f"no .mcap in {bag_dir}")
        sys.exit(2)

    # Read first tf_static message
    transforms_dump = []
    with mcap_path.open("rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for *_, m, msg in reader.iter_decoded_messages(topics=["/tf_static"]):
            for t in msg.transforms:
                ts = TransformStamped()
                ts.header.frame_id = t.header.frame_id
                ts.header.stamp.sec = int(t.header.stamp.sec)
                ts.header.stamp.nanosec = int(t.header.stamp.nanosec)
                ts.child_frame_id = t.child_frame_id
                ts.transform.translation.x = float(t.transform.translation.x)
                ts.transform.translation.y = float(t.transform.translation.y)
                ts.transform.translation.z = float(t.transform.translation.z)
                ts.transform.rotation.x = float(t.transform.rotation.x)
                ts.transform.rotation.y = float(t.transform.rotation.y)
                ts.transform.rotation.z = float(t.transform.rotation.z)
                ts.transform.rotation.w = float(t.transform.rotation.w)
                transforms_dump.append(ts)
            break

    if not transforms_dump:
        print("no /tf_static messages in bag")
        sys.exit(2)
    print(f"loaded {len(transforms_dump)} static transforms from bag")

    rclpy.init()
    node = Node("bag_tf_static_publisher")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(TFMessage, "/tf_static", qos)

    msg = TFMessage()
    msg.transforms = transforms_dump
    pub.publish(msg)
    node.get_logger().info(f"/tf_static published with {len(transforms_dump)} transforms (transient_local). "
                           "Late subscribers will receive it on subscribe.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
