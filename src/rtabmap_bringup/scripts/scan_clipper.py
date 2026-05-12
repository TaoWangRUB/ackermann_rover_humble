#!/usr/bin/env python3
"""Replace inf/nan in /scan with range_max so RTAB-Map's Grid/RayTracing
clears the full camera FOV up to range_max in open environments instead of
leaving unknown cells where rays returned no obstacle.

Ported from upstream gazebo_ackmann_rc_sim with the missing-writeback bug
fixed (the upstream version processed `ranges` locally but never assigned it
back to msg before publishing).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanClipper(Node):
    def __init__(self):
        super().__init__('scan_clipper')

        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('min_range', 0.12)
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')

        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        self.pub = self.create_publisher(LaserScan, out_topic, 10)
        self.sub = self.create_subscription(LaserScan, in_topic, self.cb, 10)

        self.get_logger().info(
            f'scan_clipper: {in_topic} -> {out_topic}  range=[{self.min_range}, {self.max_range}]'
        )

    def cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        bad = np.isinf(ranges) | np.isnan(ranges)
        ranges[bad] = self.max_range
        ranges = np.clip(ranges, self.min_range, self.max_range)

        msg.ranges = ranges.tolist()
        msg.range_min = self.min_range
        msg.range_max = self.max_range
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanClipper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
