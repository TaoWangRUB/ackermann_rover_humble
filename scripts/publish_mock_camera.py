#!/usr/bin/env python3
"""
Mock camera topic publisher for testing cam_probe on x86_64 development machines.
Publishes synthetic RealSense camera topics for offline testing without hardware.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from builtin_interfaces.msg import Time
import cv2
import numpy as np
from datetime import datetime


class MockCameraPublisher(Node):
    """Publishes mock RealSense D435i camera topics at realistic rates."""

    def __init__(self):
        super().__init__("mock_camera_publisher")

        # Publishers
        self.color_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self.depth_pub = self.create_publisher(Image, "/camera/depth/image_rect_raw", 10)
        self.imu_pub = self.create_publisher(Imu, "/camera/imu", 10)

        # Timers (D435i spec: 30 Hz color, 30 Hz depth, 200 Hz IMU)
        self.create_timer(1.0 / 30.0, self.publish_color)
        self.create_timer(1.0 / 30.0, self.publish_depth)
        self.create_timer(1.0 / 200.0, self.publish_imu)

        self.frame_id = 0
        self.imu_id = 0
        self.get_logger().info("Mock camera publisher started (30 Hz color/depth, 200 Hz IMU)")

    def publish_color(self):
        """Publish synthetic RGB image."""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_color_optical_frame"
        msg.height = 480
        msg.width = 640
        msg.encoding = "rgb8"
        msg.step = 640 * 3

        # Create synthetic RGB image (gradient pattern)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame[:, :, 0] = np.linspace(0, 255, 640, dtype=np.uint8)  # Red gradient
        frame[:, :, 1] = np.linspace(0, 255, 480, dtype=np.uint8).reshape(-1, 1)  # Green gradient
        msg.data = frame.tobytes()

        self.color_pub.publish(msg)
        self.frame_id += 1

    def publish_depth(self):
        """Publish synthetic depth image."""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_depth_optical_frame"
        msg.height = 480
        msg.width = 640
        msg.encoding = "z16"
        msg.step = 640 * 2

        # Create synthetic depth image (distance ramp: 0.3-2.0 meters)
        depth_data = np.linspace(300, 2000, 640 * 480, dtype=np.uint16).reshape(480, 640)
        msg.data = depth_data.tobytes()

        self.depth_pub.publish(msg)

    def publish_imu(self):
        """Publish synthetic IMU data."""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_imu_optical_frame"

        # Synthetic IMU data (stationary, gravity only)
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81

        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0

        # Covariance (typical values)
        msg.linear_acceleration_covariance[0] = 0.01
        msg.angular_velocity_covariance[0] = 0.01

        self.imu_pub.publish(msg)
        self.imu_id += 1


def main(args=None):
    rclpy.init(args=args)
    node = MockCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
