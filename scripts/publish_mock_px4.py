#!/usr/bin/env python3
"""
Mock PX4 topic publisher for testing px4_probe on x86_64 development machines.
Publishes synthetic PX4 autopilot topics for offline testing without hardware.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import signal


class MockPx4Publisher(Node):
    """Publishes mock PX4 vehicle status topics at realistic rates."""

    def __init__(self):
        super().__init__("mock_px4_publisher")

        # Try to import PX4 messages, fall back gracefully if not available
        try:
            from px4_msgs.msg import VehicleStatus, BatteryStatus, VehicleOdometry
            self.have_px4_msgs = True
        except ImportError:
            self.get_logger().warn(
                "px4_msgs not available - install via: colcon build --packages-select px4_msgs"
            )
            self.have_px4_msgs = False
            return

        # Publishers
        self.vehicle_status_pub = self.create_publisher(
            VehicleStatus, "/fmu/out/vehicle_status_v2", 10
        )
        self.battery_status_pub = self.create_publisher(
            BatteryStatus, "/fmu/out/battery_status", 10
        )
        self.vehicle_odom_pub = self.create_publisher(
            VehicleOdometry, "/fmu/out/vehicle_odometry", 10
        )

        # Timers (typical PX4 rates: 50 Hz vehicle_status, 5 Hz battery, 30 Hz odometry)
        self.create_timer(1.0 / 10.0, self.publish_vehicle_status)
        self.create_timer(1.0 / 5.0, self.publish_battery_status)
        self.create_timer(1.0 / 10.0, self.publish_vehicle_odometry)

        self.status_id = 0
        self.get_logger().info("Mock PX4 publisher started (10 Hz status, 5 Hz battery, 10 Hz odometry)")

    def publish_vehicle_status(self):
        """Publish synthetic vehicle status (armed, disarmed, nav_state, etc.)."""
        if not self.have_px4_msgs:
            return

        from px4_msgs.msg import VehicleStatus

        msg = VehicleStatus()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # microseconds

        # Simulate armed state (toggle every 30 seconds)
        self.status_id += 1
        armed_duration = 30 * 10  # 30 seconds at 10 Hz
        is_armed = (self.status_id // armed_duration) % 2 == 1
        msg.arming_state = (
            VehicleStatus.ARMING_STATE_ARMED
            if is_armed
            else VehicleStatus.ARMING_STATE_DISARMED
        )

        # Navigation state: OFFBOARD while armed, MANUAL while disarmed
        nav = (
            VehicleStatus.NAVIGATION_STATE_OFFBOARD
            if is_armed
            else VehicleStatus.NAVIGATION_STATE_MANUAL
        )
        msg.nav_state = nav
        msg.nav_state_display = nav
        msg.nav_state_timestamp = msg.timestamp

        # Healthy state indicators
        msg.failsafe = False
        msg.failure_detector_status = 0

        self.vehicle_status_pub.publish(msg)

    def publish_battery_status(self):
        """Publish synthetic battery status."""
        if not self.have_px4_msgs:
            return

        from px4_msgs.msg import BatteryStatus

        msg = BatteryStatus()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # microseconds

        # Simulate battery drain over time (start at 100%, decrease by ~0.5% per second)
        elapsed_sec = self.get_clock().now().nanoseconds / 1e9
        remaining = max(0.10, 1.0 - (elapsed_sec * 0.005))  # Never drop below 10%
        msg.remaining = remaining
        msg.connected = True
        msg.voltage_v = 11.1 + remaining * 0.9  # 11.1V at 0%, 12V at 100%
        msg.current_a = 10.0 if self.status_id % 50 > 25 else 5.0  # Varying current
        msg.state_of_health = 100  # Perfect health

        self.battery_status_pub.publish(msg)

    def publish_vehicle_odometry(self):
        """Publish synthetic vehicle odometry (position, velocity)."""
        if not self.have_px4_msgs:
            return

        from px4_msgs.msg import VehicleOdometry

        msg = VehicleOdometry()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # microseconds
        msg.timestamp_sample = msg.timestamp

        # Simulate movement in a circle (odometry frame)
        elapsed_sec = self.get_clock().now().nanoseconds / 1e9
        angle = elapsed_sec * 0.5  # Rotate 0.5 rad/sec

        # Position (in odometry frame): circular path with 5m radius
        msg.position[0] = 5.0 * np.cos(angle)  # x
        msg.position[1] = 5.0 * np.sin(angle)  # y
        msg.position[2] = -0.5  # z (altitude: -0.5m AGL)

        # Velocity: tangent to circle
        speed = 2.5  # m/s
        msg.velocity[0] = -speed * np.sin(angle)  # vx
        msg.velocity[1] = speed * np.cos(angle)  # vy
        msg.velocity[2] = 0.0  # vz

        # Attitude (quaternion): level flight
        msg.q[0] = 1.0  # w
        msg.q[1] = 0.0  # x
        msg.q[2] = 0.0  # y
        msg.q[3] = 0.0  # z

        self.vehicle_odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockPx4Publisher()
    shutdown_requested = False

    def _request_shutdown(signum, frame):  # noqa: ARG001
        nonlocal shutdown_requested
        shutdown_requested = True
        if node.have_px4_msgs:
            node.get_logger().info("Shutdown requested, stopping mock PX4 publisher...")

    if not node.have_px4_msgs:
        node.get_logger().error(
            "Cannot start mock PX4 publisher without px4_msgs. "
            "Install with: colcon build --packages-select px4_msgs"
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)

    try:
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
