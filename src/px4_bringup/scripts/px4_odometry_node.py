#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry

class Px4OdometryNode(Node):
    def __init__(self):
        super().__init__('px4_odometry_bridge')

        # Subscriptions
        # Taking standard nav_msgs/Odometry
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publishers
        self.vehicle_odom_pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_odometry', 10)
        
        self.get_logger().info("PX4 Odometry Bridge Node started. Waiting for /odom...")

    def odom_callback(self, msg: Odometry):
        px4_odom = VehicleOdometry()
        
        # Timestamp in microseconds
        px4_odom.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        px4_odom.timestamp_sample = px4_odom.timestamp

        # Frames: 1 = POSE_FRAME_NED, 2 = VELOCITY_FRAME_FRD, 3 = VELOCITY_FRAME_BODY_FRD
        px4_odom.pose_frame = VehicleOdometry.POSE_FRAME_NED
        px4_odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED

        # Position Conversion: ENU -> NED
        # x_ned = y_enu
        # y_ned = x_enu
        # z_ned = -z_enu
        px4_odom.position = [
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.x),
            float(-msg.pose.pose.position.z)
        ]

        # Orientation Conversion: ENU Quaternion -> NED Quaternion
        # We need to swap the axes and invert Z to match NED.
        # ENU (x, y, z, w) -> NED (y, x, -z, w)
        q_enu = msg.pose.pose.orientation
        px4_odom.q = [
            float(q_enu.w),   # PX4 scalar first
            float(q_enu.y),   # vector x
            float(q_enu.x),   # vector y
            float(-q_enu.z)   # vector z
        ]

        # Velocity Conversion: ENU -> NED
        px4_odom.velocity = [
            float(msg.twist.twist.linear.y),
            float(msg.twist.twist.linear.x),
            float(-msg.twist.twist.linear.z)
        ]

        # Angular Velocity Conversion: FLU body frame -> FRD body frame
        # roll -> roll (x -> x)
        # pitch -> -pitch (y -> -y)
        # yaw -> -yaw (z -> -z)
        px4_odom.angular_velocity = [
            float(msg.twist.twist.angular.x),
            float(-msg.twist.twist.angular.y),
            float(-msg.twist.twist.angular.z)
        ]

        # Variances (if provided by EKF). For simplicity, set to NaNs if unknown.
        px4_odom.position_variance = [float('nan'), float('nan'), float('nan')]
        px4_odom.orientation_variance = [float('nan'), float('nan'), float('nan')]
        px4_odom.velocity_variance = [float('nan'), float('nan'), float('nan')]

        # Publish the converted message
        self.vehicle_odom_pub.publish(px4_odom)

def main(args=None):
    rclpy.init(args=args)
    node = Px4OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
