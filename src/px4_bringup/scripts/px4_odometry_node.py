#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry


def _quat_mult(a, b):
    """Hamilton quaternion product (w, x, y, z) convention."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    )


def _quat_conj(q):
    """Quaternion conjugate (== inverse for unit quaternions)."""
    return (q[0], -q[1], -q[2], -q[3])


# Static rotation quaternions (matches GZBridge.cpp)
# FLU -> FRD : 180° about X
Q_FLU_TO_FRD = (0.0, 1.0, 0.0, 0.0)
# ENU -> NED : +PI/2 about Z then +PI about X  (symmetric: also NED -> ENU)
Q_ENU_TO_NED = (0.0, 0.7071067811865475, 0.7071067811865475, 0.0)


def rotate_quaternion(q_flu_to_enu):
    """Convert orientation from FLU->ENU to FRD->NED.

    Mirrors GZBridge::rotateQuaternion():
        q_FRD_to_NED = q_ENU_to_NED * q_FLU_to_ENU * inv(q_FLU_to_FRD)
    """
    return _quat_mult(
        _quat_mult(Q_ENU_TO_NED, q_flu_to_enu),
        _quat_conj(Q_FLU_TO_FRD),
    )

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

        # Frames
        px4_odom.pose_frame = VehicleOdometry.POSE_FRAME_NED
        px4_odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD

        # Position Conversion: ENU -> NED
        # x_ned = y_enu
        # y_ned = x_enu
        # z_ned = -z_enu
        px4_odom.position = [
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.x),
            float(-msg.pose.pose.position.z)
        ]

        # Orientation Conversion: FLU→ENU quaternion to FRD→NED quaternion
        # q_FRD→NED = q_ENU→NED * q_FLU→ENU * inv(q_FLU→FRD)
        q = msg.pose.pose.orientation
        q_ned = rotate_quaternion((q.w, q.x, q.y, q.z))
        px4_odom.q = [float(q_ned[0]), float(q_ned[1]),
                      float(q_ned[2]), float(q_ned[3])]

        # Velocity Conversion: body FLU -> body FRD (twist is in child/body frame)
        px4_odom.velocity = [
            float(msg.twist.twist.linear.x),    # forward (same)
            float(-msg.twist.twist.linear.y),   # left -> -right
            float(-msg.twist.twist.linear.z),   # up -> -down
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
