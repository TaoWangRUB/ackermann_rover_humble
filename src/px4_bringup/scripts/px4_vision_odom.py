#!/usr/bin/env python3

import threading
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
import tf2_ros
import numpy as np
from time import time


class VIOPublisher(Node):
    def __init__(self):
        super().__init__('vio_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'ackermann/base_link')

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # ── TF2 ───────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── I/O ───────────────────────────────────────────────────────────────
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, qos_profile)
        self.px4_pub = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', 10)

        # ── Internal state ────────────────────────────────────────────────────
        self.is_publishing  = False
        self.last_odom_time = time()
        self.odom_timeout   = 5.0
        self.px4_odom       = VehicleOdometry()
        self.lock           = threading.Lock()

        # Timers
        # 5 Hz avoids XRCE-DDS cycle stalls on STM32F427 (Cube Black).
        # At 20 Hz, back-pressure causes ~1s cycle stalls which push
        # arming_check_reply request_ids out of sync → watchdog unregisters mode.
        self.timer        = self.create_timer(1.0, self.check_odom_status)
        self.timer_output = self.create_timer(0.1, self.pub_px4_odom)

        self.get_logger().info(
            f'VIOPublisher ready — topic: {odom_topic} | '
            f'odom_frame: {odom_frame} | base_frame: {base_frame}')

    # ── Static helpers ────────────────────────────────────────────────────────

    @staticmethod
    def _quat_mul(a, b):
        """Multiply two quaternions, both in [x, y, z, w] order."""
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return [
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz,
        ]

    # ── Odom callback ─────────────────────────────────────────────────────────

    def odom_callback(self, msg: Odometry) -> None:
        """Convert ROS Odometry to PX4 VehicleOdometry.

        Pipeline
        --------
        1. TF2 lookup odom_frame → base_frame  (pose of robot in world ENU).
           Falls back to msg.pose if TF is unavailable.
        2. Position:    ENU [E, N, U] → NED [N, E, D].
        3. Orientation: ENU/FLU quaternion → NED/FRD quaternion.
        4. Velocity:    msg.twist body FLU → body FRD (negate Y and Z).
        """
        odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # ── Step 1: pose of base_frame in odom_frame ──────────────────────────
        try:
            tf = self.tf_buffer.lookup_transform(
                odom_frame, base_frame, rclpy.time.Time())
            t = tf.transform.translation
            r = tf.transform.rotation
            pos_enu = np.array([t.x, t.y, t.z])
            q_enu   = [r.x, r.y, r.z, r.w]      # [x, y, z, w]
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warn(
                f'TF {odom_frame} → {base_frame} unavailable: {exc}',
                throttle_duration_sec=2.0)
            # Fallback: use message pose (assumes frames match)
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            pos_enu = np.array([p.x, p.y, p.z])
            q_enu   = [o.x, o.y, o.z, o.w]

        # ── Step 2: position ENU → NED ────────────────────────────────────────
        # ENU: x=East, y=North, z=Up
        # NED: x=North, y=East, z=Down
        pos_ned = [pos_enu[1], pos_enu[0], -pos_enu[2]]

        # ── Step 3: orientation ENU/FLU → NED/FRD ────────────────────────────
        # ENU→NED: 180° about (1,1,0)/√2  →  q = [√0.5, √0.5, 0, 0]  (x,y,z,w)
        # FLU→FRD: 180° about X           →  q = [1, 0, 0, 0]          (x,y,z,w)
        q_enu_to_ned = [np.sqrt(0.5), np.sqrt(0.5), 0.0, 0.0]
        q_flu_to_frd = [1.0, 0.0, 0.0, 0.0]
        q_tmp     = self._quat_mul(q_enu_to_ned, q_enu)
        q_ned_frd = self._quat_mul(q_tmp, q_flu_to_frd)
        # PX4 VehicleOdometry expects q = [w, x, y, z]
        q_px4 = [q_ned_frd[3], q_ned_frd[0], q_ned_frd[1], q_ned_frd[2]]

        # ── Step 4: velocity body FLU → body FRD ─────────────────────────────
        # msg.twist is expressed in child_frame_id (body/FLU).
        # FLU→FRD: forward(x) same, left(y)→right(−y), up(z)→down(−z)
        lv = msg.twist.twist.linear
        av = msg.twist.twist.angular
        v_frd = [ lv.x, -lv.y, -lv.z]
        w_frd = [ av.x, -av.y, -av.z]

        # ── Populate VehicleOdometry ──────────────────────────────────────────
        with self.lock:
            px4 = self.px4_odom
            px4.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
            px4.timestamp_sample = (msg.header.stamp.sec * 1_000_000
                                    + msg.header.stamp.nanosec // 1000)

            # Position (NED)
            px4.pose_frame        = VehicleOdometry.POSE_FRAME_NED
            px4.position[0]       = float(pos_ned[0])
            px4.position[1]       = float(pos_ned[1])
            px4.position[2]       = float(pos_ned[2])
            px4.position_variance[0] = 0.01
            px4.position_variance[1] = 0.01
            px4.position_variance[2] = 0.01

            # Orientation (NED/FRD, PX4 [w,x,y,z])
            px4.q                    = [float(q_px4[0]), float(q_px4[1]),
                                        float(q_px4[2]), float(q_px4[3])]
            px4.orientation_variance[0] = 0.01
            px4.orientation_variance[1] = 0.01
            px4.orientation_variance[2] = 0.01

            # Velocity (body FRD)
            px4.velocity_frame    = VehicleOdometry.VELOCITY_FRAME_BODY_FRD
            px4.velocity[0]       = float(v_frd[0])
            px4.velocity[1]       = float(v_frd[1])
            px4.velocity[2]       = float(v_frd[2])
            px4.velocity_variance[0] = 0.01
            px4.velocity_variance[1] = 0.01
            px4.velocity_variance[2] = 0.01

            # Angular velocity (body FRD)
            px4.angular_velocity[0] = float(w_frd[0])
            px4.angular_velocity[1] = float(w_frd[1])
            px4.angular_velocity[2] = float(w_frd[2])

            px4.quality = 100

        if not self.is_publishing:
            self.get_logger().info(
                f'{self.odom_sub.topic_name} → /fmu/in/vehicle_visual_odometry')
            self.is_publishing = True
        self.last_odom_time = time()

    # ── Output timer ──────────────────────────────────────────────────────────

    def pub_px4_odom(self) -> None:
        with self.lock:
            if not self.is_publishing:
                return  # Don't send zero-valued garbage to EKF2 before first odom message
            self.px4_odom.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.px4_pub.publish(self.px4_odom)

    # ── Status watchdog ───────────────────────────────────────────────────────

    def check_odom_status(self) -> None:
        elapsed = time() - self.last_odom_time
        if elapsed > self.odom_timeout and self.is_publishing:
            self.get_logger().warning(
                f'Odom topic silent for {elapsed:.1f} s.')
            self.is_publishing = False


def main():
    rclpy.init()
    node = VIOPublisher()
    print('spin node!')
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()