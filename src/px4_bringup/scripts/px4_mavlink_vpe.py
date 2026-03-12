#!/usr/bin/env python3
"""
px4_mavlink_vpe.py — Vision Position Estimate via MAVLink on ttyACM0

Subscribes to /odometry/filtered (nav_msgs/Odometry, ENU/FLU) and sends
MAVLink VISION_POSITION_ESTIMATE + ATT_POS_MOCAP to PX4 via ttyACM0.

This deliberately bypasses the XRCE-DDS channel so that VO publishing
does NOT compete with arming_check_reply round-trips on ttyUSB0.

Usage
-----
  python3 px4_mavlink_vpe.py [--device /dev/ttyACM0] [--baud 57600] \
                              [--rate 20] [--topic /odometry/filtered]

PX4 parameters required:
  EKF2_EV_CTRL   = 15   (fuse pos + yaw + vel + hgt from external vision)
  EKF2_HGT_REF   = 3    (external vision height reference)
  EKF2_EV_DELAY  = 0    (adjust if clock offset is measurable)
"""

import argparse
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry

try:
    from pymavlink import mavutil
except ImportError:
    raise SystemExit("pymavlink not found — install with: pip install pymavlink")


# ── Quaternion helpers (all in [x, y, z, w] order) ──────────────────────────

def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    ]


def _quat_to_euler_ned_frd(q_xyzw):
    """Return (roll, pitch, yaw) in radians for a NED/FRD quaternion [x,y,z,w]."""
    x, y, z, w = q_xyzw
    # Standard aerospace ZYX Euler from quaternion
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin( max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw


def enu_flu_to_ned_frd(pos_enu, q_enu_xyzw, vel_flu):
    """
    Convert ENU/FLU pose+velocity to NED/FRD.

    Returns
    -------
    pos_ned  : [N, E, D]
    q_ned_xyzw : quaternion [x,y,z,w] in NED/FRD
    vel_frd  : [vx, vy, vz] in body FRD
    """
    # Position: ENU [E, N, U] → NED [N, E, D]
    pos_ned = [pos_enu[1], pos_enu[0], -pos_enu[2]]

    # Orientation:
    #   ENU→NED:  180° about (1,1,0)/√2  →  q = [√0.5, √0.5, 0, 0]  (x,y,z,w)
    #   FLU→FRD:  180° about X            →  q = [1, 0, 0, 0]          (x,y,z,w)
    q_enu_to_ned = [math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0]
    q_flu_to_frd = [1.0, 0.0, 0.0, 0.0]
    q_tmp     = _quat_mul(q_enu_to_ned, q_enu_xyzw)
    q_ned_frd = _quat_mul(q_tmp, q_flu_to_frd)

    # Velocity: body FLU → body FRD (negate Y and Z)
    vel_frd = [vel_flu[0], -vel_flu[1], -vel_flu[2]]

    return pos_ned, q_ned_frd, vel_frd


# ── ROS 2 node ───────────────────────────────────────────────────────────────

class MavlinkVPENode(Node):
    def __init__(self, conn_target: str, baud: int, rate_hz: float, odom_topic: str):
        super().__init__('mavlink_vpe')

        self._lock       = threading.Lock()
        self._odom_ready = False
        self._pos_ned    = [0.0, 0.0, 0.0]
        self._q_ned_xyzw = [0.0, 0.0, 0.0, 1.0]
        self._vel_frd    = [0.0, 0.0, 0.0]
        self._last_odom  = time.time()
        self._odom_timeout = 5.0

        # MAVLink connection (serial or UDP depending on conn_target)
        self.get_logger().info(f'Connecting MAVLink: {conn_target} @ {baud} baud …')
        if isinstance(conn_target, str) and (conn_target.startswith('udpout:') or conn_target.startswith('udp:') or conn_target.startswith('udpin:')):
            # UDP connection string (no baud)
            self._mav = mavutil.mavlink_connection(conn_target, source_system=1,
                                                   source_component=mavutil.mavlink.MAV_COMP_ID_PERIPHERAL)
        else:
            # Serial device
            self._mav = mavutil.mavlink_connection(conn_target, baud=baud,
                                                   source_system=1,
                                                   source_component=mavutil.mavlink.MAV_COMP_ID_PERIPHERAL)
        self.get_logger().info('MAVLink connection open')

        # ROS subscriber
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        self._sub = self.create_subscription(Odometry, odom_topic,
                                             self._odom_cb, qos)

        # Publish timer — runs at requested rate, fully independent of XRCE-DDS
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_cb)

        # Status watchdog
        self._watchdog = self.create_timer(1.0, self._watchdog_cb)

        self.get_logger().info(
            f'VPE bridge ready — topic: {odom_topic} | '
            f'MAVLink: {device} @ {baud} | rate: {rate_hz} Hz')

    # ── Odom callback ─────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        lv = msg.twist.twist.linear

        pos_enu  = [p.x, p.y, p.z]
        q_enu    = [o.x, o.y, o.z, o.w]
        vel_flu  = [lv.x, lv.y, lv.z]

        pos_ned, q_ned, vel_frd = enu_flu_to_ned_frd(pos_enu, q_enu, vel_flu)

        with self._lock:
            self._pos_ned    = pos_ned
            self._q_ned_xyzw = q_ned
            self._vel_frd    = vel_frd
            self._odom_ready = True
            self._last_odom  = time.time()

    # ── Publish timer ─────────────────────────────────────────────────────────

    def _publish_cb(self) -> None:
        with self._lock:
            if not self._odom_ready:
                return
            pos = list(self._pos_ned)
            q   = list(self._q_ned_xyzw)
            vel = list(self._vel_frd)

        usec = int(time.time() * 1e6)
        roll, pitch, yaw = _quat_to_euler_ned_frd(q)

        # VISION_POSITION_ESTIMATE — position + attitude, NED frame
        self._mav.mav.vision_position_estimate_send(
            usec,
            pos[0],   # x = North  [m]
            pos[1],   # y = East   [m]
            pos[2],   # z = Down   [m]
            roll,
            pitch,
            yaw,
        )

        # ATT_POS_MOCAP — redundant path; PX4 EKF2 uses whichever arrives first
        # Quaternion order for ATT_POS_MOCAP: [w, x, y, z]
        self._mav.mav.att_pos_mocap_send(
            usec,
            [q[3], q[0], q[1], q[2]],  # w, x, y, z
            pos[0],
            pos[1],
            pos[2],
        )

    # ── Status watchdog ───────────────────────────────────────────────────────

    def _watchdog_cb(self) -> None:
        elapsed = time.time() - self._last_odom
        if elapsed > self._odom_timeout and self._odom_ready:
            self.get_logger().warning(
                f'Odom topic silent for {elapsed:.1f} s — stopping VPE output')
            with self._lock:
                self._odom_ready = False


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='MAVLink VPE bridge for PX4')
    parser.add_argument('--device', default='/dev/ttyACM0',
                        help='MAVLink serial device (default: /dev/ttyACM0)')
    parser.add_argument('--baud',   type=int, default=57600,
                        help='Baud rate (default: 57600)')
    parser.add_argument('--udp-host', default=None,
                        help='Optional UDP host to send MAVLink to (e.g. 127.0.0.1)')
    parser.add_argument('--udp-port', type=int, default=None,
                        help='Optional UDP port to send MAVLink to (e.g. 14550)')
    parser.add_argument('--rate',   type=float, default=20.0,
                        help='Publish rate in Hz (default: 20)')
    parser.add_argument('--topic',  default='/odometry/filtered',
                        help='Source odometry topic (default: /odometry/filtered)')
    # Strip ROS args before argparse sees them
    import sys
    args, _ = parser.parse_known_args(
        [a for a in sys.argv[1:] if not a.startswith('__')])

    # Determine connection target: UDP if udp-host/udp-port provided, else serial device
    if args.udp_host and args.udp_port:
        conn = f'udpout:{args.udp_host}:{args.udp_port}'
    else:
        conn = args.device

    rclpy.init()
    node = MavlinkVPENode(conn, args.baud, args.rate, args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
