#!/usr/bin/env python3
"""px4_vehicle_odometry.py

Two-stage pipeline:

Stage 1 — /fmu/out/vehicle_odometry (NED/FRD) → /px4_vehicle_odom (ENU/FLU)
  Converts raw PX4 VehicleOdometry to a ROS nav_msgs/Odometry expressed in
  the vehicle_odom (ENU) world frame with cubepilot_link as the body frame.

  Parameters:
    frame_id        (str, 'vehicle_odom')   world frame for the output
    child_frame_id  (str, 'cubepilot_link') body frame for the output

  Velocity handling (linear):
    VELOCITY_FRAME_BODY_FRD : negate y and z → body FLU
    VELOCITY_FRAME_NED      : NED→ENU then rotate world→body via conj(q_enu_flu)
  Angular velocity is always body FRD per the .msg spec → negate y and z.

Stage 2 — /px4_vehicle_odom → /px4_vehicle_odom_base
  Re-expresses the cubepilot-frame odometry in the rover base frame so it can
  be compared directly with the EKF odometry (/odometry/filtered).
  Mirrors px4_vision_odom.py in its parameter names (inverse pipeline).

  Parameters:
    odom_topic  (str, '/px4_vehicle_odom')     topic to subscribe to
    odom_frame  (str, 'odom')                  frame_id for the output (world)
    base_frame  (str, 'ackermann/base_link')   child_frame_id for the output (body)

Coordinate frames
-----------------
  PX4 VehicleOdometry : position NED, orientation NED/FRD, q=[w,x,y,z]
  Stage-1 output      : position ENU, orientation ENU/FLU, q=[x,y,z,w]
  Stage-2 output      : position ENU, orientation ENU/FLU, q=[x,y,z,w]
                        re-expressed at ackermann/base_link via rigid-body TF

NED→ENU position   : [E, N, U] = [ned[1], ned[0], -ned[2]]
NED/FRD→ENU/FLU q : conj(q_enu_to_ned) ⊗ q_ned_frd ⊗ conj(q_flu_to_frd)
                     (both frame-change quaternions are 180° → self-inverse)
FRD→FLU velocity   : negate y and z

Stage-2 pose composition (T_WC = received, T_CB from TF lookup):
  pos_WB = pos_WC + R(q_WC) * pos_CB
  q_WB   = q_WC ⊗ q_CB
  where T_CB = lookup_transform('cubepilot_link', base_frame)
               i.e. pose of base_frame expressed in cubepilot_link frame

Stage-2 velocity (rigid-body, all in cubepilot body frame):
  v_B_in_B  = R_BC * (v_C + ω_C × pos_CB)
  ω_B_in_B  = R_BC * ω_C
  where R_BC = conj(q_CB)  (maps C vectors → B vectors)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
import tf2_ros
import numpy as np


class PX4VehicleOdometry(Node):
    def __init__(self):
        super().__init__('px4_vehicle_odometry')

        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Stage-1 parameters ────────────────────────────────────────────
        self.declare_parameter('frame_id',       'vehicle_odom')
        self.declare_parameter('child_frame_id', 'cubepilot_link')

        self._frame_id       = self.get_parameter('frame_id').get_parameter_value().string_value
        self._child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

        # ── Stage-2 parameters (mirrors px4_vision_odom.py) ───────────────
        self.declare_parameter('odom_topic', '/px4_vehicle_odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'ackermann/base_link')

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # ── TF2 (for stage-2 cubepilot_link → base_frame lookup) ─────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # ── TF cache (to avoid lookup_transform on every high-rate message) ──
        self._tf_cache = None  # Cached (pos_CB, q_CB) tuple or None
        self._tf_cache_time = 0.0  # Time (seconds) of last cache refresh
        self._tf_cache_ttl = 5.0  # Cache TTL in seconds; refresh if stale

        # ── Stage-1: /fmu/out/vehicle_odometry → /px4_vehicle_odom ───────
        self._sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self._callback,
            px4_qos,
        )
        self._pub = self.create_publisher(Odometry, '/px4_vehicle_odom', 10)

        # ── Stage-2: /px4_vehicle_odom → /px4_vehicle_odom_base ──────────
        self._odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            10,
        )
        self._base_pub = self.create_publisher(Odometry, '/px4_vehicle_odom_base', 10)

        self.get_logger().info(
            f'px4_vehicle_odometry ready\n'
            f'  stage-1: /fmu/out/vehicle_odometry → /px4_vehicle_odom '
            f'[{self._frame_id} → {self._child_frame_id}]\n'
            f'  stage-2: {odom_topic} → /px4_vehicle_odom_base '
            f'[{odom_frame} → {base_frame}]'
        )

    # ── Static helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _quat_mul(a, b):
        """Quaternion multiply, both in [x, y, z, w]."""
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return [
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz,
        ]

    @staticmethod
    def _quat_conj(q):
        """Quaternion conjugate (= inverse for unit quaternion), [x,y,z,w]."""
        return [-q[0], -q[1], -q[2], q[3]]

    @staticmethod
    def _rotate_vec(q, v):
        """Rotate vector v by unit quaternion q ([x,y,z,w]) via sandwich product."""
        qv  = [v[0], v[1], v[2], 0.0]
        tmp = PX4VehicleOdometry._quat_mul(q, qv)
        res = PX4VehicleOdometry._quat_mul(tmp, PX4VehicleOdometry._quat_conj(q))
        return np.array([res[0], res[1], res[2]])

    # ── Stage-1 callback ────────────────────────────────────────────────────

    def _callback(self, msg: VehicleOdometry) -> None:
        """Convert PX4 VehicleOdometry (NED/FRD) → ROS Odometry (ENU/FLU)."""

        # ── Frame sanity checks ───────────────────────────────────────────
        # pose_frame must be NED — drop if not.
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            self.get_logger().warn(
                f'Unexpected pose_frame={msg.pose_frame} '
                f'(expected POSE_FRAME_NED={VehicleOdometry.POSE_FRAME_NED}). '
                'Message dropped — conversion assumes NED.',
                throttle_duration_sec=5.0,
            )
            return

        # velocity_frame: PX4 EKF2 publishes VELOCITY_FRAME_NED (1) or
        # VELOCITY_FRAME_BODY_FRD (3) depending on version/config. Both are
        # handled below. Any other value is dropped with a warning.
        vf = msg.velocity_frame
        if vf not in (VehicleOdometry.VELOCITY_FRAME_NED,
                      VehicleOdometry.VELOCITY_FRAME_BODY_FRD):
            self.get_logger().warn(
                f'Unexpected velocity_frame={vf} '
                f'(expected VELOCITY_FRAME_NED={VehicleOdometry.VELOCITY_FRAME_NED} '
                f'or VELOCITY_FRAME_BODY_FRD={VehicleOdometry.VELOCITY_FRAME_BODY_FRD}). '
                'Message dropped.',
                throttle_duration_sec=5.0,
            )
            return

        now = self.get_clock().now().to_msg()

        # ── Position: NED → ENU ───────────────────────────────────────────
        # PX4 NED:  x=North, y=East,  z=Down
        # ROS  ENU: x=East,  y=North, z=Up
        ned = msg.position           # float[3]
        pos_enu = [ned[1], ned[0], -ned[2]]

        # ── Orientation: NED/FRD → ENU/FLU ───────────────────────────────
        # PX4 q = [w, x, y, z]; convert to [x, y, z, w] for arithmetic.
        q_ned_frd = [msg.q[1], msg.q[2], msg.q[3], msg.q[0]]

        # Frame-change quaternions (180° rotations, each is its own inverse):
        #   ENU↔NED: rotate 180° about (1,1,0)/√2
        #   FLU↔FRD: rotate 180° about X
        q_enu_to_ned = [np.sqrt(0.5), np.sqrt(0.5), 0.0, 0.0]   # [x,y,z,w]
        q_flu_to_frd = [1.0,          0.0,           0.0, 0.0]   # [x,y,z,w]

        # q_enu_flu = conj(q_enu_to_ned) ⊗ q_ned_frd ⊗ conj(q_flu_to_frd)
        # For 180° rotations, conj == same rotation (negated quaternion = same),
        # so the formula is identical to the forward direction.
        q_tmp     = self._quat_mul(self._quat_conj(q_enu_to_ned), q_ned_frd)
        q_enu_flu = self._quat_mul(q_tmp, self._quat_conj(q_flu_to_frd))
        # q_enu_flu is in [x, y, z, w]

        # ── Linear velocity → body FLU ───────────────────────────────────
        vel = msg.velocity   # float[3]
        if vf == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
            # Body FRD → body FLU: negate y and z
            v_flu = [vel[0], -vel[1], -vel[2]]
        else:
            # VELOCITY_FRAME_NED: world NED → world ENU → body FLU
            # NED→ENU: [E, N, U] = [ned[1], ned[0], -ned[2]]
            v_enu = np.array([vel[1], vel[0], -vel[2]])
            # Rotate world ENU → body FLU using conj(q_enu_flu)
            # (q_enu_flu maps body→world; conj maps world→body)
            v_flu = self._rotate_vec(self._quat_conj(q_enu_flu), v_enu).tolist()

        # Angular velocity is always body FRD per the .msg spec (@VELOCITY_FRAME_BODY_FRD).
        # FRD→FLU: negate y and z.
        avel = msg.angular_velocity  # float[3] in body FRD
        w_flu = [avel[0], -avel[1], -avel[2]]

        # ── Build nav_msgs/Odometry ───────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self._frame_id
        odom.child_frame_id  = self._child_frame_id

        odom.pose.pose.position.x = float(pos_enu[0])
        odom.pose.pose.position.y = float(pos_enu[1])
        odom.pose.pose.position.z = float(pos_enu[2])

        odom.pose.pose.orientation.x = float(q_enu_flu[0])
        odom.pose.pose.orientation.y = float(q_enu_flu[1])
        odom.pose.pose.orientation.z = float(q_enu_flu[2])
        odom.pose.pose.orientation.w = float(q_enu_flu[3])

        odom.twist.twist.linear.x  = float(v_flu[0])
        odom.twist.twist.linear.y  = float(v_flu[1])
        odom.twist.twist.linear.z  = float(v_flu[2])

        odom.twist.twist.angular.x = float(w_flu[0])
        odom.twist.twist.angular.y = float(w_flu[1])
        odom.twist.twist.angular.z = float(w_flu[2])

        # Covariance: position/orientation variances from PX4 if available.
        # Lay variance on diagonal (indices 0, 7, 14 for pos; 21, 28, 35 for rot).
        pv = msg.position_variance
        ov = msg.orientation_variance
        odom.pose.covariance[0]  = float(pv[0]) if pv[0] > 0 else 0.01
        odom.pose.covariance[7]  = float(pv[1]) if pv[1] > 0 else 0.01
        odom.pose.covariance[14] = float(pv[2]) if pv[2] > 0 else 0.01
        odom.pose.covariance[21] = float(ov[0]) if ov[0] > 0 else 0.01
        odom.pose.covariance[28] = float(ov[1]) if ov[1] > 0 else 0.01
        odom.pose.covariance[35] = float(ov[2]) if ov[2] > 0 else 0.01

        vv = msg.velocity_variance
        odom.twist.covariance[0]  = float(vv[0]) if vv[0] > 0 else 0.01
        odom.twist.covariance[7]  = float(vv[1]) if vv[1] > 0 else 0.01
        odom.twist.covariance[14] = float(vv[2]) if vv[2] > 0 else 0.01

        self._pub.publish(odom)

    # ── Stage-2 callback ────────────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry) -> None:
        """Re-express /px4_vehicle_odom from cubepilot_link into base_frame.

        Pipeline (inverse sequence of px4_vision_odom.py):
          1. Receive pose of cubepilot_link in vehicle_odom (ENU/FLU).
          2. TF lookup  cubepilot_link → base_frame  (static, from URDF).
          3. Compose pose: T_world_base = T_world_cubepilot ∘ T_cubepilot_base.
          4. Rigid-body velocity transform: lever-arm correction + frame rotation.
          5. Publish as odom_frame / base_frame for direct comparison with EKF.

        Pose composition (all quaternions in [x,y,z,w]):
          pos_WB = pos_WC + R(q_WC) * pos_CB
          q_WB   = q_WC ⊗ q_CB
          where T_CB = lookup_transform(child_frame_id, base_frame)
                     = pose of base_frame origin in cubepilot_link frame.

        Velocity (rigid body, in cubepilot body frame → base body frame):
          v_B_in_B  = R_BC * (v_C + ω_C × pos_CB)   lever-arm + frame rotation
          ω_B_in_B  = R_BC * ω_C                     frame rotation only
          where R_BC = rotation by conj(q_CB)  (maps C-frame vectors to B-frame)
        """
        odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # ── Step 1: pose of cubepilot_link (C) in vehicle_odom (W) ───────
        p  = msg.pose.pose.position
        o  = msg.pose.pose.orientation
        pos_WC = np.array([p.x, p.y, p.z])
        q_WC   = [o.x, o.y, o.z, o.w]   # [x,y,z,w]

        # ── Step 2: TF cubepilot_link → base_frame (with caching) ──────────
        # lookup_transform(target, source) returns T that maps source → target.
        # lookup_transform(child_frame_id, base_frame):
        #   translation = position of base_frame origin in child_frame_id frame
        #   rotation    = orientation of base_frame in child_frame_id frame
        # This is T_CB: the pose of base_frame expressed in cubepilot_link.
        # 
        # OPTIMIZATION: Cache the transform with a 5-second TTL to avoid repeated
        # expensive lookups at 200 Hz. Since the transform is static (from URDF),
        # this is safe and reduces CPU usage significantly.
        child_frame = msg.child_frame_id   # 'cubepilot_link'
        
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self._tf_cache is not None and (now_sec - self._tf_cache_time) < self._tf_cache_ttl:
            # Use cached transform
            pos_CB, q_CB = self._tf_cache
        else:
            # Cache is stale or empty; refresh via TF lookup
            try:
                tf_cb = self.tf_buffer.lookup_transform(
                    child_frame, base_frame, rclpy.time.Time())
                t = tf_cb.transform.translation
                r = tf_cb.transform.rotation
                pos_CB = np.array([t.x, t.y, t.z])   # base_frame origin in C frame
                q_CB   = [r.x, r.y, r.z, r.w]        # base_frame orientation in C frame
                # Update cache
                self._tf_cache = (pos_CB.copy(), q_CB)
                self._tf_cache_time = now_sec
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as exc:
                self.get_logger().warn(
                    f'TF {child_frame} → {base_frame} unavailable: {exc}',
                    throttle_duration_sec=2.0,
                )
                # Fallback: treat frames as coincident (no offset, no rotation)
                pos_CB = np.zeros(3)
                q_CB   = [0.0, 0.0, 0.0, 1.0]

        # ── Step 3: compose pose ──────────────────────────────────────────
        # pos_WB = pos_WC + R(q_WC) * pos_CB
        # q_WB   = q_WC ⊗ q_CB
        pos_WB = pos_WC + self._rotate_vec(q_WC, pos_CB)
        q_WB   = self._quat_mul(q_WC, q_CB)

        # ── Step 4: rigid-body velocity transform ─────────────────────────
        # All vectors currently in cubepilot body frame C (FLU).
        lv  = msg.twist.twist.linear
        av  = msg.twist.twist.angular
        v_C = np.array([lv.x, lv.y, lv.z])   # linear velocity of C in C frame
        w_C = np.array([av.x, av.y, av.z])   # angular velocity in C frame

        # Velocity of point B (base_frame origin) still in frame C:
        #   v_B_in_C = v_C + ω_C × pos_CB
        v_B_in_C = v_C + np.cross(w_C, pos_CB)

        # Rotate from C frame to B frame (R_BC = conj(q_CB) maps C → B):
        q_BC = self._quat_conj(q_CB)   # rotation from C to B
        v_B  = self._rotate_vec(q_BC, v_B_in_C)
        w_B  = self._rotate_vec(q_BC, w_C)

        # ── Step 5: publish ───────────────────────────────────────────────
        out = Odometry()
        out.header.stamp    = msg.header.stamp
        out.header.frame_id = odom_frame
        out.child_frame_id  = base_frame

        out.pose.pose.position.x = float(pos_WB[0])
        out.pose.pose.position.y = float(pos_WB[1])
        out.pose.pose.position.z = float(pos_WB[2])

        out.pose.pose.orientation.x = float(q_WB[0])
        out.pose.pose.orientation.y = float(q_WB[1])
        out.pose.pose.orientation.z = float(q_WB[2])
        out.pose.pose.orientation.w = float(q_WB[3])

        out.twist.twist.linear.x  = float(v_B[0])
        out.twist.twist.linear.y  = float(v_B[1])
        out.twist.twist.linear.z  = float(v_B[2])

        out.twist.twist.angular.x = float(w_B[0])
        out.twist.twist.angular.y = float(w_B[1])
        out.twist.twist.angular.z = float(w_B[2])

        # Pass covariance through unchanged (already in correct body frame
        # for the diagonal pose/twist variances used here).
        out.pose.covariance  = msg.pose.covariance
        out.twist.covariance = msg.twist.covariance

        self._base_pub.publish(out)


def main():
    rclpy.init()
    node = PX4VehicleOdometry()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
