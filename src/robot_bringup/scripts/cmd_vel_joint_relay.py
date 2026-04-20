#!/usr/bin/env python3
"""
cmd_vel_joint_relay.py

Derives /joint_states from /cmd_vel using inverse Ackermann kinematics.
Used in hardware mode (use_gazebo:=false) to keep the TF tree complete and
animate the robot model in RViz.  FOR VISUALISATION ONLY.

Signal chain context
--------------------
In Gazebo + PX4 SITL the joint-state feed is physics-derived:

  /cmd_vel → rover_speed_steering_mode → PX4 control loop
           → gz_bridge (uORB → Gz transport: servo_0, command/motor_speed)
           → Gazebo JointPositionController / JointController (P-gain)
           → Gazebo physics engine  ← actual simulated dynamics
           → gz-sim-joint-state-publisher-system  ← reads real simulated state
           → joint_state_bridge → /joint_states

On real hardware that feedback loop does not exist:

  /cmd_vel → rover_speed_steering_mode → PX4 control loop
           → PWM/UART → ESC → motor / servo → physical joints
                                              ← NO feedback to ROS

Why /cmd_vel and not PX4 ActuatorServos / ActuatorMotors:
  - Those are fmu/in/ topics (external → PX4 override), NOT PX4 output topics.
    PX4 does not publish its actuator state via DDS; the gz_bridge path is fully
    internal (uORB → Gz transport, no DDS involved).
  - Even if repurposed as feedback they are still upstream of ESC/servo response
    ("commanded, not measured") — same principle as /cmd_vel but with added DDS
    dependency and uncalibrated scaling (normalised [-1,1] → rad/rpm).
  - /cmd_vel works when only cameras + Nav2 are running (no PX4 DDS required).

Kinematics (per 50 Hz publish cycle)
-------------------------------------
  R      = linear.x / angular.z          (signed turn radius)
  δ_left  = atan2(L, R - T/2)            (inner / outer kingpin, full Ackermann)
  δ_right = atan2(L, R + T/2)
  ω_wheel = linear.x / wheel_radius      (same for all 4 — single ESC channel)
  θ_wheel += ω_wheel * dt                (integrated for mesh spin animation)

Parameters
----------
wheelbase    (float, 0.174 m)    Rear-axle to front-axle distance.
track        (float, 0.174 m)    Left-to-right wheel-centre distance.
wheel_radius (float, 0.0385 m)   Wheel radius.
robot_ns     (str,  'ackermann')  Prefix used in joint names (matches URDF ns arg).
publish_rate (float, 50.0 Hz)    Joint-state publication rate.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState


class CmdVelJointRelay(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_joint_relay')

        self.declare_parameter('wheelbase',    0.174)
        self.declare_parameter('track',        0.174)
        self.declare_parameter('wheel_radius', 0.0385)
        self.declare_parameter('robot_ns',     'ackermann')
        self.declare_parameter('publish_rate', 50.0)

        L  = self.get_parameter('wheelbase').value
        T  = self.get_parameter('track').value
        r  = self.get_parameter('wheel_radius').value
        ns = self.get_parameter('robot_ns').value
        hz = self.get_parameter('publish_rate').value

        self._L, self._T, self._r = L, T, r

        # Joint name order must match ackermann_controller.yaml and the URDF.
        self._names = [
            f'{ns}/rear_left_wheel_joint',
            f'{ns}/rear_right_wheel_joint',
            f'{ns}/front_left_wheel_joint',
            f'{ns}/front_right_wheel_joint',
            f'{ns}/front_left_wheel_steering_joint',
            f'{ns}/front_right_wheel_steering_joint',
        ]

        # Accumulated wheel rotation (rad), integrated from cmd_vel.
        # Order: [rear_left, rear_right, front_left, front_right]
        self._wheel_pos = [0.0, 0.0, 0.0, 0.0]

        self._vx = 0.0   # last commanded linear.x  (m/s)
        self._wz = 0.0   # last commanded angular.z  (rad/s)
        self._last_t = self.get_clock().now()

        self._sub = self.create_subscription(
            TwistStamped, 'cmd_vel', self._on_cmd_vel_stamped, 10
        )
        self._pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(1.0 / hz, self._publish)

        self.get_logger().info(
            f'cmd_vel_joint_relay ready  '
            f'(listening on /cmd_vel as TwistStamped, '
            f'L={L} m  T={T} m  r={r} m  ns={ns}  rate={hz} Hz)'
        )

    # ------------------------------------------------------------------

    def _on_cmd_vel_stamped(self, msg: TwistStamped) -> None:
        self._vx = msg.twist.linear.x
        self._wz = msg.twist.angular.z

    def _ackermann_angles(self, v: float, w: float):
        """
        Full per-kingpin Ackermann steering angles from bicycle-model cmd_vel.

        Turn radius (signed, positive = left):  R = v / w

        Left  kingpin:  delta_L = atan2(L,  R - T/2)   (inner wheel on left turn)
        Right kingpin:  delta_R = atan2(L,  R + T/2)   (outer wheel on left turn)

        atan2(L, R±T/2) handles all sign combinations of R automatically.
        Output is clamped to the URDF joint limits of ±0.6 rad.
        """
        L, T = self._L, self._T
        LIMIT = 0.6  # rad — matches URDF front_*_wheel_steering_joint limits

        if abs(w) < 1e-6:
            return 0.0, 0.0

        if abs(v) < 1e-3:
            # Pure-rotation command: kinematically ill-defined for Ackermann.
            # Apply max steering in the commanded direction; wheels stay still.
            s = math.copysign(LIMIT, w)
            return s, s

        R  = v / w
        dl = math.atan2(L, R - T / 2.0)
        dr = math.atan2(L, R + T / 2.0)
        return (
            max(-LIMIT, min(LIMIT, dl)),
            max(-LIMIT, min(LIMIT, dr)),
        )

    def _publish(self) -> None:
        now = self.get_clock().now()
        dt  = (now - self._last_t).nanoseconds * 1e-9
        self._last_t = now

        v, w = self._vx, self._wz
        r    = self._r

        # Both rear wheels commanded at the same speed (single ESC channel).
        omega_rear = v / r

        # Front wheels free-roll; approximate at rear speed for display.
        # True path-radius correction is < 2 % at max steer for this geometry.
        omega_front = v / r

        # Integrate wheel rotation positions.
        self._wheel_pos[0] += omega_rear  * dt
        self._wheel_pos[1] += omega_rear  * dt
        self._wheel_pos[2] += omega_front * dt
        self._wheel_pos[3] += omega_front * dt

        delta_l, delta_r = self._ackermann_angles(v, w)

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name     = self._names
        msg.position = [
            self._wheel_pos[0], self._wheel_pos[1],   # rear  l/r spin angle
            self._wheel_pos[2], self._wheel_pos[3],   # front l/r spin angle
            delta_l,            delta_r,               # steering angles
        ]
        msg.velocity = [
            omega_rear,  omega_rear,
            omega_front, omega_front,
            0.0,         0.0,
        ]
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = CmdVelJointRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
