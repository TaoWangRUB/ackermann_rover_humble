#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    VehicleCommand, TrajectorySetpoint, OffboardControlMode,
    ActuatorMotors, VehicleAttitudeSetpoint, VehicleStatus, VehicleRatesSetpoint,
)
from geometry_msgs.msg import Twist, TwistStamped
import tf2_ros
import numpy as np

from enum import Enum, auto

# Define the enum
class OffboardControlFlag(Enum):
    POSITION = auto()
    VELOCITY = auto()
    ACCELERATION = auto()
    ATTITUDE = auto()
    BODY_RATE = auto()
    THRUST_AND_TORQUE = auto()
    DIRECT_ACTUATOR = auto()

# Define the global constant mapping
FLAG_MAPPING = {
    OffboardControlFlag.POSITION: "position",
    OffboardControlFlag.VELOCITY: "velocity",
    OffboardControlFlag.ACCELERATION: "acceleration",
    OffboardControlFlag.ATTITUDE: "attitude",
    OffboardControlFlag.BODY_RATE: "body_rate",
    OffboardControlFlag.THRUST_AND_TORQUE: "thrust_and_torque",
    OffboardControlFlag.DIRECT_ACTUATOR: "direct_actuator"
}


def px4_topic(base_topic, msg_type):
    if msg_type.MESSAGE_VERSION == 0:
        return base_topic
    return f"{base_topic}_v{msg_type.MESSAGE_VERSION}"

class PX4OffboardControl(Node):
    def __init__(self):
        super().__init__('px4_offboard_control')
        
        # Publishers for sending commands to PX4
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        
        # offboard heartbeat publisher
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        
        # vehicle command publisher via MavLink
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # setpointer publisher
        # control via motor directly
        self.publisher_actuator = self.create_publisher(
            ActuatorMotors,'/fmu/in/actuator_motors', qos_profile)
        
        # control via trajectory setpoint
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        # control via attitude setpoint
        self.publisher_attitude = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        
        # control via body rate setpoint
        self.publisher_bodyrate = self.create_publisher(
            VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_profile)
        
        # subscribe to vehicle status
        self.status_sub = self.create_subscription(
            VehicleStatus,
            px4_topic('/fmu/out/vehicle_status', VehicleStatus),
            self.vehicle_status_callback,
            qos_profile)

        # ── cmd_vel topic / frame parameters ────────────────────────────────
        self.declare_parameter('cmd_vel_topic',         '/cmd_vel')
        self.declare_parameter('cmd_vel_stamped_topic', '/cmd_vel_stamped')
        self.declare_parameter('base_frame',            'ackermann/base_link')
        self.declare_parameter('odom_frame',            'odom')

        _topic         = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        _topic_stamped = self.get_parameter('cmd_vel_stamped_topic').get_parameter_value().string_value

        # ── cmd_vel subscriptions (both stamped and unstamped) ──────────────
        self.cmd_vel_sub = self.create_subscription(
            Twist, _topic, self.cmd_vel_callback, 10)
        self.cmd_vel_stamped_sub = self.create_subscription(
            TwistStamped, _topic_stamped, self.cmd_vel_stamped_callback, 10)

        # Shared cmd_vel state (thread-safe)
        self._cmd_vel_lock = threading.Lock()
        self._cmd_linear  = np.zeros(3)          # [vx, vy, vz] in source frame
        self._cmd_angular = np.zeros(3)          # [wx, wy, wz] in source frame
        self._cmd_frame   = self.get_parameter('base_frame').get_parameter_value().string_value

        # ── TF2 listener for frame transforms ──────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.publish_velocity_setpoint)

        self.nav_state    = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        # Actuator helper state (retained for publish_actuator_setpoint)
        self.offboard_setpoint_counter = 0
        self.actuators        = ActuatorMotors()
        self.THROTTLE_CHANNEL = 2
        self.STEERING_CHANNEL = 0
        self.PWM_MIN     = 1000
        self.PWM_NEUTRAL = 1500
        self.PWM_MAX     = 2000
        self.PWM_RANGE   = self.PWM_MAX - self.PWM_NEUTRAL
        self.V_MAX       = 5
        self.STEER_MAX   = 30

        self.get_logger().info('px4_offboard_control initialised — listening on /cmd_vel and /cmd_vel_stamped')
    
    # ── cmd_vel callbacks ───────────────────────────────────────────────────

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Unstamped Twist — frame is taken from the 'base_frame' ROS parameter."""
        frame = self.get_parameter('base_frame').get_parameter_value().string_value
        with self._cmd_vel_lock:
            self._cmd_linear  = np.array([msg.linear.x,  msg.linear.y,  msg.linear.z])
            self._cmd_angular = np.array([msg.angular.x, msg.angular.y, msg.angular.z])
            self._cmd_frame   = frame

    def cmd_vel_stamped_callback(self, msg: TwistStamped) -> None:
        """Stamped TwistStamped — source frame taken from header.frame_id."""
        with self._cmd_vel_lock:
            self._cmd_linear  = np.array([msg.twist.linear.x,  msg.twist.linear.y,  msg.twist.linear.z])
            self._cmd_angular = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
            self._cmd_frame   = msg.header.frame_id or self.get_parameter('base_frame').get_parameter_value().string_value

    # ── TF helper ───────────────────────────────────────────────────────────

    def _quat_to_rot(self, x: float, y: float, z: float, w: float) -> np.ndarray:
        """Quaternion [x,y,z,w] → 3×3 rotation matrix."""
        return np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
            [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ])

    # ── vehicle status callback ─────────────────────────────────────────────

    def vehicle_status_callback(self, msg):
        """Track arming and nav-state changes."""
        if self.nav_state != msg.nav_state:
            self.get_logger().info(
                'offboard status: %d → %d' % (self.nav_state, msg.nav_state))
            self.nav_state = msg.nav_state
        if self.arming_state != msg.arming_state:
            self.get_logger().info(
                'arm status: %d → %d' % (self.arming_state, msg.arming_state))
            self.arming_state = msg.arming_state
        
    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")
    
    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')
    
    def disarm(self):
        """Send an disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Arm command sent')

    def actuator_control(self, steering, throttle):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR, param1=steering, param3=throttle)
    
    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
    
    def publish_offboard_control_heartbeat_signal(self, flag_selector: OffboardControlFlag):
        """
        Publish the offboard control mode.
        
        :param flag_selector: An OffboardControlFlag enum value to select which flag to set to True.
        """
        msg = OffboardControlMode()
        
        # Initialize all flags to False
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        
        # Use the global FLAG_MAPPING constant
        if flag_selector not in FLAG_MAPPING:
            raise ValueError(f"Invalid flag_selector: {flag_selector}. Must be one of {list(FLAG_MAPPING.keys())}")
        
        # Set the corresponding flag to True
        setattr(msg, FLAG_MAPPING[flag_selector], True)
        
        # Set the timestamp
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Publish the message
        self.offboard_control_mode_publisher.publish(msg)
    
    def calculate_ackermann_actuators(self, speed, steering_angle):
        """Convert PWM inputs to normalized actuator values with full bi-directional control"""
        
        # Convert throttle PWM to [-1, 1]
        # PWM: 1000 (full reverse) -> 1500 (stop) -> 2000 (full forward)
        throttle_pwm = self.PWM_NEUTRAL + 400 * speed / self.V_MAX
        throttle = (throttle_pwm - self.PWM_NEUTRAL) / self.PWM_RANGE
        throttle = np.clip(throttle, -1.0, 1.0)

        # Convert steering PWM to [-1, 1] 
        # PWM: 1000 (full left) -> 1500 (center) -> 2000 (full right)
        steering_pwm = self.PWM_NEUTRAL + 400 * steering_angle / self.STEER_MAX
        steering = (steering_pwm - self.PWM_NEUTRAL) / self.PWM_RANGE
        steering = np.clip(steering, -1.0, 1.0)

        return throttle, steering
    
    def publish_actuator_setpoint(self):
        """ 
        timer callback to send actuator setpoint
        Actuator Output has to be set for correct channel that been connected with servo/motor
        peripheral via Actuator Set X should be selected
        """ 
        
        # offboard streaming should be available before arming
        self.publish_offboard_control_heartbeat_signal(OffboardControlFlag.DIRECT_ACTUATOR)
        
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1
        
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # Compute actuators control value [-1, 1]
            # Calculate actuator outputs
            throttle, steering = self.calculate_ackermann_actuators(self.speed, self.steering_angle)  # <---

            # Set actuator values (adjust channels based on your mixer)
            self.actuators.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.actuators.timestamp_sample = int(self.get_clock().now().nanoseconds / 1000)
            self.actuators.control[self.THROTTLE_CHANNEL] = throttle  # <---
            self.actuators.control[self.STEERING_CHANNEL] = steering  # <---
            #self.publisher_actuator.publish(self.actuators)
            self.actuator_control(-0.5, 0.2)
            #self.get_logger().info("throttle = %f, steering = %f" % (throttle, steering) )
	
    def publish_trajectory_setpoint(self):
        """ 
        timer callback to send trajectory setpoint in NED frame
        """ 
        # offboard streaming should be available before arming
        self.publish_offboard_control_heartbeat_signal(OffboardControlFlag.POSITION)
        
        # publish setpoint in offboard mode and vehicle is armed
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and \
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            trajectory_msg.position[0] = self.radius * np.cos(self.theta)
            trajectory_msg.position[1] = self.radius * np.sin(self.theta)
            #trajectory_msg.position[2] = -self.altitude 
            #trajectory_msg.velocity[0] = self.omega * self.radius
            #trajectory_msg.yaw = self.theta
            trajectory_msg.yawspeed = self.omega
            self.publisher_trajectory.publish(trajectory_msg)

            self.theta = self.theta + self.omega * self.dt

    def publish_velocity_setpoint(self) -> None:
        """Timer callback (50 Hz): forward /cmd_vel to PX4 as a NED velocity setpoint.

        Pipeline
        --------
        1. Read latest cmd_vel (thread-safe).
        2. If source frame ≠ ackermann/base_link, rotate via TF2 (velocity only).
        3. Rotate body velocity (FLU) → world ENU using odom→base_link transform.
        4. Convert ENU → NED: [N, E, D] = [enu_y, enu_x, −enu_z].
        5. Publish TrajectorySetpoint with velocity + yaw-rate to PX4.
        """
        # Always stream the offboard heartbeat so PX4 knows we are alive
        self.publish_offboard_control_heartbeat_signal(OffboardControlFlag.VELOCITY)

        if not (self.nav_state    == VehicleStatus.NAVIGATION_STATE_OFFBOARD and
                self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            return

        # ── Step 0: snapshot cmd_vel ────────────────────────────────────────
        with self._cmd_vel_lock:
            v_src    = self._cmd_linear.copy()   # [vx, vy, vz] in source frame
            w_src    = self._cmd_angular.copy()  # [wx, wy, wz] in source frame
            src_frame = self._cmd_frame

        BASE_FRAME = self.get_parameter('base_frame').get_parameter_value().string_value

        # ── Step 1: source frame → ackermann/base_link ──────────────────────
        # (pure rotation — velocity vectors are unaffected by translation)
        v_body = v_src
        w_body = w_src
        if src_frame != BASE_FRAME:
            try:
                tf_base_src = self.tf_buffer.lookup_transform(
                    BASE_FRAME, src_frame, rclpy.time.Time())
                q = tf_base_src.transform.rotation
                R = self._quat_to_rot(q.x, q.y, q.z, q.w)
                v_body = R @ v_src
                w_body = R @ w_src
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as exc:
                self.get_logger().warn(
                    f'TF {src_frame} → {BASE_FRAME} unavailable: {exc}',
                    throttle_duration_sec=2.0)
                return

        # ── Step 2: ackermann/base_link (FLU) → world ENU ──────────────────
        # Requires the SLAM/odom stack to be publishing the odom→base_link TF.
        try:
            tf_odom_base = self.tf_buffer.lookup_transform(
                self.get_parameter('odom_frame').get_parameter_value().string_value,
                BASE_FRAME, rclpy.time.Time())
            q2 = tf_odom_base.transform.rotation
            R_world = self._quat_to_rot(q2.x, q2.y, q2.z, q2.w)
            v_enu = R_world @ v_body
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warn(
                f'TF odom → {BASE_FRAME} unavailable: {exc}',
                throttle_duration_sec=2.0)
            return

        # ── Step 3: ENU → NED ───────────────────────────────────────────────
        # ENU: x=East, y=North, z=Up
        # NED: x=North, y=East, z=Down
        v_ned = np.array([v_enu[1], v_enu[0], -v_enu[2]])

        # Yaw rate: FLU w_z (CCW+) → NED yawspeed (CW+), so negate
        yawspeed_ned = -float(w_body[2])

        # ── Step 4: publish ─────────────────────────────────────────────────
        msg = TrajectorySetpoint()
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
        msg.velocity[0]  = float(v_ned[0])
        msg.velocity[1]  = float(v_ned[1])
        msg.velocity[2]  = float(v_ned[2])
        msg.yawspeed     = yawspeed_ned
        self.publisher_trajectory.publish(msg)
            
    def publish_attitude_setpoint(self):
        """ 
        timer callback to send attitude setpoint in local odom frame
        """
        # offboard streaming should be available before arming
        self.publish_offboard_control_heartbeat_signal(OffboardControlFlag.ATTITUDE)
        
        # publish setpoint in offboard mode and vehicle is armed
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and \
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            
            msg = VehicleAttitudeSetpoint()

            # Set the timestamp (in microseconds)
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            
            # Set a desired quaternion.
            # For a no-rotation (neutral attitude) the quaternion is identity: [w, x, y, z] = [1, 0, 0, 0].
            msg.q_d = [1.0, 0.0, 0.0, 0.0]
            
            # Set yaw rate setpoint (rad/s); adjust this value as needed.
            msg.yaw_sp_move_rate = 0.0
            
            # Set thrust command in the body frame.
            # For multicopters, typically thrust_body[2] is negative throttle demand.
            # For a rover, adjust these values based on your vehicle configuration.
            msg.thrust_body = [0.0, 0.0, 0.0]
            
            # Optional flags:
            msg.reset_integral = False
            msg.fw_control_yaw_wheel = False
            
            # pub setpoint
            self.publisher_attitude.publish(msg)
	
    def publish_bodyrate_setpoint(self):
        """ 
        timer callback to send body setpoint in FRD frame
        """
        # offboard streaming should be available before arming
        self.publish_offboard_control_heartbeat_signal(OffboardControlFlag.BODY_RATE)
        
        # publish setpoint in offboard mode and vehicle is armed
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and \
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            
            msg = VehicleRatesSetpoint()
            # Set the timestamp (in microseconds)
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            
            # Set desired angular rates in rad/s (adjust as needed)
            msg.roll  = 0.0  # Commanded roll rate
            msg.pitch = 0.0  # Commanded pitch rate
            msg.yaw   = self.omega  # Commanded yaw rate
            
            # Set desired throttle in FxRyDz frame
            msg.thrust_body = [0.5, 0.0, 0.0]
            
            # Publish the rate setpoint
            self.publisher_bodyrate.publish(msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = PX4OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
