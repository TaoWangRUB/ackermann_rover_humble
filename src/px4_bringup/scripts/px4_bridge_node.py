#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3Stamped
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_vector3
from std_srvs.srv import SetBool, Trigger

class Px4BridgeNode(Node):
    def __init__(self):
        super().__init__('px4_dds_bridge')

        # Parameters
        self.declare_parameter('command_rate_hz', 50.0)
        self.declare_parameter('base_frame', 'ackermann/base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        
        rate = self.get_parameter('command_rate_hz').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value

        # Subscriptions
        # Taking standard cmd_vel Twist
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, 10)

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Services
        self.arm_srv = self.create_service(SetBool, '/px4/arm', self.arm_callback)
        self.offboard_srv = self.create_service(Trigger, '/px4/set_offboard', self.offboard_callback)

        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer for offboard heartbeat AND setpoint publishing
        timer_period = 1.0 / rate
        self.heartbeat_timer = self.create_timer(timer_period, self.timer_callback)

        self.current_cmd_vel = Twist()
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        
        self.get_logger().info("PX4 Bridge Node started. Waiting for cmd_vel...")

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def cmd_vel_callback(self, msg):
        self.current_cmd_vel = msg

    def timer_callback(self):
        # Publish OffboardControlMode heartbeat (required at >2Hz to keep offboard)
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

        # Publish TrajectorySetpoint translated into NED frame velocity
        self.publish_trajectory_setpoint()

    def publish_trajectory_setpoint(self):
        try:
            # Look up transform from base frame to odom
            t = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time())
                
            # Populate Vector3 with linear velocity in base frame (FLU)
            vel_base = Vector3Stamped()
            vel_base.header.frame_id = self.base_frame
            vel_base.header.stamp = self.get_clock().now().to_msg()
            vel_base.vector.x = self.current_cmd_vel.linear.x
            vel_base.vector.y = self.current_cmd_vel.linear.y
            vel_base.vector.z = self.current_cmd_vel.linear.z

            # Apply rotation from base_frame to odom frame (ENU)
            vel_odom = do_transform_vector3(vel_base, t)

            # Convert from ENU (odom) to NED (PX4 local frame)
            # ENU X (East) -> NED Y (East)
            # ENU Y (North)-> NED X (North)
            # ENU Z (Up)   -> NED Z (Down)
            vel_ned_x = vel_odom.vector.y
            vel_ned_y = vel_odom.vector.x
            vel_ned_z = -vel_odom.vector.z

            setpoint = TrajectorySetpoint()
            setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            setpoint.velocity = [float(vel_ned_x), float(vel_ned_y), float(vel_ned_z)]
            # Angular velocity Z is CCW (FLU), PX4 wants CW (NED)
            setpoint.yawspeed = float(-self.current_cmd_vel.angular.z) 

            # Rest are NaNs by standard convention for velocity control
            setpoint.position = [float('nan'), float('nan'), float('nan')]
            setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
            setpoint.yaw = float('nan')

            self.trajectory_setpoint_pub.publish(setpoint)
            
        except TransformException as ex:
            # Mute flooding until TF is fully ready
            self.get_logger().debug(f'TF skip: {ex}')

    # --- ROS Services ---
    def arm_callback(self, request, response):
        if request.data:
            self.arm()
            response.message = "Arm command sent to PX4"
        else:
            self.disarm()
            response.message = "Disarm command sent to PX4"
        response.success = True
        return response

    def offboard_callback(self, request, response):
        self.set_offboard_mode()
        response.success = True
        response.message = "Offboard mode command sent to PX4"
        return response

    # --- Utility commands for ARM/DISARM/OFFBOARD via rosservice calling or local use ---
    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def set_offboard_mode(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard mode command sent')


def main(args=None):
    rclpy.init(args=args)
    node = Px4BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
