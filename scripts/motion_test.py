#!/usr/bin/env python3
"""Test script to verify robot motion by publishing cmd_vel and checking odom.

Usage:
    python3 scripts/motion_test.py

Publishes cmd_vel at 1.0 m/s for 5 seconds and reports distance traveled.
Requires Gazebo simulation with ros2_control running.
"""
import sys

if len(sys.argv) > 1 and sys.argv[1] in ('-h', '--help'):
    print(__doc__.strip())
    sys.exit(0)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from control_msgs.msg import SteeringControllerStatus
import time
import math


def main():
    rclpy.init()
    node = Node('motion_test')

    odom_data = {'x': None, 'y': None, 'init_x': None, 'init_y': None}
    clock_time = {'sec': 0, 'nanosec': 0}
    ctrl_state = {'linear_vel': None, 'steering': None}

    def odom_cb(msg):
        odom_data['x'] = msg.pose.pose.position.x
        odom_data['y'] = msg.pose.pose.position.y

    def clock_cb(msg):
        clock_time['sec'] = msg.clock.sec
        clock_time['nanosec'] = msg.clock.nanosec

    def ctrl_cb(msg):
        ctrl_state['linear_vel'] = msg.linear_velocity_command
        ctrl_state['steering'] = msg.steering_angle_command

    node.create_subscription(Odometry, '/ackermann/odom', odom_cb, 10)
    node.create_subscription(Clock, '/clock', clock_cb, 10)
    node.create_subscription(
        SteeringControllerStatus,
        '/ackermann_steering_controller/controller_state',
        ctrl_cb, 10
    )

    # Wait for initial odom
    end = time.time() + 3
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    odom_data['init_x'] = odom_data['x']
    odom_data['init_y'] = odom_data['y']
    print(f"Initial odom: x={odom_data['x']}, y={odom_data['y']}")
    print(f"Clock: {clock_time['sec']}.{clock_time['nanosec']}")

    # Publish cmd_vel for 5 seconds
    pub = node.create_publisher(TwistStamped, '/cmd_vel', 10)

    start = time.time()
    count = 0
    while time.time() - start < 5:
        rclpy.spin_once(node, timeout_sec=0.01)
        msg = TwistStamped()
        msg.header.stamp.sec = clock_time['sec']
        msg.header.stamp.nanosec = clock_time['nanosec']
        msg.header.frame_id = 'ackermann/base_link'
        msg.twist.linear.x = 1.0
        msg.twist.angular.z = 0.0
        pub.publish(msg)
        count += 1
        time.sleep(0.05)

    # Print results
    print(f"Published {count} messages")
    print(f"Controller state: vel_cmd={ctrl_state['linear_vel']}, steer={ctrl_state['steering']}")
    print(f"Final odom: x={odom_data['x']}, y={odom_data['y']}")

    if odom_data['init_x'] is not None and odom_data['x'] is not None:
        dx = odom_data['x'] - odom_data['init_x']
        dy = odom_data['y'] - odom_data['init_y']
        dist = math.sqrt(dx**2 + dy**2)
        print(f"Distance traveled: {dist:.4f} m")
        if dist > 0.05:
            print("SUCCESS: Robot is moving!")
        else:
            print("FAILURE: Robot did not move")
    else:
        print("FAILURE: No odom data received")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
