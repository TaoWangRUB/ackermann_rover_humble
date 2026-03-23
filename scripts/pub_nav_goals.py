#!/usr/bin/env python3
"""Publish Nav2 NavigateToPose goals to verify Ackermann planner + controller changes.

Tests:
  1. Forward goal  — 2 m ahead (baseline, should always work)
  2. Lateral goal  — 2 m to the left (requires turning)
  3. Behind goal   — 2 m behind (was freezing before SmacPlannerHybrid + PathAngleCritic mode:2)

Usage (from workspace root, after sourcing install/setup.bash):
  python3 scripts/pub_nav_goals.py [--timeout 30] [--test forward|lateral|behind|all]
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def euler_to_quat(yaw):
    """Return (x, y, z, w) quaternion for a pure yaw rotation."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)


def make_goal(x, y, yaw, frame='map'):
    goal_msg = NavigateToPose.Goal()
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = x
    pose.pose.position.y = y
    qx, qy, qz, qw = euler_to_quat(yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    goal_msg.pose = pose
    return goal_msg


class NavGoalTester(Node):
    def __init__(self, timeout: float):
        super().__init__('nav_goal_tester')
        self._timeout = timeout
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._odom = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)

    def _odom_cb(self, msg):
        self._odom['x'] = msg.pose.pose.position.x
        self._odom['y'] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # yaw from quaternion
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom['yaw'] = math.atan2(siny, cosy)

    def _wait_for_odom(self, secs=3.0):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def send_goal(self, label: str, x: float, y: float, yaw: float) -> bool:
        self.get_logger().info(f'[{label}] Waiting for navigate_to_pose action server...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available')
            return False

        goal = make_goal(x, y, yaw)
        self.get_logger().info(
            f'[{label}] Sending goal → x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}°'
        )

        send_future = self._client.send_goal_async(goal)
        start = time.time()
        while not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > 10.0:
                self.get_logger().error(f'[{label}] Goal acceptance timed out')
                return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'[{label}] Goal REJECTED')
            return False
        self.get_logger().info(f'[{label}] Goal accepted, waiting for result...')

        result_future = goal_handle.get_result_async()
        start = time.time()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.time() - start
            if elapsed > self._timeout:
                self.get_logger().error(
                    f'[{label}] TIMEOUT after {self._timeout:.0f}s — rover likely froze'
                )
                goal_handle.cancel_goal_async()
                return False

        result = result_future.result()
        # NavigateToPose result error_code 0 = success
        success = result.result.error_code == 0
        status = 'PASS' if success else f'FAIL (error_code={result.result.error_code})'
        self.get_logger().info(f'[{label}] {status}')
        return success

    def run_tests(self, tests: list) -> dict[str, bool]:
        self._wait_for_odom()
        ox, oy, oyaw = self._odom['x'], self._odom['y'], self._odom['yaw']
        self.get_logger().info(
            f'Robot pose: x={ox:.2f} y={oy:.2f} yaw={math.degrees(oyaw):.1f}°'
        )

        # Goal offsets are in the robot's current frame → convert to map frame
        def robot_to_map(dx, dy, goal_yaw_offset):
            mx = ox + dx * math.cos(oyaw) - dy * math.sin(oyaw)
            my = oy + dx * math.sin(oyaw) + dy * math.cos(oyaw)
            return mx, my, oyaw + goal_yaw_offset

        scenarios = {
            'forward': robot_to_map(2.0,  0.0,  0.0),   # 2 m ahead, same heading
            'lateral': robot_to_map(0.0,  2.0,  1.57),  # 2 m left, face left
            'behind':  robot_to_map(-2.0, 0.0,  3.14),  # 2 m behind, face away
        }

        results = {}
        for name in tests:
            x, y, yaw = scenarios[name]
            results[name] = self.send_goal(name, x, y, yaw)
            # Brief pause between goals
            time.sleep(2.0)

        return results


def main():
    parser = argparse.ArgumentParser(description='Nav2 goal verification for Ackermann rover')
    parser.add_argument(
        '--test',
        choices=['forward', 'lateral', 'behind', 'all'],
        default='all',
        help='Which test(s) to run (default: all)',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=30.0,
        help='Per-goal timeout in seconds (default: 30)',
    )
    args = parser.parse_args()

    tests = ['forward', 'lateral', 'behind'] if args.test == 'all' else [args.test]

    rclpy.init()
    node = NavGoalTester(timeout=args.timeout)

    try:
        results = node.run_tests(tests)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print('\n--- Results ---')
    all_passed = True
    for name, passed in results.items():
        status = 'PASS' if passed else 'FAIL'
        print(f'  {name:10s}: {status}')
        if not passed:
            all_passed = False

    sys.exit(0 if all_passed else 1)


if __name__ == '__main__':
    main()
