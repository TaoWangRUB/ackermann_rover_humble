#!/usr/bin/env python3
"""Launch Nav2 stack tailored for the Ackermann rover."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('nav2_bringup')
    default_params = os.path.join(bringup_share, 'config', 'nav2_ackermann.yaml')
    bt_share = get_package_share_directory('nav2_bt_navigator')

    default_bt_xml = os.path.join(bt_share, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    default_nav_through_bt = os.path.join(
        bt_share, 'behavior_trees', 'navigate_through_poses_w_replanning_and_recovery.xml'
    )

    params_file = LaunchConfiguration('params_file')
    bt_xml = LaunchConfiguration('bt_xml')
    nav_through_bt = LaunchConfiguration('navigate_through_poses_bt')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the Nav2 parameter file.'
    )

    declare_bt_xml = DeclareLaunchArgument(
        'bt_xml',
        default_value=default_bt_xml,
        description='Behavior Tree XML used by bt_navigator for NavigateToPose.'
    )

    declare_nav_through_bt = DeclareLaunchArgument(
        'navigate_through_poses_bt',
        default_value=default_nav_through_bt,
        description='Behavior Tree XML used for NavigateThroughPoses.'
    )

    nav2_remaps = [('odom', '/odometry/filtered')]

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=nav2_remaps,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
        remappings=nav2_remaps,
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file],
        remappings=nav2_remaps,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
        remappings=nav2_remaps,
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            params_file,
            {
                'default_bt_xml_filename': bt_xml,
                'nav_through_poses_bt_xml': nav_through_bt,
            },
        ],
        remappings=nav2_remaps,
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file],
        remappings=nav2_remaps,
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file],
        remappings=nav2_remaps,
    )

    lifecycle_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_nav2',
        output='screen',
        parameters=[params_file],
        remappings=nav2_remaps,
    )

    return LaunchDescription(
        [
            declare_params_file,
            declare_bt_xml,
            declare_nav_through_bt,
            controller_server,
            planner_server,
            smoother_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            lifecycle_nav,
        ]
    )
