#!/usr/bin/env python3
"""Bring-up launch that composes Gazebo and RTAB-Map for the rover."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use the Gazebo simulation clock for all nodes.'
    ),
    DeclareLaunchArgument(
        'gazebo_args',
        default_value=TextSubstitution(
            text=f"-r {os.path.join(get_package_share_directory('robot_description'), 'worlds', 'warehouse.sdf')}"
        ),
        description='Arguments passed to gz sim (e.g., "-v 4 -r custom.world").'
    ),
    DeclareLaunchArgument(
        'robot_name',
        default_value='ackmann',
        description='Name of the Gazebo entity and TF prefix.'
    ),
    DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Optional ROS namespace applied to spawned nodes.'
    ),
    DeclareLaunchArgument('x', default_value='0.0', description='Initial X position in meters.'),
    DeclareLaunchArgument('y', default_value='0.0', description='Initial Y position in meters.'),
    DeclareLaunchArgument('z', default_value='0.1', description='Initial Z position in meters.'),
    DeclareLaunchArgument(
        'vision',
        default_value='true',
        choices=['true', 'false'],
        description='Use RGB-D visual odometry (true) or fall back to ICP (false).'
    ),
    DeclareLaunchArgument(
        'localization',
        default_value='false',
        choices=['true', 'false'],
        description='Launch RTAB-Map in localization-only mode.'
    ),
    DeclareLaunchArgument(
        'rtabmap_viz',
        default_value='true',
        choices=['true', 'false'],
        description='Start rtabmap_viz for monitoring.'
    ),
]


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    gazebo_args = LaunchConfiguration('gazebo_args')
    robot_name = LaunchConfiguration('robot_name')
    namespace = LaunchConfiguration('namespace')
    x_pos = LaunchConfiguration('x')
    y_pos = LaunchConfiguration('y')
    z_pos = LaunchConfiguration('z')
    vision = LaunchConfiguration('vision')
    localization = LaunchConfiguration('localization')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')

    robot_description_share = get_package_share_directory('robot_description')
    rtabmap_bringup_share = get_package_share_directory('rtabmap_bringup')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_description_share, 'launch', 'gazebo_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gz_args': gazebo_args,
            'robot_name': robot_name,
            'namespace': namespace,
            'x': x_pos,
            'y': y_pos,
            'z': z_pos,
        }.items()
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_bringup_share, 'launch', 'rtabmap_slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'vision': vision,
            'localization': localization,
            'rtabmap_viz': rtabmap_viz,
        }.items()
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo_launch)
    ld.add_action(rtabmap_launch)

    return ld
