#!/usr/bin/env python3
"""Bring-up launch that composes Gazebo and RTAB-Map for the rover."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

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
            text=f"-r {os.path.join(get_package_share_directory('description_robot'), 'worlds', 'warehouse.sdf')}"
        ),
        description='Arguments passed to gz sim (e.g., "-v 4 -r custom.world").'
    ),
    DeclareLaunchArgument(
        'robot_name',
            default_value='ackermann',
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
        'rtabmap',
        default_value='false',
        choices=['true', 'false'],
        description='Launch RTAB-Map once Gazebo is running.'
    ),
    DeclareLaunchArgument(
        'rtabmap_viz',
        default_value='false',
        choices=['true', 'false'],
        description='Start rtabmap_viz for monitoring.'
    ),
    DeclareLaunchArgument(
        'nav2',
        default_value='false',
        choices=['true', 'false'],
        description='Launch the Nav2 stack once localization is running.'
    ),
    DeclareLaunchArgument(
        'nav2_params_file',
        default_value=TextSubstitution(
            text=os.path.join(
                get_package_share_directory('ackermann_nav2_bringup'), 'config', 'nav2_params.yaml'
            )
        ),
        description='Parameter file used to configure Nav2 components.'
    ),
    DeclareLaunchArgument(
        'nav2_bt_xml',
        default_value=TextSubstitution(
            text=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees',
                'navigate_w_replanning_and_recovery.xml'
            )
        ),
        description='Behavior Tree XML file for NavigateToPose.'
    ),
    DeclareLaunchArgument(
        'nav2_through_poses_bt',
        default_value=TextSubstitution(
            text=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees',
                'navigate_through_poses_w_replanning_and_recovery.xml'
            )
        ),
        description='Behavior Tree XML file for NavigateThroughPoses.'
    ),
    DeclareLaunchArgument(
        'rviz_config',
        default_value=TextSubstitution(
            text=os.path.join(
                get_package_share_directory('robot_bringup'),
                'rviz',
                'robot.rviz'
            )
        ),
        description='RViz configuration file to load at startup.'
    ),
    DeclareLaunchArgument(
        'rviz',
        default_value='true',
        choices=['true', 'false'],
        description='When true, start rviz2 with the configured view.'
    ),
    DeclareLaunchArgument(
        'enable_px4_sitl',
        default_value='false',
        choices=['true', 'false'],
        description='Enable PX4 SITL mode (disables ros2_control, enables PX4 joint plugins).'
    ),
    DeclareLaunchArgument(
        'px4_mode_type',
        default_value='speed_steering',
        description='PX4 bridge mode: trajectory, speed_steering, speed_attitude, or manual.'
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
    rtabmap_enable = LaunchConfiguration('rtabmap')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    nav2_enable = LaunchConfiguration('nav2')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    nav2_bt_xml = LaunchConfiguration('nav2_bt_xml')
    nav2_through_bt = LaunchConfiguration('nav2_through_poses_bt')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_enable = LaunchConfiguration('rviz')
    enable_px4_sitl = LaunchConfiguration('enable_px4_sitl')
    px4_mode_type = LaunchConfiguration('px4_mode_type')

    robot_description_share = get_package_share_directory('description_robot')
    rtabmap_bringup_share = get_package_share_directory('rtabmap_bringup')
    nav2_bringup_share = get_package_share_directory('ackermann_nav2_bringup')

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
            'enable_px4_sitl': enable_px4_sitl,
        }.items()
    )

    # NOTE: px4_bridge (rover_speed_steering_mode etc.) is NOT launched here.
    # It must be started AFTER MicroXRCEAgent + PX4 SITL are running:
    #   ros2 launch px4_bringup px4_bringup.launch.py mode_type:=speed_steering

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_bringup_share, 'launch', 'rtabmap_slam.launch.py')
        ),
        condition=IfCondition(rtabmap_enable),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'vision': vision,
            'localization': localization,
            'rtabmap_viz': rtabmap_viz,
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'nav2_bringup.launch.py')
        ),
        condition=IfCondition(nav2_enable),
        launch_arguments={
            'params_file': nav2_params_file,
            'bt_xml': nav2_bt_xml,
            'navigate_through_poses_bt': nav2_through_bt,
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz_enable),
    )

    from launch.actions import TimerAction
    rviz_delayed = TimerAction(
        period=5.0,
        actions=[rviz_node]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo_launch)
    ld.add_action(rtabmap_launch)
    ld.add_action(nav2_launch)
    ld.add_action(rviz_delayed)

    return ld
