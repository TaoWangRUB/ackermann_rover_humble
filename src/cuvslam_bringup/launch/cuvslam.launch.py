#!/usr/bin/env python3
"""Launch cuVSLAM stereo-fisheye VIO and adapt its raw odometry to the rover frame."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('cuvslam_bringup'),
            'config',
            't265_stereo_fisheye.yaml',
        ]),
        description='cuVSLAM wrapper config file.',
    ),
    DeclareLaunchArgument(
        'raw_odom_topic',
        default_value='/cuvslam/raw_odometry',
        description='Raw cuVSLAM odometry topic before rover-frame adaptation.',
    ),
    DeclareLaunchArgument(
        'odom_topic',
        default_value='/cuvslam_odom',
        description='Adapted cuVSLAM odometry topic exposed to the rest of the stack.',
    ),
    DeclareLaunchArgument(
        'base_frame',
        default_value='ackermann/base_link',
        description='Target child_frame_id for the adapted odometry.',
    ),
    DeclareLaunchArgument(
        'output_frame',
        default_value='odom',
        description='Target frame_id for the adapted odometry.',
    ),
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Use simulation time for the cuVSLAM node and odom relay.',
    ),
    DeclareLaunchArgument(
        'cuvslam_library_dir',
        default_value='/workspace/src/cuVSLAM/build/bin',
        description='Directory containing libcuvslam.so; prepended to LD_LIBRARY_PATH.',
    ),
]


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration('config_file')
    raw_odom_topic = LaunchConfiguration('raw_odom_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    base_frame = LaunchConfiguration('base_frame')
    output_frame = LaunchConfiguration('output_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')
    cuvslam_library_dir = LaunchConfiguration('cuvslam_library_dir')

    cuvslam_node = Node(
        package='cuvslam_bringup',
        executable='cuvslam_odom_node',
        name='cuvslam_odom_node',
        output='screen',
        additional_env={
            'LD_LIBRARY_PATH': [
                cuvslam_library_dir,
                ':',
                EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
            ],
        },
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'raw_odom_topic': raw_odom_topic,
                'odom_frame_id': output_frame,
                'rig_frame_id': base_frame,
            },
        ],
    )

    # Adapt raw cuVSLAM odometry into the rover's odom -> ackermann/base_link
    # contract using the shared odom_tf_relay executable from
    # realsense_camera_bringup.
    odom_relay = Node(
        package='realsense_camera_bringup',
        executable='odom_tf_relay',
        name='cuvslam_odom_relay',
        output='screen',
        parameters=[{
            'input_topic': raw_odom_topic,
            'output_topic': odom_topic,
            'base_frame': base_frame,
            'output_frame': output_frame,
            'publish_tf': False,
            'use_sim_time': use_sim_time,
        }],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(cuvslam_node)
    ld.add_action(odom_relay)
    return ld
