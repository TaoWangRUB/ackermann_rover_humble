#!/usr/bin/env python3
"""Launch VINS-Fusion and adapt its raw odometry into the rover frame contract."""

import platform
from glob import glob
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('vins_fusion_bringup'),
            'config',
            't265_stereo_fisheye_imu.yaml',
        ]),
        description='VINS-Fusion estimator config file.',
    ),
    DeclareLaunchArgument(
        'raw_odom_topic',
        default_value='/vins/raw_odometry',
        description='Raw VINS odometry topic before rover-frame adaptation.',
    ),
    DeclareLaunchArgument(
        'odom_topic',
        default_value='/vins_odom',
        description='Adapted VINS odometry topic exposed to the rest of the stack.',
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
        description='Use simulation time for the odom relay node.',
    ),
    DeclareLaunchArgument(
        'opencv_prefix',
        default_value=f'/workspace/docker_cache/opencv-cuda/{platform.machine()}/current',
        description='Prefix containing the optional CUDA-enabled OpenCV runtime for vins.',
    ),
]


def detect_wsl_cuda_dir() -> str:
    if not Path('/dev/dxg').exists():
        return ''

    candidates = sorted(glob('/usr/lib/wsl/drivers/*/libcuda.so.1'))
    if not candidates:
        return ''

    return str(Path(candidates[0]).resolve().parent)


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration('config_file')
    raw_odom_topic = LaunchConfiguration('raw_odom_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    base_frame = LaunchConfiguration('base_frame')
    output_frame = LaunchConfiguration('output_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')
    opencv_prefix = LaunchConfiguration('opencv_prefix')
    wsl_cuda_dir = detect_wsl_cuda_dir()

    ld_library_path = []
    if wsl_cuda_dir:
        ld_library_path.extend([wsl_cuda_dir, ':'])
    ld_library_path.extend([
        PathJoinSubstitution([opencv_prefix, 'lib']),
        ':',
        EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
    ])

    vins_node = Node(
        package='vins',
        executable='vins_node',
        name='vins_estimator',
        output='screen',
        arguments=[config_file],
        additional_env={
            'LD_LIBRARY_PATH': ld_library_path,
        },
        remappings=[
            ('odometry', raw_odom_topic),
            ('margin_cloud', '/vins/margin_cloud'),
            ('keyframe_pose', '/vins/keyframe_pose'),
            ('keyframe_point', '/vins/keyframe_point'),
            ('extrinsic', '/vins/extrinsic'),
        ],
    )

    odom_relay = Node(
        package='realsense_camera_bringup',
        executable='odom_tf_relay',
        name='vins_odom_relay',
        output='screen',
        parameters=[{
            'input_topic': raw_odom_topic,
            'output_topic': odom_topic,
            'base_frame': base_frame,
            'output_frame': output_frame,
            'publish_tf': True,
            'use_sim_time': use_sim_time,
        }],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(vins_node)
    ld.add_action(odom_relay)
    return ld
