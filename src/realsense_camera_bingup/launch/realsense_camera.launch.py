from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('camera_name', default_value='camera',
                          description='Camera namespace and TF frame prefix'),
    DeclareLaunchArgument('serial_no', default_value='',
                          description='Device USB serial number (empty = first found)'),
    DeclareLaunchArgument('camera_model', default_value='d435i',
                          choices=['d435i', 'l515', 't265'],
                          description='Camera hardware model'),
    # D435i / L515
    DeclareLaunchArgument('enable_color', default_value='true'),
    DeclareLaunchArgument('enable_depth', default_value='true'),
    DeclareLaunchArgument('enable_imu',   default_value='false',
                          description='IMU streams — D435i and L515'),
    DeclareLaunchArgument('color_width',  default_value='640'),
    DeclareLaunchArgument('color_height', default_value='480'),
    DeclareLaunchArgument('color_fps',    default_value='30'),
    DeclareLaunchArgument('depth_width',  default_value='640'),
    DeclareLaunchArgument('depth_height', default_value='480'),
    DeclareLaunchArgument('depth_fps',    default_value='30'),
    DeclareLaunchArgument('align_depth_to_color', default_value='false'),
    # T265
    DeclareLaunchArgument('fisheye_fps', default_value='30',
                          description='Fisheye frame rate — T265 only'),
]


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('realsense_camera_bingup')
    params_file = str(Path(pkg_share) / 'config' / 'realsense_params.yaml')

    node = Node(
        package='realsense_camera_bingup',
        executable='realsense_camera_node',
        name='realsense_camera_node',
        output='screen',
        parameters=[
            params_file,
            {
                'camera_name':           LaunchConfiguration('camera_name'),
                'serial_no':             LaunchConfiguration('serial_no'),
                'camera_model':          LaunchConfiguration('camera_model'),
                'enable_color':          LaunchConfiguration('enable_color'),
                'enable_depth':          LaunchConfiguration('enable_depth'),
                'enable_imu':            LaunchConfiguration('enable_imu'),
                'color_width':           LaunchConfiguration('color_width'),
                'color_height':          LaunchConfiguration('color_height'),
                'color_fps':             LaunchConfiguration('color_fps'),
                'depth_width':           LaunchConfiguration('depth_width'),
                'depth_height':          LaunchConfiguration('depth_height'),
                'depth_fps':             LaunchConfiguration('depth_fps'),
                'align_depth_to_color':  LaunchConfiguration('align_depth_to_color'),
                'fisheye_fps':           LaunchConfiguration('fisheye_fps'),
            },
        ],
    )

    return LaunchDescription(ARGUMENTS + [node])
