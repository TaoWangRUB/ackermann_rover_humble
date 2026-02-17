from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ackermann_control')
    params_file = os.path.join(pkg_share, 'params', 'ackermann.yaml')

    controller_node = Node(
        package='ackermann_control',
        executable='ackermann_controller',
        name='ackermann_controller',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([controller_node])