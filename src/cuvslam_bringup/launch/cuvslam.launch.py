from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config = os.path.join(
        os.getenv('ROS_PACKAGE_PATH', '/workspace/src/cuvslam_bringup'),
        'config',
        't265_stereo_fisheye.yaml'
    )
    return LaunchDescription([
        Node(
            package='cuvslam_bringup',
            executable='cuvslam_odom_node',
            name='cuvslam_odom_node',
            output='screen',
            parameters=[config],
        ),
    ])
