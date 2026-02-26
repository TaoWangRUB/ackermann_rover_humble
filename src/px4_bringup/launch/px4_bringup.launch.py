from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Note: px4_offboard_control and px4_vision_odom are the active bridge nodes.
    #       Add remappings / parameters below once the interfaces are finalised.

    px4_bridge_node = Node(
        package='px4_bringup',
        executable='px4_offboard_control.py',
        name='px4_offboard_control',
        output='screen',
        parameters=[
        ],
        remappings=[
        ]
    )

    px4_odom_node = Node(
        package='px4_bringup',
        executable='px4_vision_odom.py',
        name='px4_odometry',
        output='screen'
        # Note: /odom maps to /fmu/in/vehicle_odometry natively in the node.
        # If the expected odometry topic is /odometry/filtered, remap it here:
        # remappings=[('/odom', '/odometry/filtered')]
    )

    return LaunchDescription([
        px4_bridge_node,
        px4_odom_node
    ])
