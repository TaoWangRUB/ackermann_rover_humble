import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_px4_bringup = get_package_share_directory('px4_bringup')
    
    # Declare launch arguments
    dds_domain_id_arg = DeclareLaunchArgument(
        'dds_domain_id', default_value='0',
        description='ROS 2 / DDS Domain ID for the PX4 bridge'
    )
    px4_uav_id_arg = DeclareLaunchArgument(
        'px4_uav_id', default_value='0',
        description='MAVLink / PX4 UAV ID for the bridge'
    )

    # Note: Since the exact bridge node executable isn't strictly defined yet (e.g. micro_ros_agent),
    #       we will set up a placeholder Node block for the px4-offboard package bridge.
    #       The topics are remapped here to match interfaces.md.
    
    config_file = PathJoinSubstitution([pkg_px4_bringup, 'config', 'px4_bridge.yaml'])

    px4_bridge_node = Node(
        package='px4_bringup',
        executable='px4_bridge_node.py',
        name='px4_dds_bridge',
        output='screen',
        parameters=[
            config_file,
            {'dds_domain_id': LaunchConfiguration('dds_domain_id')},
            {'px4_uav_id': LaunchConfiguration('px4_uav_id')}
        ],
        remappings=[
            # Input from Ackermann control -> Output to bridge
            ('/px4/setpoint/ackermann', '/cmd_ackermann'),
            # Output from bridge -> Input to Ackermann control / Safety
            ('/px4/status', '/px4/status'),
            ('/px4/actuator_feedback', '/px4/actuator_feedback')
        ]
    )

    px4_odom_node = Node(
        package='px4_bringup',
        executable='px4_odometry_node.py',
        name='px4_odometry_bridge',
        output='screen'
        # Note: /odom maps to /fmu/in/vehicle_odometry natively in the node.
        # If the expected odometry topic is /odometry/filtered, remap it here:
        # remappings=[('/odom', '/odometry/filtered')]
    )

    return LaunchDescription([
        dds_domain_id_arg,
        px4_uav_id_arg,
        LogInfo(msg="Launching PX4 DDS Bridge..."),
        px4_bridge_node,
        px4_odom_node
    ])
