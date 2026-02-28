import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def _launch_mode_node(context, *args, **kwargs):
    """Select and launch the appropriate C++ mode node based on mode_type argument."""
    mode_type = LaunchConfiguration('mode_type').perform(context)
    use_legacy = LaunchConfiguration('use_legacy_bridge').perform(context)

    nodes = []

    # Always launch the odometry bridge
    nodes.append(
        Node(
            package='px4_bringup',
            executable='px4_odometry_node.py',
            name='px4_odometry_bridge',
            output='screen',
        )
    )

    if use_legacy.lower() == 'true':
        # Legacy Python offboard bridge
        pkg_px4_bringup = get_package_share_directory('px4_bringup')
        config_file = os.path.join(pkg_px4_bringup, 'config', 'px4_bridge.yaml')
        nodes.append(
            Node(
                package='px4_bringup',
                executable='px4_bridge_node.py',
                name='px4_dds_bridge',
                output='screen',
                parameters=[config_file],
            )
        )
    else:
        # C++ custom mode via px4_ros2_interface_lib
        mode_executables = {
            'trajectory': 'offboard_trajectory_mode',
            'speed_steering': 'rover_speed_steering_mode',
            'speed_attitude': 'rover_speed_attitude_mode',
        }

        executable = mode_executables.get(mode_type)
        if executable is None:
            raise ValueError(
                f"Unknown mode_type '{mode_type}'. "
                f"Valid options: {list(mode_executables.keys())}"
            )

        pkg_px4_bringup = get_package_share_directory('px4_bringup')
        config_file = os.path.join(pkg_px4_bringup, 'config', 'px4_bridge.yaml')
        nodes.append(
            Node(
                package='px4_bringup',
                executable=executable,
                name=executable,
                output='screen',
                parameters=[config_file],
            )
        )

    return nodes


def generate_launch_description():
    # Declare launch arguments
    mode_type_arg = DeclareLaunchArgument(
        'mode_type',
        default_value='speed_steering',
        description='PX4 mode type: trajectory, speed_steering, or speed_attitude'
    )

    use_legacy_arg = DeclareLaunchArgument(
        'use_legacy_bridge',
        default_value='false',
        description='Use legacy Python offboard bridge instead of C++ custom modes'
    )

    return LaunchDescription([
        mode_type_arg,
        use_legacy_arg,
        LogInfo(msg="Launching PX4 Bridge..."),
        OpaqueFunction(function=_launch_mode_node),
    ])
