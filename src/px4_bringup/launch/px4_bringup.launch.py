import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context, *args, **kwargs):
    """Launch the PX4 mode node and optionally the VO bridge."""
    mode_type = LaunchConfiguration('mode_type').perform(context)
    enable_vo = LaunchConfiguration('enable_vo_bridge').perform(context)
    odom_topic = LaunchConfiguration('odom_topic').perform(context)
    odom_frame = LaunchConfiguration('odom_frame').perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context)

    nodes = []
    pkg_px4_bringup = get_package_share_directory('px4_bringup')
    config_file = os.path.join(pkg_px4_bringup, 'config', 'px4_bridge.yaml')

    # ── VO Bridge (px4_vision_odom.py → /fmu/in/vehicle_visual_odometry) ──
    if enable_vo.lower() == 'true':
        nodes.append(
            Node(
                package='px4_bringup',
                executable='px4_vision_odom.py',
                name='px4_vision_odom',
                output='screen',
                parameters=[{
                    'odom_topic': odom_topic,
                    'odom_frame': odom_frame,
                    'base_frame': base_frame,
                }],
            )
        )

    # ── Mode node (C++ custom mode via px4_ros2_interface_lib) ────────────
    mode_executables = {
        'trajectory': 'offboard_trajectory_mode',
        'speed_steering': 'rover_speed_steering_mode',
        'speed_attitude': 'rover_speed_attitude_mode',
        'manual': 'rover_manual_mode',
    }

    executable = mode_executables.get(mode_type)
    if executable is None:
        raise ValueError(
            f"Unknown mode_type '{mode_type}'. "
            f"Valid options: {list(mode_executables.keys())}"
        )

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
    # ── Mode selection ────────────────────────────────────────────────────
    mode_type_arg = DeclareLaunchArgument(
        'mode_type',
        default_value='speed_steering',
        description='PX4 mode type: trajectory, speed_steering, speed_attitude, or manual'
    )

    # ── VO bridge ─────────────────────────────────────────────────────────
    enable_vo_arg = DeclareLaunchArgument(
        'enable_vo_bridge',
        default_value='false',
        description='Launch px4_vision_odom.py to feed VO into PX4 EKF2'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odometry/filtered',
        description='Odometry topic for the VO bridge (ENU/FLU nav_msgs/Odometry)'
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='TF frame for the odom reference (world ENU)'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='ackermann/base_link',
        description='TF frame for the robot base (body FLU)'
    )

    return LaunchDescription([
        mode_type_arg,
        enable_vo_arg,
        odom_topic_arg,
        odom_frame_arg,
        base_frame_arg,
        LogInfo(msg="Launching PX4 bringup (mode + VO bridge)..."),
        OpaqueFunction(function=_launch_nodes),
    ])
