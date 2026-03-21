import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_px4_bringup = get_package_share_directory('px4_bringup')
    config_file = os.path.join(pkg_px4_bringup, 'config', 'px4_bridge.yaml')

    # ── ESC type ──────────────────────────────────────────────────────────
    reversible_drive_arg = DeclareLaunchArgument(
        'reversible_drive',
        default_value='false',
        choices=['true', 'false'],
        description=(
            'Bidirectional ESC: true = throttle [-1,1], false = throttle [0,1]'
        ),
    )

    # ── Mode selection ────────────────────────────────────────────────────
    enable_mode_node_arg = DeclareLaunchArgument(
        'enable_mode_node',
        default_value='true',
        description='Launch the PX4 custom mode node (set false for VO-bridge-only use)'
    )

    mode_type_arg = DeclareLaunchArgument(
        'mode_type',
        default_value='manual',
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
        default_value='cubepilot_link',
        description='TF frame for the IMU/flight controller (body FLU)'
    )

    # ── Odometry transport selection ──────────────────────────────────────
    odom_transport_arg = DeclareLaunchArgument(
        'odometry_transport',
        default_value='xrce',
        description="Transport for sending odometry to PX4: 'xrce' or 'mavlink'"
    )

    mav_device_arg = DeclareLaunchArgument(
        'mavlink_device',
        default_value='/dev/ttyACM0',
        description='MAVLink serial device for MAVLink bridge (mavlink transport)'
    )

    mav_baud_arg = DeclareLaunchArgument(
        'mavlink_baud',
        default_value='57600',
        description='Baud rate for MAVLink bridge'
    )

    mav_rate_arg = DeclareLaunchArgument(
        'mavlink_rate',
        default_value='20',
        description='Publish rate (Hz) for MAVLink bridge'
    )

    # ── Vehicle odometry bridge ───────────────────────────────────────────
    enable_vehicle_odometry_arg = DeclareLaunchArgument(
        'enable_vehicle_odometry',
        default_value='true',
        description='Launch px4_vehicle_odometry.py to convert /fmu/out/vehicle_odometry → /px4_vehicle_odom'
    )

    vehicle_odom_frame_arg = DeclareLaunchArgument(
        'vehicle_odom_frame',
        default_value='vehicle_odom',
        description='frame_id for the published vehicle odometry (ENU world-fixed)'
    )

    vehicle_odom_child_frame_arg = DeclareLaunchArgument(
        'vehicle_odom_child_frame',
        default_value='cubepilot_link',
        description='child_frame_id for the published vehicle odometry (matches URDF cubepilot_link)'
    )

    # ── VO bridge: XRCE transport ─────────────────────────────────────────
    vo_xrce_node = Node(
        package='px4_bringup',
        executable='px4_vision_odom.py',
        name='px4_vision_odom',
        output='screen',
        parameters=[{
            'odom_topic': LaunchConfiguration('odom_topic'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
        }],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('enable_vo_bridge'), "' == 'true'",
            " and '", LaunchConfiguration('odometry_transport'), "' == 'xrce'",
        ])),
    )

    # ── VO bridge: MAVLink transport ──────────────────────────────────────
    vo_mavlink_node = Node(
        package='px4_bringup',
        executable='px4_mavlink_vpe.py',
        name='px4_mavlink_vpe',
        output='screen',
        arguments=['--topic', LaunchConfiguration('odom_topic')],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('enable_vo_bridge'), "' == 'true'",
            " and '", LaunchConfiguration('odometry_transport'), "' == 'mavlink'",
        ])),
    )

    # ── Vehicle odometry bridge ───────────────────────────────────────────
    vehicle_odom_node = Node(
        package='px4_bringup',
        executable='px4_vehicle_odometry.py',
        name='px4_vehicle_odometry',
        output='screen',
        parameters=[{
            'frame_id':       LaunchConfiguration('vehicle_odom_frame'),
            'child_frame_id': LaunchConfiguration('vehicle_odom_child_frame'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_vehicle_odometry')),
    )

    # ── Mode nodes (C++ custom modes via px4_ros2_interface_lib) ─────────
    trajectory_node = Node(
        package='px4_bringup',
        executable='offboard_trajectory_mode',
        name='offboard_trajectory_mode',
        output='screen',
        parameters=[config_file],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('enable_mode_node'), "' == 'true'",
            " and '", LaunchConfiguration('mode_type'), "' == 'trajectory'",
        ])),
    )

    speed_steering_node = Node(
        package='px4_bringup',
        executable='rover_speed_steering_mode',
        name='rover_speed_steering_mode',
        output='screen',
        parameters=[config_file],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('enable_mode_node'), "' == 'true'",
            " and '", LaunchConfiguration('mode_type'), "' == 'speed_steering'",
        ])),
    )

    speed_attitude_node = Node(
        package='px4_bringup',
        executable='rover_speed_attitude_mode',
        name='rover_speed_attitude_mode',
        output='screen',
        parameters=[config_file],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('enable_mode_node'), "' == 'true'",
            " and '", LaunchConfiguration('mode_type'), "' == 'speed_attitude'",
        ])),
    )

    manual_node = Node(
        package='px4_bringup',
        executable='rover_manual_mode',
        name='rover_manual_mode',
        output='screen',
        parameters=[
            config_file,
            {'bidirectional_esc': ParameterValue(
                LaunchConfiguration('reversible_drive'), value_type=bool)},
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('enable_mode_node'), "' == 'true'",
            " and '", LaunchConfiguration('mode_type'), "' == 'manual'",
        ])),
    )

    return LaunchDescription([
        reversible_drive_arg,
        enable_mode_node_arg,
        mode_type_arg,
        enable_vo_arg,
        odom_topic_arg,
        odom_frame_arg,
        base_frame_arg,
        odom_transport_arg,
        mav_device_arg,
        mav_baud_arg,
        mav_rate_arg,
        enable_vehicle_odometry_arg,
        vehicle_odom_frame_arg,
        vehicle_odom_child_frame_arg,
        LogInfo(msg="Launching PX4 bringup (mode + VO bridge)..."),
        vo_xrce_node,
        vo_mavlink_node,
        vehicle_odom_node,
        trajectory_node,
        speed_steering_node,
        speed_attitude_node,
        manual_node,
    ])
