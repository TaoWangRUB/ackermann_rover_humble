import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context, *args, **kwargs):
    """Launch the PX4 mode node and optionally the VO bridge."""
    mode_type = LaunchConfiguration('mode_type').perform(context)
    enable_vo = LaunchConfiguration('enable_vo_bridge').perform(context)
    odom_topic = LaunchConfiguration('odom_topic').perform(context)
    odom_frame = LaunchConfiguration('odom_frame').perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context)
    odom_transport = LaunchConfiguration('odometry_transport').perform(context)
    mav_device = LaunchConfiguration('mavlink_device').perform(context)
    mav_baud = LaunchConfiguration('mavlink_baud').perform(context)
    mav_rate = LaunchConfiguration('mavlink_rate').perform(context)
    enable_veh_odom = LaunchConfiguration('enable_vehicle_odometry').perform(context)
    veh_odom_frame = LaunchConfiguration('vehicle_odom_frame').perform(context)
    veh_odom_child_frame = LaunchConfiguration('vehicle_odom_child_frame').perform(context)

    nodes = []
    pkg_px4_bringup = get_package_share_directory('px4_bringup')
    config_file = os.path.join(pkg_px4_bringup, 'config', 'px4_bridge.yaml')

    # ── VO Bridge: choose XRCE (px4_vision_odom.py) or MAVLink (px4_mavlink_vpe.py)
    if enable_vo.lower() == 'true':
        if odom_transport.lower() == 'xrce':
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
        elif odom_transport.lower() == 'mavlink':
                # Launch the MAVLink bridge as a package-installed python executable
                # (installed via CMakeLists install(PROGRAMS ...)). Use Node so
                # ros2 launch/ros2 run resolution works in both dev and installed trees.
                nodes.append(
                    Node(
                        package='px4_bringup',
                        executable='px4_mavlink_vpe.py',
                        name='px4_mavlink_vpe',
                        output='screen',
                        # The bridge has sensible defaults; pass topic explicitly.
                        arguments=['--topic', odom_topic],
                    )
                )
        else:
            raise ValueError("odometry_transport must be 'xrce' or 'mavlink'")

    # ── Vehicle odometry bridge: /fmu/out/vehicle_odometry → /px4_vehicle_odom
    if enable_veh_odom.lower() == 'true':
        nodes.append(
            Node(
                package='px4_bringup',
                executable='px4_vehicle_odometry.py',
                name='px4_vehicle_odometry',
                output='screen',
                parameters=[{
                    'frame_id':       veh_odom_frame,
                    'child_frame_id': veh_odom_child_frame,
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
        default_value='ackermann/base_link',
        description='TF frame for the robot base (body FLU)'
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

    return LaunchDescription([
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
        OpaqueFunction(function=_launch_nodes),
    ])
