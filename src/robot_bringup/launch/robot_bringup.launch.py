#!/usr/bin/env python3
"""Bring-up launch that composes Gazebo (or real hardware) and RTAB-Map for the rover.

Modes
-----
Simulation (default, L515):
  ros2 launch robot_bringup robot_bringup.launch.py

Simulation with D435i:
  ros2 launch robot_bringup robot_bringup.launch.py depth_camera:=d435i

Hardware — L515 (default):
  ros2 launch robot_bringup robot_bringup.launch.py \\
      use_gazebo:=false use_sim_time:=false rtabmap:=true

Hardware — D435i + T265:
  ros2 launch robot_bringup robot_bringup.launch.py \\
      use_gazebo:=false use_sim_time:=false \\
      depth_camera:=d435i hw_enable_t265:=true rtabmap:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_gazebo',
        default_value='true',
        choices=['true', 'false'],
        description=(
            'true  — Gazebo simulation: sensors come from the sim bridge. '
            'false — Hardware mode: sensors come from realsense_camera_bringup; '
            'also set use_sim_time:=false.'
        ),
    ),
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use the Gazebo simulation clock. Set to false with use_gazebo:=false.'
    ),
    DeclareLaunchArgument(
        'gazebo_args',
        default_value=TextSubstitution(
            text=f"-r {os.path.join(get_package_share_directory('description_robot'), 'worlds', 'warehouse.sdf')}"
        ),
        description='Arguments passed to gz sim (e.g., "-v 4 -r custom.world").'
    ),
    DeclareLaunchArgument(
        'robot_name',
            default_value='ackermann',
        description='Name of the Gazebo entity and TF prefix.'
    ),
    DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Optional ROS namespace applied to spawned nodes.'
    ),
    DeclareLaunchArgument('x', default_value='0.0', description='Initial X position in meters.'),
    DeclareLaunchArgument('y', default_value='0.0', description='Initial Y position in meters.'),
    DeclareLaunchArgument('z', default_value='0.1', description='Initial Z position in meters.'),
    DeclareLaunchArgument(
        'vision',
        default_value='true',
        choices=['true', 'false'],
        description='Use RGB-D visual odometry (true) or fall back to ICP (false).'
    ),
    DeclareLaunchArgument(
        'localization',
        default_value='false',
        choices=['true', 'false'],
        description='Launch RTAB-Map in localization-only mode.'
    ),
    DeclareLaunchArgument(
        'rtabmap',
        default_value='false',
        choices=['true', 'false'],
        description='Launch RTAB-Map once Gazebo is running.'
    ),
    DeclareLaunchArgument(
        'rtabmap_viz',
        default_value='false',
        choices=['true', 'false'],
        description='Start rtabmap_viz for monitoring.'
    ),
    DeclareLaunchArgument(
        'nav2',
        default_value='false',
        choices=['true', 'false'],
        description='Launch the Nav2 stack once localization is running.'
    ),
    DeclareLaunchArgument(
        'nav2_params_file',
        default_value=TextSubstitution(
            text=os.path.join(
                get_package_share_directory('ackermann_nav2_bringup'), 'config', 'nav2_params.yaml'
            )
        ),
        description='Parameter file used to configure Nav2 components.'
    ),
    DeclareLaunchArgument(
        'nav2_bt_xml',
        default_value=TextSubstitution(
            text=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees',
                'navigate_to_pose_w_replanning_and_recovery.xml'
            )
        ),
        description='Behavior Tree XML file for NavigateToPose.'
    ),
    DeclareLaunchArgument(
        'nav2_through_poses_bt',
        default_value=TextSubstitution(
            text=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees',
                'navigate_through_poses_w_replanning_and_recovery.xml'
            )
        ),
        description='Behavior Tree XML file for NavigateThroughPoses.'
    ),
    DeclareLaunchArgument(
        'rviz_config',
        default_value=TextSubstitution(
            text=os.path.join(
                get_package_share_directory('robot_bringup'),
                'rviz',
                'robot.rviz'
            )
        ),
        description='RViz configuration file to load at startup.'
    ),
    DeclareLaunchArgument(
        'rviz',
        default_value='true',
        choices=['true', 'false'],
        description='When true, start rviz2 with the configured view.'
    ),
    DeclareLaunchArgument(
        'enable_px4_sitl',
        default_value='false',
        choices=['true', 'false'],
        description='Enable PX4 SITL mode (disables ros2_control, enables PX4 joint plugins).'
    ),
    DeclareLaunchArgument(
        'px4_mode_type',
        default_value='speed_steering',
        description='PX4 bridge mode: trajectory, speed_steering, speed_attitude, or manual.'
    ),
    DeclareLaunchArgument(
        'reversible_drive',
        default_value='false',
        choices=['true', 'false'],
        description=(
            'Bidirectional ESC: true = allow reverse (vx_min<0, throttle [-1,1]); '
            'false = unidirectional (vx_min=0, throttle [0,1]).'
        ),
    ),
    # -----------------------------------------------------------------------
    # Depth camera selection — applies to both simulation and hardware.
    # Drives: which camera node is launched (HW), Gazebo bridge topics (sim),
    # URDF TF frames, and all RTAB-Map topic subscriptions.
    # -----------------------------------------------------------------------
    DeclareLaunchArgument(
        'depth_camera',
        default_value='l515',
        description=(
            'Name of the depth camera used for RTAB-Map (e.g. l515, d435i). '
            'Controls which topics are bridged in sim and which node is started in HW.'
        ),
    ),
    DeclareLaunchArgument(
        'hw_enable_t265',
        default_value='false',
        choices=['true', 'false'],
        description='[HW mode] Enable T265 tracking camera and odom_tf_relay.',
    ),
    DeclareLaunchArgument(
        'rover_monitor',
        default_value='false',
        choices=['true', 'false'],
        description='Launch the rover_monitor health monitoring system.',
    ),
]


def generate_launch_description() -> LaunchDescription:
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gazebo_args = LaunchConfiguration('gazebo_args')
    robot_name = LaunchConfiguration('robot_name')
    namespace = LaunchConfiguration('namespace')
    x_pos = LaunchConfiguration('x')
    y_pos = LaunchConfiguration('y')
    z_pos = LaunchConfiguration('z')
    vision = LaunchConfiguration('vision')
    localization = LaunchConfiguration('localization')
    rtabmap_enable = LaunchConfiguration('rtabmap')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    nav2_enable = LaunchConfiguration('nav2')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    nav2_bt_xml = LaunchConfiguration('nav2_bt_xml')
    nav2_through_bt = LaunchConfiguration('nav2_through_poses_bt')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_enable = LaunchConfiguration('rviz')
    enable_px4_sitl = LaunchConfiguration('enable_px4_sitl')
    px4_mode_type = LaunchConfiguration('px4_mode_type')
    reversible_drive = LaunchConfiguration('reversible_drive')
    depth_camera = LaunchConfiguration('depth_camera')
    hw_enable_t265 = LaunchConfiguration('hw_enable_t265')

    robot_description_share = get_package_share_directory('description_robot')
    rtabmap_bringup_share = get_package_share_directory('rtabmap_bringup')
    nav2_bringup_share = get_package_share_directory('ackermann_nav2_bringup')
    realsense_bringup_share = get_package_share_directory('realsense_camera_bringup')
    xacro_file = os.path.join(
        robot_description_share, 'models', 'ackermann_rover', 'ackermann_rover.urdf'
    )

    # -----------------------------------------------------------------------
    # Simulation path: Gazebo + bridge provides all sensor topics
    # -----------------------------------------------------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_description_share, 'launch', 'gazebo_bringup.launch.py')
        ),
        condition=IfCondition(use_gazebo),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gz_args': gazebo_args,
            'robot_name': robot_name,
            'namespace': namespace,
            'x': x_pos,
            'y': y_pos,
            'z': z_pos,
            'enable_px4_sitl': enable_px4_sitl,
            'depth_camera': depth_camera,
            'enable_t265': hw_enable_t265,
        }.items()
    )

    # -----------------------------------------------------------------------
    # Hardware path: real cameras + standalone robot_state_publisher
    # -----------------------------------------------------------------------

    # RSP publishes the TF tree (camera mounts, wheel kinematics) from the URDF.
    # In Gazebo mode this is started inside gazebo_bringup; here it runs standalone.
    #
    # In HW mode there are no wheel encoders connected to ROS.
    # cmd_vel_joint_relay derives /joint_states from /cmd_vel using inverse
    # Ackermann kinematics: individual per-kingpin steering angles and integrated
    # wheel rotation positions.  Intended for RViz visualisation only; SLAM and
    # Nav2 use camera-derived odometry.
    hw_joint_state_publisher = Node(
        package='robot_bringup',
        executable='cmd_vel_joint_relay.py',
        output='screen',
        condition=UnlessCondition(use_gazebo),
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_ns': robot_name,
        }],
    )

    hw_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        condition=UnlessCondition(use_gazebo),
        parameters=[{
            'robot_description': Command([
                'xacro', ' ', xacro_file, ' ',
                'ns:=', robot_name, ' ',
                'enable_d435i:=', PythonExpression(['"true" if "', depth_camera, '" == "d435i" else "false"']), ' ',
                'enable_l515:=',  PythonExpression(['"true" if "', depth_camera, '" == "l515"  else "false"']), ' ',
                'enable_t265:=', hw_enable_t265, ' ',
                'enable_rplidar:=false ',
                'enable_cubepilot:=true',
            ]),
            'use_sim_time': use_sim_time,
        }],
    )

    # Real RealSense cameras (D435i / L515 / T265 selected by hw_enable_* args).
    # The T265 odom_tf_relay is also started automatically when enable_t265:=true.
    # When T265 is enabled, delay D435i/L515 startup by 12s so T265 resets
    # and starts streaming first (avoids USB power-state races on shared hub).
    hw_cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_bringup_share, 'launch', 'realsense_camera.launch.py')
        ),
        condition=UnlessCondition(use_gazebo),
        launch_arguments={
            'enable_d435i': PythonExpression(['"true" if "', depth_camera, '" == "d435i" else "false"']),
            'enable_l515':  PythonExpression(['"true" if "', depth_camera, '" == "l515"  else "false"']),
            'enable_t265':  hw_enable_t265,
            'd435i_startup_delay_s': PythonExpression(['"12.0" if "', hw_enable_t265, '" == "true" else "0.0"']),
            'l515_startup_delay_s':  PythonExpression(['"12.0" if "', hw_enable_t265, '" == "true" else "0.0"']),
        }.items(),
    )

    # -----------------------------------------------------------------------
    # RTAB-Map — topic names derived from depth_camera in both modes.
    #
    # Sim:  /{depth_camera}/image, /{depth_camera}/camera_info,
    #       /{depth_camera}/depth_image, /{depth_camera}/imu/raw
    # HW:   {depth_camera}/color/image_raw, {depth_camera}/color/camera_info,
    #       {depth_camera}/aligned_depth_to_color/image_raw, {depth_camera}/imu
    # -----------------------------------------------------------------------
    # NOTE: px4_bridge is NOT launched here; start it separately after XRCE-DDS:
    #   ros2 launch px4_bringup px4_bringup.launch.py mode_type:=speed_steering

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_bringup_share, 'launch', 'rtabmap_slam.launch.py')
        ),
        condition=IfCondition(rtabmap_enable),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'vision': vision,
            'localization': localization,
            'rtabmap_viz': rtabmap_viz,
            'rgb_image_topic': PythonExpression([
                '"', depth_camera, '/color/image_raw"'
                ' if "', use_gazebo, '" == "false"'
                ' else "/', depth_camera, '/image"'
            ]),
            'rgb_camera_info_topic': PythonExpression([
                '"', depth_camera, '/color/camera_info"'
                ' if "', use_gazebo, '" == "false"'
                ' else "/', depth_camera, '/camera_info"'
            ]),
            'depth_image_topic': PythonExpression([
                '"', depth_camera, '/aligned_depth_to_color/image_raw"'
                ' if "', use_gazebo, '" == "false"'
                ' else "/', depth_camera, '/depth_image"'
            ]),
            'depth_camera_info_topic': PythonExpression([
                '"', depth_camera, '/aligned_depth_to_color/camera_info"'
                ' if "', use_gazebo, '" == "false"'
                ' else "/', depth_camera, '/camera_info"'
            ]),
            'imu_raw_topic': PythonExpression([
                '"', depth_camera, '/imu"'
                ' if "', use_gazebo, '" == "false"'
                ' else "/', depth_camera, '/imu/raw"'
            ]),
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'nav2_bringup.launch.py')
        ),
        condition=IfCondition(nav2_enable),
        launch_arguments={
            'params_file': nav2_params_file,
            'bt_xml': nav2_bt_xml,
            'navigate_through_poses_bt': nav2_through_bt,
            'reversible_drive': reversible_drive,
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz_enable),
    )

    from launch.actions import TimerAction
    rviz_delayed = TimerAction(
        period=5.0,
        actions=[rviz_node]
    )

    # --- Rover Monitor (optional) ---
    rover_monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rover_monitor'),
                'launch',
                'monitor.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('rover_monitor')),
    )

    ld = LaunchDescription(ARGUMENTS)
    # Simulation
    ld.add_action(gazebo_launch)
    # Hardware (mutually exclusive with Gazebo)
    ld.add_action(hw_joint_state_publisher)
    ld.add_action(hw_robot_state_publisher)
    ld.add_action(hw_cameras_launch)
    # Common
    ld.add_action(rtabmap_launch)
    ld.add_action(nav2_launch)
    ld.add_action(rviz_delayed)
    ld.add_action(rover_monitor_launch)

    return ld
