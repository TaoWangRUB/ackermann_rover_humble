#!/usr/bin/env python3
"""Gazebo bring-up launch file for the Ackermann rover."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node

from launch.event_handlers import OnProcessExit


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('description_robot')
    gz_share = get_package_share_directory('ros_gz_sim')
    realsense_share = get_package_share_directory('realsense2_description')
    default_world = os.path.join(pkg_share, 'worlds', 'warehouse.sdf')
    xacro_file = os.path.join(pkg_share, 'models', 'ackermann_rover', 'ackermann_rover.urdf')

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    namespace = LaunchConfiguration('namespace')
    x_pos = LaunchConfiguration('x')
    y_pos = LaunchConfiguration('y')
    z_pos = LaunchConfiguration('z')
    gz_args = LaunchConfiguration('gz_args')
    enable_d435i = LaunchConfiguration('enable_d435i')
    enable_l515 = LaunchConfiguration('enable_l515')
    enable_t265 = LaunchConfiguration('enable_t265')
    enable_rplidar = LaunchConfiguration('enable_rplidar')
    enable_cubepilot = LaunchConfiguration('enable_cubepilot')

    resource_roots = [
        os.path.dirname(pkg_share),  # .../share
        pkg_share,
        os.path.join(pkg_share, 'models'),
        os.path.dirname(realsense_share),
        realsense_share,
    ]
    existing_resource_entries = [
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
        os.environ.get('GAZEBO_MODEL_PATH', ''),
    ]
    combined_resource_path = ':'.join(
        path for path in (*resource_roots, *existing_resource_entries) if path
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Enable ROS clock tied to Gazebo.'
    )
    declare_gz_args = DeclareLaunchArgument(
        'gz_args',
        default_value=TextSubstitution(text=f'-s -r {default_world}'),
        description='Arguments forwarded to gz sim (e.g. "-v 4 -r my.world").',
    )
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='ackermann', description='Entity name for the spawned robot.'
    )
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='', description='Optional ROS namespace for spawned nodes.'
    )
    declare_x = DeclareLaunchArgument('x', default_value='0.0', description='Initial X position in meters.')
    declare_y = DeclareLaunchArgument('y', default_value='0.0', description='Initial Y position in meters.')
    declare_z = DeclareLaunchArgument('z', default_value='0.1', description='Initial Z position in meters.')
    declare_enable_d435i = DeclareLaunchArgument('enable_d435i', default_value='false', description='Enable D435i depth camera.')
    declare_enable_l515 = DeclareLaunchArgument('enable_l515', default_value='true', description='Enable L515 LiDAR camera.')
    declare_enable_t265 = DeclareLaunchArgument('enable_t265', default_value='false', description='Enable T265 tracking camera.')
    declare_enable_rplidar = DeclareLaunchArgument('enable_rplidar', default_value='true', description='Enable RPLiDAR.')
    declare_enable_cubepilot = DeclareLaunchArgument('enable_cubepilot', default_value='true', description='Enable CubePilot (IMU, baro, mag, GPS).')
    declare_enable_px4_sitl = DeclareLaunchArgument('enable_px4_sitl', default_value='false', description='Enable PX4 SITL joint plugins.')

    set_model_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=combined_resource_path)
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH', value=combined_resource_path
    )
    set_gz_resource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=combined_resource_path)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                'robot_description': Command([
                'xacro', ' ', xacro_file, ' ',
                'gazebo:=harmonic', ' ',
                'namespace:=', namespace, ' ',
                'enable_d435i:=', enable_d435i, ' ',
                'enable_l515:=', enable_l515, ' ',
                'enable_t265:=', enable_t265, ' ',
                'enable_rplidar:=', enable_rplidar, ' ',
                'enable_cubepilot:=', enable_cubepilot,
                ' enable_px4_sitl:=', LaunchConfiguration('enable_px4_sitl')]),
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # Bridge Gazebo Transport topics to ROS 2 so RTAB-Map can consume them
    bridge_topics = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        #'/d435i/camera_info' + '@sensor_msgs/msg/CameraInfo' + '[gz.msgs.CameraInfo',
        #'/d435i/points' + '@sensor_msgs/msg/PointCloud2' + '[gz.msgs.PointCloudPacked',
        #'/d435i/depth_image' + '@sensor_msgs/msg/Image' + '[gz.msgs.Image',
        #'/d435i/image' + '@sensor_msgs/msg/Image' + '[gz.msgs.Image',
        #'/d435i/imu' + '@sensor_msgs/msg/Imu' + '[gz.msgs.IMU',
        '/l515/camera_info' + '@sensor_msgs/msg/CameraInfo' + '[gz.msgs.CameraInfo',
        '/l515/points' + '@sensor_msgs/msg/PointCloud2' + '[gz.msgs.PointCloudPacked',
        '/l515/depth_image' + '@sensor_msgs/msg/Image' + '[gz.msgs.Image',
        '/l515/image' + '@sensor_msgs/msg/Image' + '[gz.msgs.Image',
        '/l515/imu/raw' + '@sensor_msgs/msg/Imu' + '[gz.msgs.IMU',
        #'/t265/fisheye1/camera_info' + '@sensor_msgs/msg/CameraInfo' + '[gz.msgs.CameraInfo',
        #'/t265/fisheye1/image_raw' + '@sensor_msgs/msg/Image' + '[gz.msgs.Image',
        #'/t265/fisheye2/camera_info' + '@sensor_msgs/msg/CameraInfo' + '[gz.msgs.CameraInfo',
        #'/t265/fisheye2/image_raw' + '@sensor_msgs/msg/Image' + '[gz.msgs.Image',
        #'/t265/imu' + '@sensor_msgs/msg/Imu' + '[gz.msgs.IMU',
        '/t265/pose/sample' + '@nav_msgs/msg/Odometry' + '[gz.msgs.Odometry',
        '/ackermann/odom' + '@nav_msgs/msg/Odometry' + '[gz.msgs.Odometry',
        '/ackermann/tf' + '@tf2_msgs/msg/TFMessage' + '[gz.msgs.Pose_V',
        '/model/ackermann/tf' + '@tf2_msgs/msg/TFMessage' + '[gz.msgs.Pose_V',
        #'/ackermann/joint_state' + '@sensor_msgs/msg/JointState' + '[gz.msgs.Model',
        #'/ackermann/cmd_vel' + '@geometry_msgs/msg/Twist' + ']gz.msgs.Twist',
        '/rplidar/scan' + '@sensor_msgs/msg/LaserScan' + '[gz.msgs.LaserScan',
        #'/cubepilot/imu' + '@sensor_msgs/msg/Imu' + '[gz.msgs.IMU',
    ]

    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_topics,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-x', x_pos,
            '-y', y_pos,
            '-z', z_pos,
        ],
        output='screen',
    )
    
    
    ros2_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'controller_bringup.launch.py')),
        launch_arguments=[
            ('namespace', namespace),
        ]
    ) 
    ros2_controller_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[ros2_controller],
        ),
        condition=UnlessCondition(LaunchConfiguration('enable_px4_sitl')),
    )
    
    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_gz_args,
            declare_robot_name,
            declare_namespace,
            declare_x,
            declare_y,
            declare_z,
            declare_enable_d435i,
            declare_enable_l515,
            declare_enable_t265,
            declare_enable_rplidar,
            declare_enable_cubepilot,
            declare_enable_px4_sitl,
            set_model_path,
            set_ign_resource_path,
            set_gz_resource_path,
            parameter_bridge,
            gazebo,
            spawn_entity,
            #joint_state_publisher,
            robot_state_publisher,
            ros2_controller_callback,
        ]
    )
