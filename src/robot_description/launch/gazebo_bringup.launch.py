#!/usr/bin/env python3
"""Gazebo bring-up launch file for the Ackermann rover."""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
import xacro


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('robot_description')
    gz_share = get_package_share_directory('ros_gz_sim')
    realsense_share = get_package_share_directory('realsense2_description')
    default_world = os.path.join(pkg_share, 'worlds', 'warehouse.sdf')
    xacro_file = os.path.join(pkg_share, 'urdf', 'donkey_sensors.urdf')

    # Preprocess the legacy URDF/Xacro tree once so shared nodes can consume it.
    robot_description_config = xacro.process_file(xacro_file).toxml()

    with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf') as temp_urdf:
        temp_urdf.write(robot_description_config)
        robot_urdf_path = temp_urdf.name

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    namespace = LaunchConfiguration('namespace')
    x_pos = LaunchConfiguration('x')
    y_pos = LaunchConfiguration('y')
    z_pos = LaunchConfiguration('z')
    gz_args = LaunchConfiguration('gz_args')

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
        default_value=TextSubstitution(text=f'-r {default_world}'),
        description='Arguments forwarded to gz sim (e.g. "-v 4 -r my.world").',
    )
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='ackmann', description='Entity name for the spawned robot.'
    )
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='', description='Optional ROS namespace for spawned nodes.'
    )
    declare_x = DeclareLaunchArgument('x', default_value='0.0', description='Initial X position in meters.')
    declare_y = DeclareLaunchArgument('y', default_value='0.0', description='Initial Y position in meters.')
    declare_z = DeclareLaunchArgument('z', default_value='0.1', description='Initial Z position in meters.')

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
                'robot_description': robot_description_config,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    spawn_entity = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'ros_gz_sim',
            'create',
            '-entity',
            robot_name,
            '-file',
            robot_urdf_path,
            '-x',
            x_pos,
            '-y',
            y_pos,
            '-z',
            z_pos,
        ],
        output='screen',
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
            set_model_path,
            set_ign_resource_path,
            set_gz_resource_path,
            gazebo,
            robot_state_publisher,
            spawn_entity,
        ]
    )
