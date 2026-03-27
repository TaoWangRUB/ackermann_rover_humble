import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_monitor')

    config_file = os.path.join(pkg_dir, 'config', 'rover_monitor.yaml')
    publisher_config = os.path.join(pkg_dir, 'config', 'publisher.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'),

        ComposableNodeContainer(
            name='monitor_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='rover_monitor',
                    plugin='rover_monitor::CamProbe',
                    name='cam_probe',
                    parameters=[
                        config_file,
                        {'use_sim_time': use_sim_time},
                    ],
                    extra_arguments=[
                        {'use_intra_process_comms': True},
                    ],
                ),
                ComposableNode(
                    package='rover_monitor',
                    plugin='rover_monitor::Px4Probe',
                    name='px4_probe',
                    parameters=[
                        config_file,
                        {'use_sim_time': use_sim_time},
                    ],
                    extra_arguments=[
                        {'use_intra_process_comms': True},
                    ],
                ),
                ComposableNode(
                    package='rover_monitor',
                    plugin='rover_monitor::JetsonProbe',
                    name='jetson_probe',
                    parameters=[
                        config_file,
                        {'use_sim_time': use_sim_time},
                    ],
                    extra_arguments=[
                        {'use_intra_process_comms': True},
                    ],
                ),
                ComposableNode(
                    package='rover_monitor',
                    plugin='rover_monitor::Aggregator',
                    name='monitor_aggregator',
                    parameters=[
                        config_file,
                        {'use_sim_time': use_sim_time},
                    ],
                    extra_arguments=[
                        {'use_intra_process_comms': True},
                    ],
                ),
                ComposableNode(
                    package='rover_monitor',
                    plugin='rover_monitor::TelemetryPublisher',
                    name='telemetry_publisher',
                    parameters=[
                        config_file,
                        publisher_config,
                        {'use_sim_time': use_sim_time},
                    ],
                    extra_arguments=[
                        {'use_intra_process_comms': True},
                    ],
                ),
            ],
            output='screen',
        ),
    ])
