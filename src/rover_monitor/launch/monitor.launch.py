import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _build_container(context):
    """Build the composable container with resolved depth_camera prefix."""
    pkg_dir = get_package_share_directory('rover_monitor')
    config_file = os.path.join(pkg_dir, 'config', 'rover_monitor.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_telemetry = LaunchConfiguration('enable_telemetry')
    publisher_config_file = LaunchConfiguration('publisher_config_file')

    depth_camera = LaunchConfiguration('depth_camera').perform(context)

    cam_topic_overrides = {
        'probes.cam.color_topic': '/{}/color/image_raw'.format(depth_camera),
        'probes.cam.depth_topic': '/{}/depth/image_rect_raw'.format(depth_camera),
        'probes.cam.imu_topic': '/{}/imu'.format(depth_camera),
    }

    return [ComposableNodeContainer(
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
                    cam_topic_overrides,
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
                    publisher_config_file,
                    {'use_sim_time': use_sim_time},
                ],
                extra_arguments=[
                    {'use_intra_process_comms': True},
                ],
                condition=IfCondition(enable_telemetry),
            ),
        ],
        output='screen',
    )]


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_monitor')
    default_publisher_config = os.path.join(pkg_dir, 'config', 'publisher.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'enable_telemetry',
            default_value='true',
            description='Load TelemetryPublisher component (requires reachable MQTT broker)'),
        DeclareLaunchArgument(
            'publisher_config_file',
            default_value=default_publisher_config,
            description='TelemetryPublisher YAML config path'),
        DeclareLaunchArgument(
            'depth_camera',
            default_value='d435i',
            description='Depth camera name — sets cam topic prefix (d435i, l515)'),
        OpaqueFunction(function=_build_container),
    ])
