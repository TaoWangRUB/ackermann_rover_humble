#!/usr/bin/env python3
"""Launch RTAB-Map SLAM for the Ackermann rover."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true.'
    ),
    DeclareLaunchArgument(
        'localization',
        default_value='false',
        choices=['true', 'false'],
        description='Run RTAB-Map in localization-only mode.'
    ),
    DeclareLaunchArgument(
        'vision',
        default_value='true',
        choices=['true', 'false'],
        description='Use RGB-D visual odometry (true) or ICP odometry (false).'
    ),
    DeclareLaunchArgument(
        'rtabmap_viz',
        default_value='true',
        choices=['true', 'false'],
        description='Launch rtabmap_viz for debugging.'
    ),
]


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    vision = LaunchConfiguration('vision')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')

    odom_topic = PythonExpression([
        '"/vo_odom" if "', vision, '" == "true" else "/icp_odom"'
    ])
    subscribe_scan = PythonExpression([
        'True if "', vision, '" == "false" else False'
    ])
    scan_topic = PythonExpression([
        '"/scan" if "', vision, '" == "false" else "/scan"'
    ])

    rtabmap_parameters = {
        'frame_id': 'ackmann/base_footprint',
        'subscribe_rgbd': True,
        'subscribe_scan': subscribe_scan,
        'use_action_for_goal': True,
        'odom_sensor_sync': True,
        'Mem/NotLinkedNodesKept': 'false',
        'Grid/MaxGroundHeight': '0.1',
        'Grid/MaxObstacleHeight': '0.8',
        'Grid/NormalsSegmentation': 'false',
        'Grid/3D': 'false',
        'Grid/RayTracing': 'true'
    }

    shared_parameters = {
        'frame_id': 'ackmann/base_footprint',
        'use_sim_time': use_sim_time,
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        'Mem/NotLinkedNodesKept': 'false',
        'Icp/PointToPlaneMinComplexity': '0.04'
    }

    remappings = [
        ('scan', scan_topic),
        ('odom', odom_topic),
        ('imu', '/imu/data'),
        ('rgb/image', '/ackmann/depth_camera/image'),
        ('rgb/camera_info', '/ackmann/depth_camera/camera_info'),
        ('depth/image', '/ackmann/depth_camera/depth_image'),
        ('depth/camera_info', '/ackmann/depth_camera/camera_info'),
    ]

    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        output='screen',
        parameters=[{
            'approx_sync': False,
            'queue_size': 30,
            'approx_sync_max_interval': 0.05,
            'use_sim_time': use_sim_time
        }],
        remappings=remappings,
    )

    imu_transform_node = Node(
        package='imu_transformer',
        executable='imu_transformer_node',
        parameters=[{
            'target_frame': 'ackmann/base_footprint',
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('imu_in', '/l515/imu/raw'),
            ('imu_out', '/l515/imu/raw_transformed'),
        ],
    )

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        output='screen',
        parameters=[{
            'use_mag': False,
            'world_frame': 'enu',
            'publish_tf': False,
        }],
        remappings=[('imu/data_raw', '/l515/imu/raw_transformed')],
    )

    visual_odom_parameters = {
        'frame_id': 'ackmann/base_footprint',
        'odom_frame_id': 'odom',
        'publish_tf': False,
        'use_sim_time': use_sim_time,
        'Odom/Strategy': '0',
        'Vis/MinInliers': '10',
        'Vis/FeatureType': '6',
        'Vis/MaxFeatures': '1000',
        'Vis/EstimationType': '1',
        'Vis/CorType': '0',
        'Vis/CorGuessWinSize': '15',
        'Vis/MaxDepth': '20.0',
        'Odom/GuessMotion': 'true',
        'Odom/GuessSmoothingDelay': '0.1',
    }

    visual_odom = Node(
        condition=IfCondition(vision),
        package='rtabmap_odom',
        executable='rgbd_odometry',
        output='screen',
        parameters=[visual_odom_parameters],
        remappings=remappings,
        arguments=['--ros-args', '--log-level', 'rgbd_odometry:=warn'],
    )

    icp_parameters = {
        'odom_frame_id': 'odom',
        'publish_tf': True,
        'use_sim_time': use_sim_time,
        'Icp/CorrespondenceRatio': '0.03',
        'Icp/PointToPlaneMinComplexity': '0.01',
        'Icp/MaxCorrespondenceDistance': '0.2',
        'Icp/VoxelSize': '0.05',
        'Icp/MaxTranslation': '1.0',
        'Icp/MaxRotation': '1.0',
        'Odom/Strategy': '1',
        'Odom/GuessSmoothingDelay': '0.3',
        'Odom/GuessMotion': 'true',
    }

    icp_odom = Node(
        condition=UnlessCondition(vision),
        package='rtabmap_odom',
        executable='icp_odometry',
        output='screen',
        parameters=[icp_parameters, shared_parameters],
        remappings=remappings,
        arguments=['--ros-args', '--log-level', 'icp_odometry:=warn'],
    )

    ekf_parameters = {
        'frequency': 30.0,
        'predict_to_current_time': True,
        'history_length': 5.0,
        'use_sim_time': use_sim_time,
        'two_d_mode': True,
        'publish_tf': True,
        'map_frame': 'map',
        'odom_frame': 'odom',
        'base_link_frame': 'ackmann/base_footprint',
        'world_frame': 'odom',
        'sensor_timeout': 0.2,
        'transform_timeout': 0.2,
        'transform_time_offset': 0.1,
        'odom0': odom_topic,
        'odom0_config': [True, True, False, False, False, True, True, True, False, False, False, True, False, False, False],
        'odom0_queue_size': 10,
        'odom0_nodelay': False,
        'odom0_differential': False,
        'odom0_relative': True,
        'imu0': '/imu/data',
        'imu0_config': [False, False, False, False, False, True, False, False, False, False, False, True, False, False, False],
        'imu0_queue_size': 10,
        'imu0_nodelay': False,
        'imu0_differential': False,
        'imu0_relative': True,
        'imu0_remove_gravitational_acceleration': True,
        'imu0_angular_velocity_covariance': [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001],
        'imu0_linear_acceleration_covariance': [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01],
        'imu0_orientation_covariance': [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01],
    }

    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_parameters],
    )

    slam = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[rtabmap_parameters, shared_parameters],
        remappings=remappings + [('odom', '/odometry/filtered')],
        arguments=['-d'],
    )

    localization_node = Node(
        condition=IfCondition(localization),
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[rtabmap_parameters, shared_parameters, {
            'Mem/IncrementalMemory': 'False',
            'Mem/InitWMWithAllNodes': 'True'
        }],
        remappings=remappings,
    )

    rtabmap_viz_node = Node(
        condition=IfCondition(rtabmap_viz),
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[rtabmap_parameters, shared_parameters],
        remappings=remappings,
    )

    depth_to_scan = Node(
        condition=IfCondition(vision),
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='rgbd_to_scan',
        parameters=[{
            'scan_height': 10,
            'range_min': 0.1,
            'range_max': 20.0,
            'output_frame': 'ackmann/base_footprint',
            'angle_min': -3.1415,
            'angle_max': 3.1415,
            'angle_increment': 0.0087,
        }],
        remappings=[
            ('depth', '/ackmann/depth_camera/depth_image'),
            ('depth_camera_info', '/ackmann/depth_camera/camera_info'),
            ('scan', '/scan'),
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rgbd_sync)
    ld.add_action(depth_to_scan)
    ld.add_action(imu_transform_node)
    ld.add_action(imu_filter_node)
    ld.add_action(visual_odom)
    ld.add_action(icp_odom)
    ld.add_action(ekf_filter_node)
    ld.add_action(slam)
    ld.add_action(localization_node)
    ld.add_action(rtabmap_viz_node)

    return ld
