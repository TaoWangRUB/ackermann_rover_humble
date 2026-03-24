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
    DeclareLaunchArgument(
        'imu_raw_topic', default_value='/l515/imu/raw',
        description='imu topic from sensor'),
    
    DeclareLaunchArgument(
        'rgb_image_topic', default_value='/l515/image',
        description='imu topic from sensor'),
    
    DeclareLaunchArgument(
        'rgb_camera_info_topic', default_value='/l515/camera_info',
        description='imu topic from sensor'),
    
    DeclareLaunchArgument(
        'depth_image_topic', default_value='/l515/depth_image',
        description='imu topic from sensor'),
    
    DeclareLaunchArgument(
        'depth_camera_info_topic', default_value='/l515/camera_info',
        description='imu topic from sensor'),

    # odom_tf_relay — re-express an external odom source into ackermann/base_link
    DeclareLaunchArgument(
        'enable_odom_relay', default_value='false',
        choices=['true', 'false'],
        description='Launch odom_tf_relay to re-express an external odom into base_link frame.'),
    DeclareLaunchArgument(
        'relay_input_topic', default_value='/t265/odom',
        description='Source odometry topic for odom_tf_relay.'),
    DeclareLaunchArgument(
        'relay_output_topic', default_value='/t265/odom_base',
        description='Output odometry topic from odom_tf_relay (child_frame → base_link).'),
    DeclareLaunchArgument(
        'relay_base_frame', default_value='ackermann/base_link',
        description='Target child_frame_id for odom_tf_relay output.'),
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
        'subscribe_rgbd':True,
        'subscribe_scan':subscribe_scan,
        'subscribe_odom':True,
        'use_action_for_goal':True,
        'odom_sensor_sync': False,   
        # RTAB-Map's parameters should be strings:
        'Mem/NotLinkedNodesKept':'false',
        'Grid/MaxGroundHeight': '0.1',
        'Grid/MaxObstacleHeight': '0.8',
        'Grid/NormalsSegmentation': 'false',
        #'Grid/RangeMax': '20',
        'Grid/3D': 'false',
        'Grid/RayTracing': 'true',
        'Reg/Force3DoF': 'true',
        'topic_queue_size': 20,
        'sync_queue_size': 20,
        'RGBD/LinearUpdate': '0.05',     # Update map more often (smaller motion threshold)
        'RGBD/AngularUpdate': '0.05',
    }

    shared_parameters = {
        'frame_id': 'ackermann/base_link',
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
        ('rgb/image', LaunchConfiguration('rgb_image_topic')), 
        ('rgb/camera_info', LaunchConfiguration('rgb_camera_info_topic')),
        ('depth/image', LaunchConfiguration('depth_image_topic')),
        ('depth/camera_info', LaunchConfiguration('depth_camera_info_topic')),
        ('cloud', '/camera/cloud')
    ]

    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        output='screen',
        parameters=[{
            'approx_sync': True,
            'queue_size': 30,
            'approx_sync_max_interval': 0.02,
            'use_sim_time': use_sim_time
        }],
        remappings=remappings,
    )

    imu_transform_node = Node(
        package='imu_transformer',
        executable='imu_transformer_node',
        parameters=[{
            'target_frame': 'ackermann/base_link',
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('imu_in', LaunchConfiguration('imu_raw_topic')),
            ('imu_out', '/imu/raw_transformed'),
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
            'use_sim_time': use_sim_time,
        }],
        remappings=[('imu/data_raw', '/imu/raw_transformed')],
    )

    visual_odom_parameters = {
        'frame_id': 'ackermann/base_link',
        'odom_frame_id': 'odom',
        'publish_tf': False,
        'wait_for_imu_to_init': True,
        'use_sim_time': use_sim_time,
        'approx_sync': True,
        'queue_size': 30,
        'approx_sync_max_interval': 0.02,
        'Odom/Strategy': '0',
        'Vis/MinInliers': '15',
        'Vis/FeatureType': '6',
        'Vis/MaxFeatures': '1000',
        'Vis/EstimationType': '1',
        'Vis/CorType': '0',
        'Vis/CorGuessWinSize': '15',
        'Vis/MaxDepth': '20.0',
        'Odom/GuessMotion': 'true',
        'Odom/GuessSmoothingDelay': '0.1',
        'Reg/Force3DoF': 'true',
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
        'publish_tf': False,
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
        'smooth_lagged_data': True,
        'use_sim_time': use_sim_time,
        'two_d_mode': True,
        'publish_tf': True,
        'map_frame': 'map',
        'odom_frame': 'odom',
        'base_link_frame': 'ackermann/base_link',
        'world_frame': 'odom',
        'sensor_timeout': 0.2,
        'transform_timeout': 0.2,
        'transform_time_offset': 0.1,
        'odom0': odom_topic, #odom_topic '/ackermann_odom'
        "odom0_config": [True, True, False,     # x, y, z position
                         False, False, True,    # roll, pitch, yaw
                         True, True, False,     # x, y, z velocity
                         False, False, True,    # roll, pitch, yaw rates
                         False, False, False],  # x, y, z acceleration
        'odom0_queue_size': 10,
        'odom0_nodelay': False,
        'odom0_differential': False,
        'odom0_relative': True,
        'imu0': '/imu/data',
        "imu0_config": [False, False, False,   # x, y, z position
                        False, False, True,    # roll, pitch, yaw
                        False, False, False,   # x, y, z velocity
                        False, False, True,    # roll, pitch, yaw rates
                        False, False, False],  # x, y, z acceleration
        'imu0_queue_size': 10,
        'imu0_nodelay': False,
        'imu0_differential': False,
        'imu0_relative': True,
        'imu0_remove_gravitational_acceleration': True,
        'imu0_angular_velocity_covariance': [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001],
        'imu0_linear_acceleration_covariance': [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01],
        'imu0_orientation_covariance': [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01],
    }

    odom_relay_node = Node(
        condition=IfCondition(LaunchConfiguration('enable_odom_relay')),
        package='realsense_camera_bringup',
        executable='odom_tf_relay',
        name='odom_tf_relay',
        output='screen',
        parameters=[{
            'input_topic':  LaunchConfiguration('relay_input_topic'),
            'output_topic': LaunchConfiguration('relay_output_topic'),
            'base_frame':   LaunchConfiguration('relay_base_frame'),
            'use_sim_time': use_sim_time,
        }],
    )

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
        remappings=remappings + [('odom', odom_topic)],#/ackermann/odom /odometry/filtered
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
            'output_frame': 'ackermann/base_link',
            'angle_min': -3.1415,
            'angle_max': 3.1415,
            'angle_increment': 0.0087,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('depth', LaunchConfiguration('depth_image_topic')),
            ('depth_camera_info', LaunchConfiguration('depth_camera_info_topic')),
            ('scan', '/scan'),
        ],
    )
    
    # Obstacle detection with the camera for nav2 local costmap.
    # First, we need to convert depth image to a point cloud.
    rgbd_to_points = Node(
        package='rtabmap_util', executable='point_cloud_xyz', output='screen',
        parameters=[{'decimation': 2,
                     'max_depth': 20.0,
                     'voxel_size': 0.02,
                     'use_sim_time': use_sim_time}],
        remappings=remappings)
    
    # Second, we segment the floor from the obstacles.
    obstacle_parameters={
          'frame_id':'ackermann/base_link',
          'use_sim_time':use_sim_time,
          'subscribe_depth':True,
          'use_action_for_goal':True,
          'Reg/Force3DoF':'true',
          'Grid/RayTracing':'true', # Fill empty space
          'Grid/3D':'false', # Use 2D occupancy
          'Grid/RangeMax':'3',
          'Grid/NormalsSegmentation':'true', # Use passthrough filter to detect obstacles
          'Grid/MaxGroundHeight':'0.1', # All points above 5 cm are obstacles
          'Grid/MaxObstacleHeight':'0.8',  # All points over 1 meter are ignored
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }
    obstacle_detection = Node(
        package='rtabmap_util', executable='obstacles_detection', output='screen',
        parameters=[obstacle_parameters],
        remappings=[('obstacles', '/camera/obstacles'),
                    ('ground', '/camera/ground')])
    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(odom_relay_node)
    ld.add_action(rgbd_sync)
    ld.add_action(depth_to_scan)
    ld.add_action(imu_transform_node)
    ld.add_action(imu_filter_node)
    ld.add_action(visual_odom)
    ld.add_action(icp_odom)
    ld.add_action(ekf_filter_node)
    ld.add_action(slam)
    ld.add_action(localization_node)
    ld.add_action(rgbd_to_points)
    ld.add_action(obstacle_detection)
    ld.add_action(rtabmap_viz_node)

    return ld
