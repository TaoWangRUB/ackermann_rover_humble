#!/usr/bin/env python3
"""Launch RTAB-Map SLAM for the Ackermann rover."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

EMPTY_PARAMS_FILE = os.path.join(
    get_package_share_directory('rtabmap_bringup'), 'config', 'empty_overrides.yaml'
)
JETSON_PROFILE_FILE = os.path.join(
    get_package_share_directory('rtabmap_bringup'), 'config', 'jetson_profile.yaml'
)

# DB lives under the workspace mount so it persists across container restarts.
# Host equivalent: <repo>/.rtabmap/rover.db (repo is mounted at /workspace).
RTABMAP_DB_PATH = '/workspace/.rtabmap/rover.db'

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
        'delete_db_on_start',
        default_value='false',
        choices=['true', 'false'],
        description='Wipe the RTAB-Map database at startup (mapping mode only; '
                    'forced off when localization:=true).'
    ),
    DeclareLaunchArgument(
        'vision',
        default_value='true',
        choices=['true', 'false'],
        description='Use RGB-D visual odometry (true) or ICP odometry (false).'
    ),
    DeclareLaunchArgument(
        'use_t265_odom',
        default_value='false',
        choices=['true', 'false'],
        description='Use T265 tracking camera as the odometry source for EKF '
                    'and RTAB-Map while keeping VO/ICP published on their own '
                    'topics for debugging or comparison.'
    ),
    DeclareLaunchArgument(
        'use_vins_odom',
        default_value='false',
        choices=['true', 'false'],
        description='Use VINS-Fusion as the odometry source for EKF and RTAB-Map '
                    'while keeping VO/ICP published on their own topics.'
    ),
    DeclareLaunchArgument(
        'use_cuvslam_odom',
        default_value='false',
        choices=['true', 'false'],
        description='Use cuVSLAM as the odometry source for EKF and RTAB-Map.'
    ),
    DeclareLaunchArgument(
        'use_rgbd_odom',
        default_value='false',
        choices=['true', 'false'],
        description='Use cuVSLAM RGBD as the odometry source for EKF and RTAB-Map.'
    ),
    DeclareLaunchArgument(
        't265_odom_topic',
        default_value='/t265/odom_base',
        description='T265 odometry topic (used when use_t265_odom:=true).'
    ),
    DeclareLaunchArgument(
        'vins_odom_topic',
        default_value='/vins_odom',
        description='VINS odometry topic (used when use_vins_odom:=true).'
    ),
    DeclareLaunchArgument(
        'cuvslam_odom_topic',
        default_value='/cuvslam_odom',
        description='cuVSLAM odometry topic (used when use_cuvslam_odom:=true).'
    ),
    DeclareLaunchArgument(
        'cuvslam_rgbd_odom_topic',
        default_value='/cuvslam_rgbd_odom',
        description='cuVSLAM RGBD odometry topic (used when use_rgbd_odom:=true).'
    ),
    DeclareLaunchArgument(
        'rtabmap_viz',
        default_value='true',
        choices=['true', 'false'],
        description='Launch rtabmap_viz for debugging.'
    ),
    DeclareLaunchArgument(
        'rtabmap_detection_rate',
        default_value='1.0',
        description='RTAB-Map signature creation rate in Hz. Use 0 to process every frame.'
    ),
    DeclareLaunchArgument(
        'rtabmap_vis_min_inliers',
        default_value='6',
        description='Minimum visual inliers required by RTAB-Map to accept loop closures. '
                    'RTAB-Map enforces an internal minimum of 6 — lower values are silently clamped. '
                    'Default 6 is the most permissive legal value; raise to 10-20 if false-positive '
                    'closures appear on a feature-rich scene.'
    ),
    DeclareLaunchArgument(
        'rtabmap_vis_max_features',
        default_value='1200',
        description='Maximum number of visual features extracted by RTAB-Map for loop closure.'
    ),
    DeclareLaunchArgument(
        'rtabmap_vis_estimation_type',
        default_value='1',
        description='RTAB-Map loop verification estimator: 0=3D-3D, 1=PnP (3D-2D, default, needs depth), 2=2D-2D epipolar (no depth required).'
    ),
    DeclareLaunchArgument(
        'rtabmap_vis_max_depth',
        default_value='4.0',
        description='Max depth [m] for features used in loop verification. '
                    'D435i depth becomes unreliable beyond ~4-5m (noise >5% relative); '
                    'clip features at 4m so PnP/3D-3D verification only uses well-conditioned '
                    'keypoints. Set 0 to disable the limit.'
    ),
    DeclareLaunchArgument(
        'rtabmap_kp_detector_strategy',
        default_value='6',
        description='RTAB-Map BoW feature detector: 0=SURF, 2=ORB, 6=GFTT/BRIEF (default), 8=GFTT/BRISK, 9=KAZE.'
    ),
    DeclareLaunchArgument(
        'rtabmap_vis_epipolar_var',
        default_value='0.02',
        description='Max epipolar geometry variance for accepting a 2D-2D loop closure (Vis/EstimationType=2). Default 0.02; raise to 0.3-0.5 for rotation-dominant motion.'
    ),
    DeclareLaunchArgument(
        'rtabmap_reg_strategy',
        default_value='2',
        description='Registration strategy for loop verification: 0=Vis only, 1=ICP only, '
                    '2=Vis+ICP cascade (default — needs /scan, which subscribe_scan=True now '
                    'guarantees by consuming depthimage_to_laserscan output when vision=true).'
    ),
    DeclareLaunchArgument(
        'rtabmap_linear_update',
        default_value='0.2',
        description='Min linear motion (m) before adding a new node. Smaller = denser map but tiny baseline between '
                    'adjacent nodes -> epipolar/PnP verification fails on near-zero translation. Default 0.2m '
                    'guarantees enough parallax for visual loop verification. RTAB-Map default is 0.1m.'
    ),
    DeclareLaunchArgument(
        'rtabmap_angular_update',
        default_value='0.2',
        description='Min angular motion (rad) before adding a new node. Default 0.2 (≈11.5°) filters '
                    'small angular jitter — important for hand-held cameras and noisy IMU integration. '
                    'RTAB-Map default is 0.1.'
    ),
    DeclareLaunchArgument(
        'approx_sync_max_interval',
        default_value='0.1',
        description='Maximum interval allowed by the RGB-D approximate synchronizers.'
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

    DeclareLaunchArgument(
        'rtabmap_udebug', default_value='false',
        choices=['true', 'false'],
        description='Pass --udebug to the rtabmap binary so its internal UDEBUG '
                    'logs appear (matcher counts, RANSAC inliers, etc.). Verbose; '
                    'use for diagnostic runs only.'),

    DeclareLaunchArgument(
        'jetson_profile', default_value='false',
        choices=['true', 'false'],
        description='Apply the Jetson-Xavier constrained-CPU profile '
                    '(config/jetson_profile.yaml): half the per-frame feature '
                    'pool, 1Hz detection rate, larger node-update threshold, '
                    'depth-image post-decimation. ~3x lower verifier latency '
                    'at the cost of ~3-4x fewer closures. Auto-applied by '
                    'start_jetson_session.sh.'),

    DeclareLaunchArgument(
        'extra_rtabmap_params_file', default_value=EMPTY_PARAMS_FILE,
        description='Optional path to a YAML file with extra RTAB-Map parameters '
                    '(merged after the in-launch defaults; later wins). Use this for '
                    'tuning knobs that the launch file does not expose explicitly, '
                    'e.g. Vis/CorNNType, RGBD/LoopClosureReextractFeatures. The default '
                    'is an empty stub so the file path is always valid.'),
]


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    vision = LaunchConfiguration('vision')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    use_vins_odom = LaunchConfiguration('use_vins_odom')
    use_cuvslam_odom = LaunchConfiguration('use_cuvslam_odom')
    use_rgbd_odom = LaunchConfiguration('use_rgbd_odom')
    use_t265_odom = LaunchConfiguration('use_t265_odom')
    rtabmap_detection_rate = LaunchConfiguration('rtabmap_detection_rate')
    rtabmap_vis_min_inliers = LaunchConfiguration('rtabmap_vis_min_inliers')
    rtabmap_vis_max_features = LaunchConfiguration('rtabmap_vis_max_features')
    rtabmap_vis_estimation_type = LaunchConfiguration('rtabmap_vis_estimation_type')
    rtabmap_vis_max_depth = LaunchConfiguration('rtabmap_vis_max_depth')
    rtabmap_kp_detector_strategy = LaunchConfiguration('rtabmap_kp_detector_strategy')
    rtabmap_vis_epipolar_var = LaunchConfiguration('rtabmap_vis_epipolar_var')
    rtabmap_reg_strategy = LaunchConfiguration('rtabmap_reg_strategy')
    rtabmap_linear_update = LaunchConfiguration('rtabmap_linear_update')
    rtabmap_angular_update = LaunchConfiguration('rtabmap_angular_update')
    approx_sync_max_interval = LaunchConfiguration('approx_sync_max_interval')
    t265_odom_topic = LaunchConfiguration('t265_odom_topic')
    vins_odom_topic = LaunchConfiguration('vins_odom_topic')
    cuvslam_odom_topic = LaunchConfiguration('cuvslam_odom_topic')
    cuvslam_rgbd_odom_topic = LaunchConfiguration('cuvslam_rgbd_odom_topic')

    # Priority: cuVSLAM > cuVSLAM RGBD > VINS-Fusion > T265 built-in > RGB-D VO > ICP.
    # VO/ICP remain available on their own topics for debugging or comparison.
    odom_topic = PythonExpression([
        '"', cuvslam_odom_topic, '" if "', use_cuvslam_odom, '" == "true"'
        ' else ("', cuvslam_rgbd_odom_topic, '" if "', use_rgbd_odom, '" == "true"'
        ' else ("', vins_odom_topic, '" if "', use_vins_odom, '" == "true"'
        ' else ("', t265_odom_topic, '" if "', use_t265_odom, '" == "true"'
        ' else ("/vo_odom" if "', vision, '" == "true" else "/icp_odom"))))'
    ])

    # Always launch VO/ICP based on vision mode (even when use_t265_odom).
    launch_visual_odom = PythonExpression([
        '"true" if "', vision, '" == "true" else "false"'
    ])
    launch_icp_odom = PythonExpression([
        '"true" if "', vision, '" == "false" else "false"'
    ])
    # Subscribe to /scan whether the source is a real lidar (vision=false) or
    # depthimage_to_laserscan (vision=true) — having the scan available lets
    # Reg/Strategy=2 (Vis+ICP cascade) actually have ICP data, instead of
    # silently nulling valid Vis transforms when ICP cannot find correspondences.
    subscribe_scan = True
    scan_topic = '/scan'

    rtabmap_parameters = {
        'subscribe_rgbd':True,
        'subscribe_scan':subscribe_scan,
        'subscribe_odom':True,
        'use_action_for_goal':True,
        'odom_sensor_sync': False,
        'database_path': RTABMAP_DB_PATH,
        # RTAB-Map's parameters should be strings:
        'Mem/NotLinkedNodesKept':'false',
        'Mem/RehearsalSimilarity': '0.3',
        'Grid/MaxGroundHeight': '0.1',
        'Grid/MaxObstacleHeight': '0.8',
        'Grid/NormalsSegmentation': 'false',
        #'Grid/RangeMax': '20',
        'Grid/3D': 'false',
        'Grid/RayTracing': 'true',
        'Reg/Force3DoF': 'true',
        'Kp/MaxFeatures': '1500',
        'Vis/CorGuessWinSize': '40',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityMaxGraphDepth': '50',
        'RGBD/ProximityPathMaxNeighbors': '10',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'approx_sync_max_interval': ParameterValue(
            approx_sync_max_interval, value_type=float
        ),
        'topic_queue_size': 30,
        'sync_queue_size': 30,
        'Rtabmap/DetectionRate': ParameterValue(rtabmap_detection_rate, value_type=str),
        'RGBD/LinearUpdate': ParameterValue(rtabmap_linear_update, value_type=str),
        'RGBD/AngularUpdate': ParameterValue(rtabmap_angular_update, value_type=str),
        'Vis/MinInliers': ParameterValue(rtabmap_vis_min_inliers, value_type=str),
        'Vis/MaxFeatures': ParameterValue(rtabmap_vis_max_features, value_type=str),
        'Vis/EstimationType': ParameterValue(rtabmap_vis_estimation_type, value_type=str),
        'Vis/MaxDepth': ParameterValue(rtabmap_vis_max_depth, value_type=str),
        'Kp/DetectorStrategy': ParameterValue(rtabmap_kp_detector_strategy, value_type=str),
        'Vis/EpipolarGeometryVar': ParameterValue(rtabmap_vis_epipolar_var, value_type=str),
    }

    shared_parameters = {
        'frame_id': 'ackermann/base_link',
        'use_sim_time': use_sim_time,
        'Reg/Strategy': ParameterValue(rtabmap_reg_strategy, value_type=str),
        'Reg/Force3DoF': 'true',
        'Mem/NotLinkedNodesKept': 'false',
        'Icp/PointToPlaneMinComplexity': '0.04'
    }

    sensor_remappings = [
        ('scan', scan_topic),
        ('imu', '/imu/data'),
        ('rgb/image', LaunchConfiguration('rgb_image_topic')), 
        ('rgb/camera_info', LaunchConfiguration('rgb_camera_info_topic')),
        ('depth/image', LaunchConfiguration('depth_image_topic')),
        ('depth/camera_info', LaunchConfiguration('depth_camera_info_topic')),
        ('cloud', '/camera/cloud')
    ]

    consumer_remappings = sensor_remappings + [
        ('odom', odom_topic),
    ]

    visual_odom_remappings = sensor_remappings + [
        ('odom', '/vo_odom'),
    ]

    icp_odom_remappings = sensor_remappings + [
        ('odom', '/icp_odom'),
    ]

    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        output='screen',
        parameters=[{
            'approx_sync': True,
            'queue_size': 30,
            'approx_sync_max_interval': ParameterValue(
                approx_sync_max_interval, value_type=float
            ),
            'use_sim_time': use_sim_time
        }],
        remappings=sensor_remappings,
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
        # When T265 provides the primary odom source, keep VO independent from
        # the D435i IMU's arbitrary startup yaw so /vo_odom remains comparable.
        'wait_imu_to_init': PythonExpression([
            'False if "', use_cuvslam_odom, '" == "true" or "', use_vins_odom,
            '" == "true" or "', use_t265_odom, '" == "true" else True'
        ]),
        'use_sim_time': use_sim_time,
        'approx_sync': True,
        'queue_size': 30,
        'approx_sync_max_interval': ParameterValue(
            approx_sync_max_interval, value_type=float
        ),
        'Odom/Strategy': '0',
        'Odom/ImageDecimation': '2',
        'Vis/MinInliers': '20',
        'Vis/FeatureType': '6',
        'Vis/MaxFeatures': '800',
        'Vis/EstimationType': '1',
        'Vis/CorType': '0',
        'Vis/CorGuessWinSize': '20',
        'Vis/MaxDepth': '5.0',
        'Odom/GuessMotion': 'true',
        'Odom/GuessSmoothingDelay': '0.1',
        'Reg/Force3DoF': 'true',
    }

    visual_odom = Node(
        condition=IfCondition(launch_visual_odom),
        package='rtabmap_odom',
        executable='rgbd_odometry',
        output='screen',
        parameters=[visual_odom_parameters],
        remappings=visual_odom_remappings,
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
        condition=IfCondition(launch_icp_odom),
        package='rtabmap_odom',
        executable='icp_odometry',
        output='screen',
        parameters=[icp_parameters, shared_parameters],
        remappings=icp_odom_remappings,
        arguments=['--ros-args', '--log-level', 'icp_odometry:=warn'],
    )

    ekf_parameters = {
        'frequency': 30.0,
        'predict_to_current_time': True,
        'history_length': 5.0,
        'smooth_lagged_data': True,
        'use_sim_time': use_sim_time,
        'two_d_mode': True,
        # External VIO relays adapt sensor-frame odometry into base_link and own
        # the odom -> base_link TF edge. EKF still publishes /odometry/filtered,
        # but must not publish a duplicate TF in those modes.
        'publish_tf': PythonExpression([
            'False if "', use_t265_odom, '" == "true" or "',
            use_cuvslam_odom, '" == "true" or "', use_rgbd_odom,
            '" == "true" or "', use_vins_odom, '" == "true" else True'
        ]),
        'map_frame': 'map',
        'odom_frame': 'odom',
        'base_link_frame': 'ackermann/base_link',
        'world_frame': 'odom',
        'sensor_timeout': 0.2,
        'transform_timeout': 0.2,
        'transform_time_offset': 0.0,
        'odom0': odom_topic,
        "odom0_config": [True, True, False,     # x, y, z position
                         False, False, True,    # roll, pitch, yaw
                         True, True, False,     # x, y, z velocity
                         False, False, True,    # roll, pitch, yaw rates
                         False, False, False],  # x, y, z acceleration
        'odom0_queue_size': 10,
        'odom0_nodelay': False,
        'odom0_differential': False,
        'odom0_relative': False,
        # IMU yaw rate — only fused when RTAB-Map VO/ICP is the odom source.
        # External VIO sources already incorporate IMU-driven heading, so adding
        # the filtered IMU yaw rate here double-counts heading information.
        'imu0': '/imu/data',
        "imu0_config": [False, False, False,   # x, y, z position
                        False, False, False,   # roll, pitch, yaw (VO owns heading)
                        False, False, False,   # x, y, z velocity
                        False, False, PythonExpression([
                            'False if "', use_cuvslam_odom, '" == "true" or "',
                            use_vins_odom, '" == "true" or "', use_t265_odom,
                            '" == "true" else True'
                        ]),    # yaw rate: skip when external VIO owns heading
                        False, False, False],  # x, y, z acceleration
        'imu0_queue_size': 10,
        'imu0_nodelay': False,
        'imu0_differential': True,
        'imu0_relative': False,
        'imu0_remove_gravitational_acceleration': True,
        'imu0_angular_velocity_covariance': [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001],
        'imu0_linear_acceleration_covariance': [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1],
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
            'publish_tf':   PythonExpression([
                'True if "', use_t265_odom, '" == "true" else False'
            ]),
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

    # Mapping mode splits into two conditional nodes so that -d is only passed
    # when the user explicitly asks for a wipe. Localization mode (handled below)
    # never gets -d: deleting the DB would erase the map being loaded.
    slam_mapping_keep = PythonExpression([
        '"true" if "', localization, '" == "false" and "',
        LaunchConfiguration('delete_db_on_start'), '" == "false" else "false"'
    ])
    slam_mapping_wipe = PythonExpression([
        '"true" if "', localization, '" == "false" and "',
        LaunchConfiguration('delete_db_on_start'), '" == "true" else "false"'
    ])

    extra_params_file = LaunchConfiguration('extra_rtabmap_params_file')

    # When jetson_profile=true, load jetson_profile.yaml; otherwise use the
    # empty stub. Both are valid file paths so launch_ros doesn't choke. Order
    # in slam_common_params puts user `extra_rtabmap_params_file` last, so
    # explicit per-run overrides still win over the Jetson defaults.
    jetson_profile_file = PythonExpression([
        "'", JETSON_PROFILE_FILE, "' if '",
        LaunchConfiguration('jetson_profile'), "' == 'true' else '",
        EMPTY_PARAMS_FILE, "'"
    ])

    slam_common_params = [rtabmap_parameters, shared_parameters,
                          {'Mem/IncrementalMemory': 'True'},
                          jetson_profile_file,
                          extra_params_file]

    # rtabmap binary's --udebug enables UDEBUG output via utilite. Each variant
    # below is gated by IfCondition so only one fires per run.
    udebug_on = PythonExpression([
        '"true" if "', LaunchConfiguration('rtabmap_udebug'), '" == "true" else "false"'
    ])
    udebug_off = PythonExpression([
        '"true" if "', LaunchConfiguration('rtabmap_udebug'), '" == "false" else "false"'
    ])

    slam_keep_quiet = Node(
        condition=IfCondition(PythonExpression([
            '"true" if "', slam_mapping_keep, '" == "true" and "', udebug_off, '" == "true" else "false"'
        ])),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=slam_common_params, remappings=consumer_remappings,
    )
    slam_keep_debug = Node(
        condition=IfCondition(PythonExpression([
            '"true" if "', slam_mapping_keep, '" == "true" and "', udebug_on, '" == "true" else "false"'
        ])),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=slam_common_params, remappings=consumer_remappings,
        arguments=['--udebug'],
    )
    slam_wipe_quiet = Node(
        condition=IfCondition(PythonExpression([
            '"true" if "', slam_mapping_wipe, '" == "true" and "', udebug_off, '" == "true" else "false"'
        ])),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=slam_common_params, remappings=consumer_remappings,
        arguments=['-d'],
    )
    slam_wipe_debug = Node(
        condition=IfCondition(PythonExpression([
            '"true" if "', slam_mapping_wipe, '" == "true" and "', udebug_on, '" == "true" else "false"'
        ])),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=slam_common_params, remappings=consumer_remappings,
        arguments=['-d', '--udebug'],
    )

    localization_node = Node(
        condition=IfCondition(localization),
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[rtabmap_parameters, shared_parameters, {
            'Mem/IncrementalMemory': 'False',
            'Mem/InitWMWithAllNodes': 'True'
        }, jetson_profile_file, extra_params_file],
        remappings=consumer_remappings,
    )

    rtabmap_viz_node = Node(
        condition=IfCondition(rtabmap_viz),
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[rtabmap_parameters, shared_parameters],
        remappings=consumer_remappings,
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
        remappings=sensor_remappings)
    
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
    
    os.makedirs(os.path.dirname(RTABMAP_DB_PATH), exist_ok=True)

    ld = LaunchDescription(ARGUMENTS)
    #ld.add_action(odom_relay_node)
    ld.add_action(rgbd_sync)
    ld.add_action(depth_to_scan)
    #ld.add_action(imu_transform_node)
    #ld.add_action(imu_filter_node)
    #ld.add_action(visual_odom)
    #ld.add_action(icp_odom)
    #ld.add_action(ekf_filter_node)
    ld.add_action(slam_keep_quiet)
    ld.add_action(slam_keep_debug)
    ld.add_action(slam_wipe_quiet)
    ld.add_action(slam_wipe_debug)
    ld.add_action(localization_node)
    ld.add_action(rgbd_to_points)
    ld.add_action(obstacle_detection)
    ld.add_action(rtabmap_viz_node)

    return ld
