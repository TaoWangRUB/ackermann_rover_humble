from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ===========================================================================
# Shared parameters (passed to all camera nodes via YAML + per-node overrides)
# ===========================================================================
COMMON_PARAMS = {
    'publish_tf':            LaunchConfiguration('publish_tf'),
    'enable_sync':           LaunchConfiguration('enable_sync'),
    'unite_imu_method':      LaunchConfiguration('unite_imu_method'),
    'enable_rgbd':           LaunchConfiguration('enable_rgbd'),
    'pointcloud.enable':     LaunchConfiguration('pointcloud.enable'),
}


def _depth_camera_params(prefix: str) -> dict:
    """Parameters specific to D435i / L515 depth cameras."""
    return {
        **COMMON_PARAMS,
        'camera_name':           LaunchConfiguration(f'{prefix}_camera_name'),
        'serial_no':             LaunchConfiguration(f'{prefix}_serial_no'),
        'camera_model':          prefix,
        'device_type':           LaunchConfiguration(f'{prefix}_device_type'),
        'camera_namespace':      LaunchConfiguration(f'{prefix}_camera_namespace'),
        # Stream enables
        'enable_color':          LaunchConfiguration(f'{prefix}_enable_color'),
        'enable_depth':          LaunchConfiguration(f'{prefix}_enable_depth'),
        'enable_infra1':         LaunchConfiguration(f'{prefix}_enable_infra1'),
        'enable_infra2':         LaunchConfiguration(f'{prefix}_enable_infra2'),
        # IMU
        'enable_imu':            LaunchConfiguration(f'{prefix}_enable_imu'),
        'enable_accel':          LaunchConfiguration(f'{prefix}_enable_accel'),
        'enable_gyro':           LaunchConfiguration(f'{prefix}_enable_gyro'),
        # Profile overrides
        'rgb_camera.color_profile':   LaunchConfiguration(f'{prefix}_color_profile'),
        'depth_module.depth_profile': LaunchConfiguration(f'{prefix}_depth_profile'),
        # Color
        'color_width':           LaunchConfiguration(f'{prefix}_color_width'),
        'color_height':          LaunchConfiguration(f'{prefix}_color_height'),
        'color_fps':             LaunchConfiguration(f'{prefix}_color_fps'),
        # Depth
        'depth_width':           LaunchConfiguration(f'{prefix}_depth_width'),
        'depth_height':          LaunchConfiguration(f'{prefix}_depth_height'),
        'depth_fps':             LaunchConfiguration(f'{prefix}_depth_fps'),
        # RGB sensor controls
        'rgb_camera.enable_auto_exposure': LaunchConfiguration(f'{prefix}_rgb_auto_exposure'),
        'rgb_camera.exposure':   LaunchConfiguration(f'{prefix}_rgb_exposure'),
        'rgb_camera.gain':       LaunchConfiguration(f'{prefix}_rgb_gain'),
        # Depth sensor controls
        'depth_module.enable_auto_exposure': LaunchConfiguration(f'{prefix}_depth_auto_exposure'),
        'depth_module.exposure': LaunchConfiguration(f'{prefix}_depth_exposure'),
        'depth_module.gain':     LaunchConfiguration(f'{prefix}_depth_gain'),
        # Alignment
        'align_depth.enable':    LaunchConfiguration(f'{prefix}_align_depth'),
        'align_depth_to_color':  False,
        # T265-only (unused but declared)
        'enable_fisheye':        False,
        'fisheye_fps':           30,
    }


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('realsense_camera_bringup')
    params_file = str(Path(pkg_share) / 'config' / 'realsense_params.yaml')

    ld = LaunchDescription()

    # ===================================================================
    # Global arguments
    # ===================================================================
    ld.add_action(DeclareLaunchArgument('enable_d435i', default_value='true',
                                        description='Launch D435i camera'))
    ld.add_action(DeclareLaunchArgument('enable_l515', default_value='false',
                                        description='Launch L515 camera'))
    ld.add_action(DeclareLaunchArgument('enable_t265', default_value='false',
                                        description='Launch T265 camera'))

    # Shared
    ld.add_action(DeclareLaunchArgument('publish_tf', default_value='false'))
    ld.add_action(DeclareLaunchArgument('enable_sync', default_value='true'))
    ld.add_action(DeclareLaunchArgument('unite_imu_method', default_value='2'))
    ld.add_action(DeclareLaunchArgument('enable_rgbd', default_value='false'))
    ld.add_action(DeclareLaunchArgument('pointcloud.enable', default_value='false'))

    # ===================================================================
    # D435i arguments
    # ===================================================================
    d435i_args = [
        ('d435i_camera_name',       'd435i'),
        ('d435i_serial_no',         ''),
        ('d435i_device_type',       ''),
        ('d435i_camera_namespace',  ''),
        ('d435i_enable_color',      'true'),
        ('d435i_enable_depth',      'true'),
        ('d435i_enable_infra1',     'false'),
        ('d435i_enable_infra2',     'false'),
        ('d435i_enable_imu',        'true'),
        ('d435i_enable_accel',      'true'),
        ('d435i_enable_gyro',       'true'),
        ('d435i_color_profile',     ''),
        ('d435i_depth_profile',     ''),
        ('d435i_color_width',       '640'),
        ('d435i_color_height',      '480'),
        ('d435i_color_fps',         '30'),
        ('d435i_depth_width',       '640'),
        ('d435i_depth_height',      '480'),
        ('d435i_depth_fps',         '30'),
        ('d435i_rgb_auto_exposure', 'false'),
        ('d435i_rgb_exposure',      '200'),
        ('d435i_rgb_gain',          '128'),
        ('d435i_depth_auto_exposure', 'false'),
        ('d435i_depth_exposure',    '7500'),
        ('d435i_depth_gain',        '16'),
        ('d435i_align_depth',       'true'),
        ('d435i_startup_delay_s',       '0.0'),
        ('d435i_enable_hardware_reset', 'false'),
    ]
    for name, default in d435i_args:
        ld.add_action(DeclareLaunchArgument(name, default_value=default))

    # ===================================================================
    # L515 arguments
    # ===================================================================
    l515_args = [
        ('l515_camera_name',       'l515'),
        ('l515_serial_no',         ''),
        ('l515_device_type',       ''),
        ('l515_camera_namespace',  ''),
        ('l515_enable_color',      'true'),
        ('l515_enable_depth',      'true'),
        ('l515_enable_infra1',     'false'),
        ('l515_enable_infra2',     'false'),
        ('l515_enable_imu',        'false'),
        ('l515_enable_accel',      'false'),
        ('l515_enable_gyro',       'false'),
        ('l515_color_profile',     ''),
        ('l515_depth_profile',     ''),
        ('l515_color_width',       '640'),
        ('l515_color_height',      '480'),
        ('l515_color_fps',         '30'),
        ('l515_depth_width',       '640'),
        ('l515_depth_height',      '480'),
        ('l515_depth_fps',         '30'),
        ('l515_rgb_auto_exposure', 'true'),
        ('l515_rgb_exposure',      '0'),
        ('l515_rgb_gain',          '0'),
        ('l515_depth_auto_exposure', 'true'),
        ('l515_depth_exposure',    '0'),
        ('l515_depth_gain',        '0'),
        ('l515_align_depth',       'true'),
        ('l515_startup_delay_s',       '0.0'),
        ('l515_enable_hardware_reset', 'false'),
    ]
    for name, default in l515_args:
        ld.add_action(DeclareLaunchArgument(name, default_value=default))

    # ===================================================================
    # T265 arguments
    # ===================================================================
    t265_args = [
        ('t265_camera_name',       't265'),
        ('t265_serial_no',         ''),
        ('t265_device_type',       ''),
        ('t265_camera_namespace',  ''),
        ('t265_enable_imu',        'true'),
        ('t265_enable_accel',      'false'),
        ('t265_enable_gyro',       'false'),
        ('t265_enable_fisheye',    'false'),
        ('t265_fisheye_fps',       '30'),
        # T265 starts immediately and resets first; D435i/L515 use startup_delay
        # to avoid USB power-state races on shared hubs.
        ('t265_startup_delay_s',        '0.0'),
        ('t265_enable_hardware_reset',  'false'),
        # odom_tf_relay: re-express T265 odom into ackermann/base_link
        ('t265_odom_input_topic',  '/t265/odom'),
        ('t265_odom_output_topic', '/t265/odom_base'),
        ('t265_relay_base_frame',  'ackermann/base_link'),
        ('t265_relay_output_frame', 'odom'),
        ('t265_relay_publish_tf',  'false'),
    ]
    for name, default in t265_args:
        ld.add_action(DeclareLaunchArgument(name, default_value=default))

    # ===================================================================
    # D435i node
    # ===================================================================
    d435i_node = Node(
        package='realsense_camera_bringup',
        executable='realsense_camera_node',
        name=LaunchConfiguration('d435i_camera_name'),
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_d435i')),
        parameters=[params_file, _depth_camera_params('d435i'), {
            'startup_delay_s':      LaunchConfiguration('d435i_startup_delay_s'),
            'enable_hardware_reset': LaunchConfiguration('d435i_enable_hardware_reset'),
        }],
    )
    ld.add_action(d435i_node)

    # ===================================================================
    # L515 node
    # ===================================================================
    l515_node = Node(
        package='realsense_camera_bringup',
        executable='realsense_camera_node',
        name=LaunchConfiguration('l515_camera_name'),
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_l515')),
        parameters=[params_file, _depth_camera_params('l515'), {
            'startup_delay_s':      LaunchConfiguration('l515_startup_delay_s'),
            'enable_hardware_reset': LaunchConfiguration('l515_enable_hardware_reset'),
        }],
    )
    ld.add_action(l515_node)

    # ===================================================================
    # T265 node
    # ===================================================================
    t265_node = Node(
        package='realsense_camera_bringup',
        executable='realsense_camera_node',
        name=LaunchConfiguration('t265_camera_name'),
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_t265')),
        parameters=[
            params_file,
            {
                **COMMON_PARAMS,
                'camera_name':      LaunchConfiguration('t265_camera_name'),
                'serial_no':        LaunchConfiguration('t265_serial_no'),
                'camera_model':     't265',
                'device_type':      LaunchConfiguration('t265_device_type'),
                'camera_namespace': LaunchConfiguration('t265_camera_namespace'),
                'enable_imu':        LaunchConfiguration('t265_enable_imu'),
                'enable_accel':      LaunchConfiguration('t265_enable_accel'),
                'enable_gyro':       LaunchConfiguration('t265_enable_gyro'),
                'enable_fisheye':    LaunchConfiguration('t265_enable_fisheye'),
                'fisheye_fps':       LaunchConfiguration('t265_fisheye_fps'),
                'startup_delay_s':        LaunchConfiguration('t265_startup_delay_s'),
                'enable_hardware_reset':  LaunchConfiguration('t265_enable_hardware_reset'),
                # Not used by T265 but declared by the node
                'enable_color':     False,
                'enable_depth':     False,
                'enable_infra1':    False,
                'enable_infra2':    False,
                'align_depth.enable': False,
                'align_depth_to_color': False,
                'rgb_camera.enable_auto_exposure': True,
                'rgb_camera.exposure': 0,
                'rgb_camera.gain':   0,
                'depth_module.enable_auto_exposure': True,
                'depth_module.exposure': 0,
                'depth_module.gain': 0,
                'rgb_camera.color_profile': '',
                'depth_module.depth_profile': '',
                'color_width': 640, 'color_height': 480, 'color_fps': 30,
                'depth_width': 640, 'depth_height': 480, 'depth_fps': 30,
            },
        ],
    )
    ld.add_action(t265_node)

    # ===================================================================
    # T265 odom_tf_relay — re-express /t265/odom into ackermann/base_link
    # Enabled only when T265 is enabled.
    # ===================================================================
    t265_odom_relay = Node(
        package='realsense_camera_bringup',
        executable='odom_tf_relay',
        name='t265_odom_relay',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_t265')),
        parameters=[{
            'input_topic':  LaunchConfiguration('t265_odom_input_topic'),
            'output_topic': LaunchConfiguration('t265_odom_output_topic'),
            'base_frame':   LaunchConfiguration('t265_relay_base_frame'),
            'output_frame': LaunchConfiguration('t265_relay_output_frame'),
            'publish_tf':   LaunchConfiguration('t265_relay_publish_tf'),
            'max_rate_hz':  30.0,
        }],
    )
    ld.add_action(t265_odom_relay)

    return ld
