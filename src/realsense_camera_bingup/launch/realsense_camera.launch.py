from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    # --- Identity ---
    DeclareLaunchArgument('camera_name', default_value='camera',
                          description='Camera namespace and TF frame prefix'),
    DeclareLaunchArgument('serial_no', default_value='',
                          description='Device USB serial number (empty = first found)'),
    DeclareLaunchArgument('camera_model', default_value='d435i',
                          choices=['d435i', 'l515', 't265'],
                          description='Camera hardware model'),
    DeclareLaunchArgument('camera_namespace', default_value='',
                          description='Camera namespace prefix (not yet implemented)'),
    DeclareLaunchArgument('device_type', default_value='',
                          description='Substring match on device name for selection'),
    DeclareLaunchArgument('publish_tf', default_value='false',
                          description='Publish TF frames (not yet implemented)'),

    # --- D435i / L515 stream enables ---
    DeclareLaunchArgument('enable_color', default_value='true'),
    DeclareLaunchArgument('enable_depth', default_value='true'),
    DeclareLaunchArgument('enable_infra1', default_value='false',
                          description='Enable infrared stream 1'),
    DeclareLaunchArgument('enable_infra2', default_value='false',
                          description='Enable infrared stream 2'),

    # --- IMU ---
    DeclareLaunchArgument('enable_imu', default_value='false',
                          description='Enable both accel+gyro (convenience flag)'),
    DeclareLaunchArgument('enable_accel', default_value='false',
                          description='Enable accelerometer independently'),
    DeclareLaunchArgument('enable_gyro', default_value='false',
                          description='Enable gyroscope independently'),
    DeclareLaunchArgument('unite_imu_method', default_value='2',
                          description='IMU fusion: 1=copy, 2=linear_interpolation'),

    # --- Color stream ---
    DeclareLaunchArgument('color_width', default_value='640'),
    DeclareLaunchArgument('color_height', default_value='480'),
    DeclareLaunchArgument('color_fps', default_value='30'),

    # --- Depth stream ---
    DeclareLaunchArgument('depth_width', default_value='640'),
    DeclareLaunchArgument('depth_height', default_value='480'),
    DeclareLaunchArgument('depth_fps', default_value='30'),

    # --- Profile string overrides (WxHxFPS, e.g. "640x480x30") ---
    DeclareLaunchArgument('rgb_camera.color_profile', default_value='',
                          description='Color profile override: WxHxFPS'),
    DeclareLaunchArgument('depth_module.depth_profile', default_value='',
                          description='Depth profile override: WxHxFPS'),

    # --- RGB sensor controls ---
    DeclareLaunchArgument('rgb_camera.enable_auto_exposure', default_value='false'),
    DeclareLaunchArgument('rgb_camera.exposure', default_value='200',
                          description='RGB exposure in us (0 = device default)'),
    DeclareLaunchArgument('rgb_camera.gain', default_value='128',
                          description='RGB gain (0 = device default)'),

    # --- Depth sensor controls ---
    DeclareLaunchArgument('depth_module.enable_auto_exposure', default_value='false'),
    DeclareLaunchArgument('depth_module.exposure', default_value='7500',
                          description='Depth exposure in us (0 = device default)'),
    DeclareLaunchArgument('depth_module.gain', default_value='16',
                          description='Depth gain (0 = device default)'),

    # --- Sync / alignment ---
    DeclareLaunchArgument('enable_sync', default_value='true',
                          description='Pipeline frame sync (always active)'),
    DeclareLaunchArgument('align_depth.enable', default_value='true',
                          description='Align depth to color frame'),
    DeclareLaunchArgument('align_depth_to_color', default_value='false',
                          description='Legacy alias for align_depth.enable'),

    # --- Stubs ---
    DeclareLaunchArgument('enable_rgbd', default_value='false',
                          description='RGBD topic (not yet implemented)'),
    DeclareLaunchArgument('pointcloud.enable', default_value='false',
                          description='Pointcloud (not yet implemented)'),

    # --- T265 ---
    DeclareLaunchArgument('enable_fisheye', default_value='false',
                          description='Enable T265 fisheye streams (default off, odom only)'),
    DeclareLaunchArgument('fisheye_fps', default_value='30',
                          description='Fisheye frame rate — T265 only'),
]


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('realsense_camera_bingup')
    params_file = str(Path(pkg_share) / 'config' / 'realsense_params.yaml')

    node = Node(
        package='realsense_camera_bingup',
        executable='realsense_camera_node',
        name=LaunchConfiguration('camera_name'),
        output='screen',
        parameters=[
            params_file,
            {
                # Identity
                'camera_name':           LaunchConfiguration('camera_name'),
                'serial_no':             LaunchConfiguration('serial_no'),
                'camera_model':          LaunchConfiguration('camera_model'),
                'camera_namespace':      LaunchConfiguration('camera_namespace'),
                'device_type':           LaunchConfiguration('device_type'),
                'publish_tf':            LaunchConfiguration('publish_tf'),
                # Stream enables
                'enable_color':          LaunchConfiguration('enable_color'),
                'enable_depth':          LaunchConfiguration('enable_depth'),
                'enable_infra1':         LaunchConfiguration('enable_infra1'),
                'enable_infra2':         LaunchConfiguration('enable_infra2'),
                # IMU
                'enable_imu':            LaunchConfiguration('enable_imu'),
                'enable_accel':          LaunchConfiguration('enable_accel'),
                'enable_gyro':           LaunchConfiguration('enable_gyro'),
                'unite_imu_method':      LaunchConfiguration('unite_imu_method'),
                # Color
                'color_width':           LaunchConfiguration('color_width'),
                'color_height':          LaunchConfiguration('color_height'),
                'color_fps':             LaunchConfiguration('color_fps'),
                # Depth
                'depth_width':           LaunchConfiguration('depth_width'),
                'depth_height':          LaunchConfiguration('depth_height'),
                'depth_fps':             LaunchConfiguration('depth_fps'),
                # Profile overrides
                'rgb_camera.color_profile':  LaunchConfiguration('rgb_camera.color_profile'),
                'depth_module.depth_profile': LaunchConfiguration('depth_module.depth_profile'),
                # RGB sensor controls
                'rgb_camera.enable_auto_exposure': LaunchConfiguration('rgb_camera.enable_auto_exposure'),
                'rgb_camera.exposure':   LaunchConfiguration('rgb_camera.exposure'),
                'rgb_camera.gain':       LaunchConfiguration('rgb_camera.gain'),
                # Depth sensor controls
                'depth_module.enable_auto_exposure': LaunchConfiguration('depth_module.enable_auto_exposure'),
                'depth_module.exposure': LaunchConfiguration('depth_module.exposure'),
                'depth_module.gain':     LaunchConfiguration('depth_module.gain'),
                # Sync / alignment
                'enable_sync':           LaunchConfiguration('enable_sync'),
                'align_depth.enable':    LaunchConfiguration('align_depth.enable'),
                'align_depth_to_color':  LaunchConfiguration('align_depth_to_color'),
                # Stubs
                'enable_rgbd':           LaunchConfiguration('enable_rgbd'),
                'pointcloud.enable':     LaunchConfiguration('pointcloud.enable'),
                # T265
                'enable_fisheye':        LaunchConfiguration('enable_fisheye'),
                'fisheye_fps':           LaunchConfiguration('fisheye_fps'),
            },
        ],
    )

    return LaunchDescription(ARGUMENTS + [node])
