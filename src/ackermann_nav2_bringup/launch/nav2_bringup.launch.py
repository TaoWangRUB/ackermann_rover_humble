"""Ackermann-specific Nav2 bringup.

This launch file inlines the core functionality of Nav2's
`navigation_launch.py` while preserving the Clearpath ackermann-specific
namespacing and scan remappings.
"""

# Copyright 2022 Clearpath Robotics, Inc.
# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import (
    LoadComposableNodes,
    Node,
    PushRosNamespace,
    SetParameter,
    SetRemap,
)
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use sim time',
    ),
    DeclareLaunchArgument(
        'enable_stamped_cmd_vel',
        default_value='true',
        choices=['true', 'false'],
        description='Whether to use stamped cmd_vel messages',
    ),
    DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('ackermann_nav2_bringup'),
            'config',
            'nav2_params.yaml',
        ]),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    ),
    DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace',
    ),
    DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    ),
    DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if True',
    ),
    DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description=(
            'The name of the container that nodes will load in if use composition'
        ),
    ),
    DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description=(
            'Whether to respawn if a node crashes. Applied when composition is disabled.'
        ),
    ),
    DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level',
    ),
    DeclareLaunchArgument(
        'reversible_drive',
        default_value='false',
        choices=['true', 'false'],
        description=(
            'Allow reverse driving. '
            'false = unidirectional ESC: vx_min=0, min_velocity x=0. '
            'true  = bidirectional ESC: vx_min=-0.35, min_velocity x=-0.5.'
        ),
    ),
    DeclareLaunchArgument(
        'bt_xml',
        default_value=PathJoinSubstitution([
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees',
            'navigate_to_pose_w_replanning_and_recovery.xml',
        ]),
        description='Behavior Tree XML for NavigateToPose',
    ),
    DeclareLaunchArgument(
        'navigate_through_poses_bt',
        default_value=PathJoinSubstitution([
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees',
            'navigate_through_poses_w_replanning_and_recovery.xml',
        ]),
        description='Behavior Tree XML for NavigateThroughPoses',
    ),
]


def launch_setup(context, *args, **kwargs):
    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_stamped_cmd_vel = LaunchConfiguration('enable_stamped_cmd_vel')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    is_reversible = LaunchConfiguration('reversible_drive').perform(context).lower() == 'true'
    bt_xml = LaunchConfiguration('bt_xml')
    navigate_through_poses_bt = LaunchConfiguration('navigate_through_poses_bt')

    # Minimal set for CPU-constrained platforms (Xavier NX = 6 cores).
    # Removed: smoother_server (SmacPlannerHybrid has built-in smoothing),
    #          route_server (graph-based routing unused),
    #          waypoint_follower (single-goal navigation only),
    #          docking_server (no dock configured).
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
    ]

    # Map fully qualified names to relative ones so the node's namespace can be
    # prepended. Also remap odom to the EKF output /odometry/filtered to match
    # the ackermann stack.
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('odom', '/odometry/filtered'),
    ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'autostart': autostart,
        #'use_sim_time': use_sim_time,
        #'enable_stamped_cmd_vel': enable_stamped_cmd_vel,
        'vx_min': '-0.35' if is_reversible else '0.0',
        # Planner motion model: Reeds-Shepp allows reverse arcs (bidirectional),
        # Dubin is forward-only arcs. Must match reversible_drive setting.
        'motion_model_for_search': 'REEDS_SHEPP' if is_reversible else 'DUBIN',
        # min_velocity is a float[] — RewrittenYaml only handles scalars, so it is
        # injected directly on the velocity_smoother node below.
    }
    min_velocity_override = {
        'min_velocity': [-0.5, 0.0, -2.0] if is_reversible else [0.0, 0.0, -2.0],
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            SetParameter('enable_stamped_cmd_vel', enable_stamped_cmd_vel),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {
                    'default_nav_to_pose_bt_xml': bt_xml,
                    'default_nav_through_poses_bt_xml': navigate_through_poses_bt,
                }],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, min_velocity_override],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
            ),
        ],
    )

    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            SetParameter('enable_stamped_cmd_vel', enable_stamped_cmd_vel),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_planner',
                        plugin='nav2_planner::PlannerServer',
                        name='planner_server',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_behaviors',
                        plugin='behavior_server::BehaviorServer',
                        name='behavior_server',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_bt_navigator',
                        plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_navigator',
                        parameters=[configured_params, {
                            'default_nav_to_pose_bt_xml': bt_xml,
                            'default_nav_through_poses_bt_xml': navigate_through_poses_bt,
                        }],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_velocity_smoother',
                        plugin='nav2_velocity_smoother::VelocitySmoother',
                        name='velocity_smoother',
                        parameters=[configured_params, min_velocity_override],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_collision_monitor',
                        plugin='nav2_collision_monitor::CollisionMonitor',
                        name='collision_monitor',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_navigation',
                        parameters=[
                            {'autostart': autostart, 'node_names': lifecycle_nodes}
                        ],
                    ),
                ],
            ),
        ],
    )

    # Ackermann-specific namespace and scan topic remappings
    namespace_str = namespace.perform(context)
    if namespace_str and not namespace_str.startswith('/'):
        namespace_str = '/' + namespace_str

    ackermann_group = GroupAction([
        PushRosNamespace(namespace),
        SetRemap(namespace_str + '/global_costmap/scan', namespace_str + '/scan'),
        SetRemap(namespace_str + '/local_costmap/scan', namespace_str + '/scan'),
        load_nodes,
        load_composable_nodes,
    ])

    return [stdout_linebuf_envvar, ackermann_group]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld