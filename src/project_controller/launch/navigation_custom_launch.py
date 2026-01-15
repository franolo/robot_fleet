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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # --- package dir ---
    bringup_dir = get_package_share_directory('project_controller')

    # --- Launch configurations / arguments ---
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')  # kept for compatibility
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # robot_id argument (1 or 2)
    declare_robot_id_cmd = DeclareLaunchArgument(
        'robot_id',
        default_value='1',
        description='ID robota (1 lub 2)'
    )
    robot_id = LaunchConfiguration('robot_id')

    # --- lifecycle nodes list ---
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]

    # --- base remappings (common) ---
    base_remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]

    # --- remappings per robot ---
    remappings_robot1 = [
        ('/global_costmap/costmap', '/robot1/global_costmap/costmap'),
        ('/global_costmap/costmap_updates', '/robot1/global_costmap/costmap_updates'),
        ('/local_costmap/costmap', '/robot1/local_costmap/costmap'),
        ('/local_costmap/costmap_updates', '/robot1/local_costmap/costmap_updates')
    ]

    remappings_robot2 = [
        ('/global_costmap/costmap', '/robot2/global_costmap/costmap'),
        ('/global_costmap/costmap_updates', '/robot2/global_costmap/costmap_updates'),
        ('/local_costmap/costmap', '/robot2/local_costmap/costmap'),
        ('/local_costmap/costmap_updates', '/robot2/local_costmap/costmap_updates')
    ]

    # --- parameter substitutions and configured_params (RewrittenYaml wrapped by ParameterFile) ---
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True
        ),
        allow_substs=True
    )

    # --- environment variable for line-buffered logging ---
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # --- declare standard launch arguments (defaults) ---
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True'
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of container that nodes will load in if use composition'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level'
    )

    # --- helper: create nav2 nodes list given remappings ---
    def nav2_nodes(remappings):
        # remappings param should be a list of tuples
        r = base_remappings + remappings
        return [
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=r + [('cmd_vel', 'cmd_vel_nav')]
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=r
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
                remappings=r
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
                remappings=r
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=r
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=r
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=r + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}
                ]
            ),
        ]

    # --- GroupActions: load nodes for robot1 or robot2 (non-composed) ---
    load_nodes_robot1 = GroupAction(
        condition=IfCondition(PythonExpression(["'", robot_id, "' == '1'"])),
        actions=nav2_nodes(remappings_robot1)
    )

    load_nodes_robot2 = GroupAction(
        condition=IfCondition(PythonExpression(["'", robot_id, "' == '2'"])),
        actions=nav2_nodes(remappings_robot2)
    )

    # --- Build LaunchDescription ---
    ld = LaunchDescription()

    # env
    ld.add_action(stdout_linebuf_envvar)

    # declare args
    ld.add_action(declare_robot_id_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # add robot-specific groups
    ld.add_action(load_nodes_robot1)
    ld.add_action(load_nodes_robot2)

    return ld