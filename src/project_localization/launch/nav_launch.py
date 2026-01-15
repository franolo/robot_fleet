import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command

def generate_launch_description():
    # Ścieżki do pakietów
    pkg_controller = get_package_share_directory('project_controller')
    pkg_description = get_package_share_directory('project_description')
   

    # Ścieżki do plików
    map_yaml_file = os.path.join(pkg_controller, 'maps', 'my_map.yaml')
    nav2_params_file = os.path.join(pkg_controller, 'config', 'nav2_params.yaml')
    urdf_file = os.path.join(pkg_description, 'urdf', 'project.urdf.xacro')
    robot_description = Command(['xacro ', urdf_file])

    # Argumenty launcha
    declared_arguments = [
        DeclareLaunchArgument('map', default_value=map_yaml_file, description='Ścieżka do pliku mapy yaml'),
        DeclareLaunchArgument('params_file', default_value=nav2_params_file, description='Ścieżka do pliku parametrów Nav2'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Czy używać sim time'),
    ]

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription(declared_arguments + [

        # map_server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}],
        ),

        # robot_state_publisher
        

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        ),

        # Nav2 core nodes
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': True,
                         'node_names': ['controller_server',
                                        'planner_server',
                                        'bt_navigator',
                                        ]}],
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_controller, 'rviz', 'nav2_default_view.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
