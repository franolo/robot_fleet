#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import TimerAction

def generate_launch_description():

    master_logic_node = Node(
        package='project_master',       # Nazwa Twojego pakietu
        executable='master.py',         # Nazwa pliku wykonywalnego (zdefiniowana w setup.py!)
        name='master_logic',            # Nazwa węzła w ROS
        output='screen',                # Żeby widzieć print() w terminalu
        parameters=[
            # Tu możesz dodać parametry, jeśli master.py ich potrzebuje
            {'use_sim_time': True} 
        ]
    )

    return LaunchDescription([
        master_logic_node
    ])
