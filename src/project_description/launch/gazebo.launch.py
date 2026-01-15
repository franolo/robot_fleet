import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bumperbot_description = get_package_share_directory("project_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        bumperbot_description, "urdf", "project.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file"
    )

    robot_name_arg = DeclareLaunchArgument(
        name="robot_name",
        default_value="robot1",
        description="Name of the robot (used for namespacing topics)"
    )

    robot_name = LaunchConfiguration("robot_name")
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=":".join([
            os.environ.get("GAZEBO_MODEL_PATH", ""),
            str(Path(bumperbot_description).parent.resolve()),
            str(Path(bumperbot_description).resolve() / "models")
        ])
    )


    
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"
    
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[
        (
            "gz_args", 
            f"-v 4 -r {os.path.join(bumperbot_description, 'worlds', 'swiat.sdf')}"
        )
    ]
             )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "bumperbot",
                   "-x", "-3",
        "-y", "-6",
        "-z", "0.0"],
    )
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            f"/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        remappings=[
            ("/scan", ["/", robot_name, "/scan"])
        ],
        parameters=[{
            'use_sim_time': True 
        }]
    )

    return LaunchDescription([
        robot_name_arg,
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        
    ])