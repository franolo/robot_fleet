import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from controller_manager_msgs.srv import ListControllers

def generate_launch_description():
    project_description = get_package_share_directory("project_description")
    
    # Get parent directory of the package (workspace install directory)
    parent_dir = str(Path(project_description).parent)
    
    # Set Gazebo resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=":".join([
            os.environ.get("GAZEBO_MODEL_PATH", ""),  # Existing paths
            parent_dir,  # Parent of project_description (for model://)
            os.path.join(project_description, "models")  # Package models directory
        ])
    )

    is_ignition_arg = DeclareLaunchArgument(
        name="is_ignition",
        default_value="true",
        description="Use Ignition Gazebo"
    )

    robot_name = LaunchConfiguration("robot_name")
    is_ignition = LaunchConfiguration("is_ignition")
    robot_name_arg = DeclareLaunchArgument(
        name="robot_name",
        default_value="robot1",
        description="Name of the robot"
    )

    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=PathJoinSubstitution([
            FindPackageShare("project_description"), 
            "urdf", 
            "project.urdf.xacro"
        ]),
        description="Absolute path to robot urdf file"
    )

    model_arg_real = DeclareLaunchArgument(
        name="model_realny", 
        default_value=PathJoinSubstitution([
            FindPackageShare("project_description"), 
            "urdf", 
            "realny.urdf.xacro"
        ]),
        description="Absolute path to robot urdf file"
    )

    world_path = PathJoinSubstitution([
        FindPackageShare("project_description"),
        "worlds",
        "swiat.sdf"
    ])
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory("ros_gz_sim"), 
                "launch", 
                "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={
            "gz_args": ["-v 4 -r ", world_path]
        }.items()
    )

    # Dodaj ten węzeł do scalania transformacji
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_world_odom",
        arguments=["0", "0", "0", "0", "0", "0", "world", "odom"]
    )

    # Dodaj ten węzeł do zarządzania transformacjami
    tf_manager = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_manager",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
        parameters=[{'use_sim_time': True}]
    )



    robot_description_robot1 = Command([
        "xacro ",
        LaunchConfiguration("model_realny"),
        " robot_name:=", "robot1",
        " is_ignition:=", is_ignition
    ])
    # Dodaj te zmienne na początek funkcji
    tf_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    robot_state_publisher_node_robot1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace="robot1",
        parameters=[{
            "robot_description": robot_description_robot1,
            "use_sim_time": True,
            
        }],
        remappings=[("/robot_description", "/robot1/robot_description")],
        output="screen",
    )

    robot_description_robot2 = Command([
        "xacro ",
        LaunchConfiguration("model_realny"),
        " robot_name:=", "robot2",
        " is_ignition:=", is_ignition
    ])

    robot_state_publisher_node_robot2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace="robot2",
        parameters=[{
            "robot_description": robot_description_robot2,
            "use_sim_time": True,
            
        }],
        remappings=[("/robot_description", "/robot2/robot_description")],
        output="screen",
    )

    gz_spawn_entity_robot1 = Node(
    package="ros_gz_sim",
    executable="create",
    name="spawn_entity_robot1",
    namespace="robot1",
    arguments=[
        "-topic", "/robot1/robot_description",
        "-name", "robot1",
        "-x", "-4.0",
        "-y", "-1.0",
        "-z", "0.0",
    ],
    output="screen",
)


    gz_spawn_entity_robot2 = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_entity_robot2",
        namespace="robot2",
        arguments=[
            "-topic", "/robot2/robot_description",  # Use namespaced topic
            "-name", "robot2",
            "-x", "-4.0",
            "-y", "1.0",
            "-z", "0.0",
        ],
        output="screen",
    )

    bridge_nodes = []
    for robot in ["robot1", "robot2"]:
        bridge_nodes.append(Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f"{robot}_bridge",
            
            arguments=[
                f"/{robot}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                f"/{robot}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",

                "/camera/image/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/image/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "/camera/image/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/image/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "/camera/image/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            ],
            remappings=[
                ("/camera/image/image", f"/{robot}/camera/color/image_raw"),
                ("/camera/image/camera_info", f"/{robot}/camera/color/camera_info"),
                ("/camera/image/depth_image", f"/{robot}/camera/depth/image_raw"),
                ("/camera/image/camera_info", f"/{robot}/camera/depth/camera_info"),
                ("/camera/image/points", f"/{robot}/camera/depth/points"),
            ]

        ))


    

    # Global clock bridge
    gz_clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

        # Dodaj te węzły do funkcji generate_launch_description()
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="robot1",
        parameters=[
            {"robot_description": robot_description_robot1},
            {"use_sim_time": True},
            PathJoinSubstitution([
                get_package_share_directory("project_controller"),
                "config", 
                "project_controllers.yaml"
            ])
        ],
        output="screen",
    )

    # Drugi controller_manager dla drugiego robota
    controller_manager_node2 = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="robot2",
        parameters=[
            {"robot_description": robot_description_robot2},
            {"use_sim_time": True},
            PathJoinSubstitution([
                get_package_share_directory("project_controller"),
                "config", 
                "project_controllers.yaml"
            ])
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(project_description, "rviz", "display.rviz")],
    )



    return LaunchDescription([
        SetEnvironmentVariable('XACRO_VERBOSE', '1'),
        robot_name_arg,
        is_ignition_arg,
        model_arg,
        model_arg_real,
        gazebo_resource_path,
        gazebo,
        gz_clock_bridge,
        controller_manager_node,
        controller_manager_node2,
        robot_state_publisher_node_robot1,
        robot_state_publisher_node_robot2,
        gz_spawn_entity_robot1,
        gz_spawn_entity_robot2,
        rviz_node,
        *bridge_nodes,
        
    ])