from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import TimerAction

def generate_launch_description():

    # ------------------ Argumenty ------------------
    use_python_arg = DeclareLaunchArgument("use_python", default_value="True")
    wheel_radius_arg = DeclareLaunchArgument("wheel_radius", default_value="0.025")
    wheel_separation_arg = DeclareLaunchArgument("wheel_separation", default_value="0.145")
    use_simple_controller_arg = DeclareLaunchArgument("use_simple_controller", default_value="True")
    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="robot1")

    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_simple_controller = LaunchConfiguration("use_simple_controller")
    robot_name = LaunchConfiguration("robot_name")

# ------------------ Joint state broadcasters ------------------
    
    joints_state_broadcaster_robot1 = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager", "/robot1/controller_manager",
                    "--controller-manager-timeout", "60"
                ]
            )
        ]
    )

    joints_state_broadcaster_robot2 = TimerAction(
        period=3.0, 
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager", "/robot2/controller_manager",
                    "--controller-manager-timeout", "60"  # Kluczowa poprawka
                ]
            )
        ]
    )

    # ------------------ Wheel controller (opcjonalny) ------------------
    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["project_controller", "--controller-manager", "/robot1/controller_manager"],  # np. dla robot1
        condition=UnlessCondition(use_simple_controller)
    )

    # ------------------ Simple velocity controllers dla robotów ------------------
    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            # Robot1
            TimerAction(
                period=2.0,  # czeka 2 sekundy, żeby ros2_control_node się uruchomił
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        namespace="robot1",
                        arguments=["simple_velocity_controller_robot1", "--controller-manager", "/robot1/controller_manager"],
                        condition=IfCondition(use_python)
                    )
                ]
            ),
            Node(
                package="project_controller",
                executable="main_controller.py",
                name="robot1_controller",
                namespace="robot1",
                parameters=[{
                    "wheel_radius": wheel_radius,
                    "wheel_separation": wheel_separation,
                    "robot_name": "robot1"
                }]
            ),

            # Robot2
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        namespace="robot2",
                        arguments=["simple_velocity_controller_robot2", "--controller-manager", "/robot2/controller_manager"],
                        condition=IfCondition(use_python)
                    )
                ]
            ),
            Node(
                package="project_controller",
                executable="main_controller.py",
                name="robot2_controller",
                namespace="robot2",
                parameters=[{
                    "wheel_radius": wheel_radius,
                    "wheel_separation": wheel_separation,
                    "robot_name": "robot2"
                }]
            )
        ]
    )

    # ------------------ LaunchDescription ------------------
    return LaunchDescription([
        use_python_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        use_simple_controller_arg,
        robot_name_arg,
        joints_state_broadcaster_robot1,
        joints_state_broadcaster_robot2,
        wheel_controller_spawner,
        simple_controller,
    ])
