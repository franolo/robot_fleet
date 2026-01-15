from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     arguments=["--x", "0", "--y", "0","--z", "0.103",
        #             "--qx", "1", "--qy", "0", "--qz", "0", "--qw", "0",
        #             "--frame-id", "base_footprint",
        #             "--child-frame-id", "imu_link"],
        # ),
        Node(
            package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_robot1",
        namespace="robot1",
        parameters=[os.path.join(get_package_share_directory("project_localization"), "config", "ekf_robot1.yaml")],
        output="screen"
    ),

        Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_robot2",
        namespace="robot2",
        parameters=[os.path.join(get_package_share_directory("project_localization"), "config", "ekf_robot2.yaml")],
        output="screen"
    ),
        
    ])