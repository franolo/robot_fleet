#!/usr/bin/env python3
from std_msgs.msg import Header
import os
from PIL import Image
import rclpy
from rclpy.node import Node
import yaml
from ament_index_python import get_package_share_directory
from std_msgs.msg import Float64MultiArray, Empty, Bool
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from rclpy.time import Time
from tf2_ros import TransformBroadcaster, TransformStamped, StaticTransformBroadcaster
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid
from rclpy.constants import S_TO_NS
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

class OdometryHandler():
    def __init__(self, parent_node, robot_name, wheel_radius, wheel_separation): 
        self.wheel_radius_ = wheel_radius
        self.wheel_separation_ = wheel_separation
        self.robot_name_ = robot_name
        self.parent_node_ = parent_node
        if self.robot_name_ == "robot2":
            self.x_ = -1.0
            self.y_ = 2.0
        else:
            self.x_ = -1.0
            self.y_ = 0.0

        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.theta_ = 0.0
        self.prev_time_ = self.parent_node_.get_clock().now()
        self.moving_threshold_ = 0.005  

        self.wheel_cmd_pub_ = self.parent_node_.create_publisher(Float64MultiArray, f"/{self.robot_name_}/simple_velocity_controller_{self.robot_name_}/commands", 10)
        self.cmd_vel_pub_ = self.parent_node_.create_publisher(TwistStamped, f"/{self.robot_name_}/project_controller/cmd_vel", 10)
        self.vel_sub_ = self.parent_node_.create_subscription(TwistStamped, f"/{self.robot_name_}/project_controller/cmd_vel", self.vel_callback, 10)
        self.joint_sub_ = self.parent_node_.create_subscription(JointState, f"/{self.robot_name_}/joint_states", self.joint_callback, 10)
        self.odom_pub_ = self.parent_node_.create_publisher(Odometry, f"/{self.robot_name_}/odom", 10)
        self.map_pub_ = self.parent_node_.create_publisher(OccupancyGrid, "map", 10)
        self.master_ping_ = self.parent_node_.create_publisher(Empty, f"/{self.robot_name_}/project_controller/master_ping", 10)
        self.is_moving_pub_ = self.parent_node_.create_publisher(Bool, f"/{self.robot_name_}/is_moving", 10)
        self.corrected_pose_pub_ = self.parent_node_.create_publisher(PoseStamped, f'/{self.robot_name_}/corrected_pose', 10)

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self.parent_node_)

        self.br_ = StaticTransformBroadcaster(self.parent_node_)

        self.speed_conversion_ = np.array([
            [self.wheel_radius_ / 2, self.wheel_radius_ / 2],
            [self.wheel_radius_ / self.wheel_separation_, -self.wheel_radius_ / self.wheel_separation_]
        ])

        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = f"{self.robot_name_}_odom"
        self.odom_msg_.child_frame_id = f"{self.robot_name_}_base_footprint"
        
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = f"{self.robot_name_}_odom"
        self.transform_stamped_.child_frame_id = f"{self.robot_name_}_base_footprint"
        self.map_timer_ = self.parent_node_.create_timer(1.0, self.map_timer_callback)
        self.ping_timer_ = self.parent_node_.create_timer(5.0, self.ping_master_callback)
        self.tf_broadcaster_ = StaticTransformBroadcaster(self.parent_node_)

        self.pose_timer_ = self.parent_node_.create_timer(0.1, self.publish_corrected_pose)

    def publish_corrected_pose(self):
        try:
            trans = self.tf_buffer_.lookup_transform(
                'map',
                f"{self.robot_name_}_base_link",
                rclpy.time.Time()
            )
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.parent_node_.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"

            pose_msg.pose.position.x = trans.transform.translation.x
            pose_msg.pose.position.y = trans.transform.translation.y
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation = trans.transform.rotation

            self.corrected_pose_pub_.publish(pose_msg)

        except Exception as e:
            pass

    def vel_callback(self, msg):
        robot_speed = np.array([[msg.twist.linear.x], [msg.twist.angular.z]])
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]
        self.wheel_cmd_pub_.publish(wheel_speed_msg)
    
    def joint_callback(self, msg):
        try:
            left_index = msg.name.index(f"{self.robot_name_}_wheel_left_joint")
            right_index = msg.name.index(f"{self.robot_name_}_wheel_right_joint")
        except ValueError:
            return

        if not hasattr(self, 'initialized_') or not self.initialized_:
            self.left_wheel_prev_pos_ = msg.position[left_index]
            self.right_wheel_prev_pos_ = msg.position[right_index]
            self.prev_time_ = Time.from_msg(msg.header.stamp)
            self.initialized_ = True
            return

        current_time = Time.from_msg(msg.header.stamp)
        dt = current_time - self.prev_time_
        if dt.nanoseconds == 0:
            return
        seconds = dt.nanoseconds / S_TO_NS

        dp_left = msg.position[left_index] - self.left_wheel_prev_pos_
        dp_right = msg.position[right_index] - self.right_wheel_prev_pos_

        fi_left = dp_left / seconds
        fi_right = dp_right / seconds

        linear = (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left) / 2.0
        angular = (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left) / self.wheel_separation_

        d_s = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left) / 2.0
        d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left) / self.wheel_separation_

        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        #Flaga na ruch
        is_moving_msg = Bool()
        if abs(linear) > self.moving_threshold_ or abs(angular) > self.moving_threshold_:
            is_moving_msg.data = True
        else:
            is_moving_msg.data = False
        
        self.is_moving_pub_.publish(is_moving_msg)

        q = quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg_.header.stamp = self.parent_node_.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular
        self.odom_pub_.publish(self.odom_msg_)

        # TF
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.parent_node_.get_clock().now().to_msg()
        self.br_.sendTransform(self.transform_stamped_)

        self.left_wheel_prev_pos_ = msg.position[left_index]
        self.right_wheel_prev_pos_ = msg.position[right_index]
        self.prev_time_ = current_time


    def publish_cmd_vel(self, linear_x, angular_z):
        cmd_vel_msg = TwistStamped()
        cmd_vel_msg.header.stamp = self.parent_node_.get_clock().now().to_msg()
        cmd_vel_msg.twist.linear.x = linear_x
        cmd_vel_msg.twist.angular.z = angular_z
        self.cmd_vel_pub_.publish(cmd_vel_msg)

    def ping_master_callback(self):
        ping_msg = Empty()
        self.master_ping_.publish(ping_msg)

    def map_timer_callback(self):
        package_share_dir = get_package_share_directory('project_controller')
        map_dir = os.path.join(package_share_dir, 'maps')

        yaml_path = os.path.join(map_dir, 'nasz_swiat.yaml')
        with open(yaml_path, 'r') as yaml_file:
            map_data = yaml.safe_load(yaml_file)

        image_path = os.path.join(os.path.dirname(yaml_path), map_data['image'])
        if not os.path.isabs(image_path):
            image_path = os.path.join(map_dir, image_path)

        resolution = float(map_data['resolution'])
        origin = map_data['origin']
        negate = bool(map_data.get('negate', 0))
        occupied_thresh = float(map_data.get('occupied_thresh', 0.65))
        free_thresh = float(map_data.get('free_thresh', 0.196))

        img = Image.open(image_path).convert('L')
        img_data = np.array(img)
        img_data = np.flipud(img_data)

        if negate:
            img_data = 255 - img_data

        grid_data = []
        for pixel in img_data.flatten():
            occ = 100 if pixel/255.0 < occupied_thresh else (0 if pixel/255.0 > free_thresh else -1)
            grid_data.append(occ)

        height, width = img_data.shape

        map_msg = OccupancyGrid()
        header = Header()
        header.stamp = self.parent_node_.get_clock().now().to_msg()
        header.frame_id = 'map'
        map_msg.header = header

        map_msg.info.resolution = resolution
        map_msg.info.width = width
        map_msg.info.height = height
        map_msg.info.origin.position.x = origin[0]
        map_msg.info.origin.position.y = origin[1]
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = grid_data

        self.map_pub_.publish(map_msg)
                