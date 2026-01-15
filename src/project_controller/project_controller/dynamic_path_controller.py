#!/usr/bin/env python3
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped, PoseStamped, Pose
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from rclpy.time import Time
from tf2_ros import TransformBroadcaster, TransformStamped, StaticTransformBroadcaster
import numpy as np
import math
from rclpy.constants import S_TO_NS
from tf2_ros import TransformBroadcaster
from collections import deque
import heapq
from std_msgs.msg import String

class DynamicPathHandler():

    def __init__(self, parent_node, robot_name, wheel_radius, wheel_separation):
        self.wheel_radius_ = wheel_radius
        self.wheel_separation_ = wheel_separation
        self.robot_name_ = robot_name
        self.parent_node_ = parent_node
        
        self.min_turn_radius = 1.0
        self.max_turn_radius = 5.0
        self.straight_preference = 3.0 
        log=0
        self.obstacle_detected=False
        self.obstacle_reset_timer = None

        self.costmap_sub=self.parent_node_.create_subscription(OccupancyGrid, f"/{robot_name}/global_costmap/costmap", self.costmap_callback, 10)
        self.path_sub_ = self.parent_node_.create_subscription(
            Path, f"/{robot_name}/planned_path", self.path_callback, 10
        )
        self.collision_pub = self.parent_node_.create_publisher(
            String, f"/{robot_name}/collision_alert", 10
        )

        self.vel_pub = self.parent_node_.create_publisher(
            TwistStamped, f'/{robot_name}/project_controller/cmd_vel', 10
        )
        

    def costmap_callback(self, msg):
        self.costmap_msg = msg


        if not hasattr(self, "latest_path") or not self.latest_path:
            return
        
        if self.obstacle_detected:
            return

        if self.robot_name_:
            for pose in self.latest_path.poses[10:]:
                x = pose.pose.position.x
                y = pose.pose.position.y

                if not self.is_cell_free(x, y):

                    self.parent_node_.get_logger().warn(
                        f"{self.robot_name_}: path has collision at x={x:.2f}, y={y:.2f}"
                    )
                    collision_msg = String()
                    collision_msg.data = f"Collision at x={x:.2f}, y={y:.2f}"
                    self.collision_pub.publish(collision_msg)
                
                    self.obstacle_detected=True

                    stop_cmd = TwistStamped()
                    stop_cmd.twist.linear.x = 0.0
                    stop_cmd.twist.angular.z = 0.0

                    self.vel_pub.publish(stop_cmd)

                    if self.obstacle_reset_timer is not None:
                        self.obstacle_reset_timer.cancel()

                    self.obstacle_reset_timer = self.parent_node_.create_timer(
                        10.0,                        
                        self.reset_obstacle_flag     
                    )   

                    return

    def path_callback(self, msg):
        self.latest_path = msg

    def reset_obstacle_flag(self):
        self.obstacle_detected = False
        self.parent_node_.get_logger().info(
            f"{self.robot_name_}: obstacle_detected reset to False"
        )

        self.obstacle_reset_timer.cancel()
        self.obstacle_reset_timer = None
                

    def is_cell_free(self,x,y):
        if not hasattr(self, "costmap_msg"):
            return False
        grid = self.costmap_msg.data
        width = self.costmap_msg.info.width
        height = self.costmap_msg.info.height
        res = self.costmap_msg.info.resolution
        origin = self.costmap_msg.info.origin

        mx = int((x - origin.position.x) / res)
        my = int((y - origin.position.y) / res)

        if mx < 0 or my < 0 or mx >= width or my >= height:
            return False  

        idx = my * width + mx
        value = grid[idx]

        if value == 0:      
            return True
        elif value == 100:  
            return False
        elif value == -1:   
           
            return True