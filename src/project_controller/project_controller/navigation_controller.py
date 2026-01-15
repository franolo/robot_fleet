from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, PoseStamped, Pose
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import ColorRGBA
from tf_transformations import euler_from_quaternion
from tf2_ros import Buffer, TransformListener
import math
import tf2_geometry_msgs 
import rclpy
from std_msgs.msg import String


class NavigationHandler:
    def __init__(self, parent_node, robot_name, wheel_radius, wheel_separation): 
        
        self.wheel_radius_ = wheel_radius
        self.wheel_separation_ = wheel_separation
        self.robot_name_ = robot_name
        self.parent_node_ = parent_node

        self.path_ = []
        self.current_pose_ = None
        self.current_goal_idx_ = 0
        self.tolerance_ = 0.1
        self.lookahead_ = 5
        self.rcv = False
        self.visited_tolerance_ = 0.1

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.parent_node_)

        self.path_sub_ = self.parent_node_.create_subscription(
            Path, f"/{robot_name}/planned_path", self.path_callback, 10
        )

        self.marker_pub_ = self.parent_node_.create_publisher(
            MarkerArray, f"/{robot_name}/path_markers", 10
        )

        self.control_timer_ = self.parent_node_.create_timer(0.1, self.control_loop)

        self.collision_sub=self.parent_node_.create_subscription(String, f"/{robot_name}/collision_alert", self.collision_callback, 10)


    def collision_callback(self, msg: String):
        self.rcv=False



    def path_callback(self, msg: Path):
        if not self.rcv:
            self.path_ = [pose.pose for pose in msg.poses]
            self.current_goal_idx_ = 0
            self.rcv = True

    def update_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 
                f'{self.robot_name_}_base_link', 
                rclpy.time.Time()
            )
            self.current_pose_ = Pose()
            self.current_pose_.position.x = trans.transform.translation.x
            self.current_pose_.position.y = trans.transform.translation.y
            self.current_pose_.position.z = trans.transform.translation.z
            self.current_pose_.orientation = trans.transform.rotation
            return True
            
        except Exception as e:
            return False

    def control_loop(self):
        if self.update_current_pose():
            if self.rcv and self.path_:
                self.navigate()
            else:
                pass

    def get_lookahead_point(self):
        if not self.path_:
            return None
            
        lookahead_idx = self.current_goal_idx_ + self.lookahead_
        
        if lookahead_idx >= len(self.path_):
            lookahead_idx = len(self.path_) - 1
            
        return lookahead_idx
        
    def update_visited_points(self):
        if not self.path_ or self.current_pose_ is None:
            return
            
        robot_x = self.current_pose_.position.x
        robot_y = self.current_pose_.position.y
        q = self.current_pose_.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)
        
        furthest_behind = self.current_goal_idx_
        
        for i in range(self.current_goal_idx_, len(self.path_)):
            point = self.path_[i].position
            dx = point.x - robot_x
            dy = point.y - robot_y
            dist = math.sqrt(dx*dx + dy*dy)
            angle_to_point = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(angle_to_point - yaw)
            
            if dist < self.visited_tolerance_ or abs(angle_diff) > math.pi/2:
                furthest_behind = i
            else:
                break
                
        if furthest_behind > self.current_goal_idx_:
            self.current_goal_idx_ = furthest_behind
        
    def publish_goal_marker(self, lookahead_idx):
        if not self.path_:
            return
        marker_array = MarkerArray()
        self.update_visited_points()

        for i, pose in enumerate(self.path_):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.parent_node_.get_clock().now().to_msg()
            marker.ns = "path_markers"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = pose.position.x
            marker.pose.position.y = pose.position.y
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0
            
            if i == lookahead_idx:  #lookahead
                marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.9)
                marker.scale = Vector3(x=0.3, y=0.3, z=0.3)
            elif i <= self.current_goal_idx_:  #odwiedzone
                marker.color = ColorRGBA(r=0.0, g=0.6, b=0.0, a=0.4)
                marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
            else:  #nieodwiedzone
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.6)
                marker.scale = Vector3(x=0.15, y=0.15, z=0.15)
            
            marker_array.markers.append(marker)
        
        self.marker_pub_.publish(marker_array)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def navigate(self):
        if not self.path_ or self.current_pose_ is None:
            return

        last_point = self.path_[-1].position
        robot_x = self.current_pose_.position.x
        robot_y = self.current_pose_.position.y
        dist_to_end = math.sqrt((last_point.x - robot_x)**2 + (last_point.y - robot_y)**2)
        #self.parent_node_.get_logger().info(f"[{self.robot_name_}] Odległość do końca ścieżki: {dist_to_end:.2f} m.")

        if dist_to_end < 0.05 and self.current_goal_idx_ >= len(self.path_) - 1:
            self.parent_node_.get_logger().info(f"[{self.robot_name_}] Robot wykonał warunek celu.")
            self.parent_node_.odom.publish_cmd_vel(0.0, 0.0)

            if self.path_:
                self.parent_node_.get_logger().info(f"[{self.robot_name_}] Dotarto do celu.")

            self.rcv = False
            self.path_ = []
            self.current_goal_idx_ = 0
            return

        lookahead_idx = self.get_lookahead_point()
        if lookahead_idx is None:
            return
            
        goal = self.path_[lookahead_idx].position
        
        self.publish_goal_marker(lookahead_idx)

        dx = goal.x - robot_x
        dy = goal.y - robot_y
        dist = math.sqrt(dx*dx + dy*dy)

        q = self.current_pose_.orientation
        quat = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        yaw = self.normalize_angle(yaw)

        angle_to_point = math.atan2(dy, dx)
        alpha = self.normalize_angle(angle_to_point - yaw)

        v_max = 0.15
        k_alpha = 0.4
        
        v = v_max
        omega = k_alpha * alpha
        omega = max(min(omega, 0.6), -0.6)

        if abs(alpha) > 1.0:
            v *= 0.3
        elif abs(alpha) > 0.5:
            v *= 0.6
        elif abs(alpha) > 0.2:
            v *= 0.8

        self.parent_node_.odom.publish_cmd_vel(v, omega)
        
        # self.parent_node_.get_logger().info(
        #     f"[{self.robot_name_}] lookahead={lookahead_idx}({self.current_goal_idx_}+{self.lookahead_}), "
        #     f"odwiedzone={self.current_goal_idx_}, dist={dist:.2f}, alpha={alpha:.2f}"
        # )