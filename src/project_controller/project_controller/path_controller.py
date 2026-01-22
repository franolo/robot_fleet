
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


class PathHandler:
    def __init__(self, parent_node, robot_name, wheel_radius=0.1, wheel_separation=0.5): 
        self.parent_node = parent_node
        self.robot_name = robot_name
        self.wheel_radius_ = wheel_radius
        self.wheel_separation_ = wheel_separation
        
        self.min_turn_radius = 0.3
        self.max_turn_radius = 0.3
        self.theta_res =  math.pi/36 
        self.res_xy=0.1
        self.start_pose=None
        self.path_computed=False

        self.logger = parent_node.get_logger()

        # Mapa będzie aktualizowana przez Mastera
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin = (0.0, 0.0)

        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
        
        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.map_sub = self.parent_node.create_subscription(
            OccupancyGrid,
            '/map',  # Globalna mapa
            self.map_callback,
            map_qos
        )

        self.costmap_sub=self.parent_node.create_subscription(OccupancyGrid, f"/{robot_name}/global_costmap/costmap", self.costmap_callback, 10)

    
    def map_callback(self, msg: OccupancyGrid):
        self.update_map(msg)  
        self.logger.info(f"PathHandler: Mapa zaktualizowana {self.map_width}x{self.map_height}, res: {self.map_resolution}")

    def update_map(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        
        inflation_radius = 0.15
        self.map_data = self.inflate_map(self.map_data, inflation_radius)

    def plan_path(self, start_x, start_y, start_theta, goal_x, goal_y):
        if self.map_data is None:
            self.logger.warn("PathHandler: Brak mapy! Nie mogę wyznaczyć trasy.")
            return None

        if not self.is_free(start_x, start_y):
            self.logger.warn(f"PathHandler: Start w przeszkodzie ({start_x:.2f}, {start_y:.2f})")
            return None
        if not self.is_free(goal_x, goal_y):
            self.logger.warn(f"PathHandler: Cel w przeszkodzie ({goal_x:.2f}, {goal_y:.2f})")
            return None

        path_points = self.astar(start_x, start_y, start_theta, goal_x, goal_y)

        if path_points:
            return self._convert_points_to_path_msg(path_points)
        else:
            return None
        
    def costmap_callback(self,msg: OccupancyGrid):
    
        self.costmap_msg=msg
        self.costmap_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.costmap_width = msg.info.width
        self.costmap_height = msg.info.height
        self.costmap_resolution = msg.info.resolution
        self.costmap_datamap_origin = (msg.info.origin.position.x, msg.info.origin.position.y)


    def _convert_points_to_path_msg(self, path_points):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        
        for x, y, theta in path_points:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            
            q = quaternion_from_euler(0, 0, theta)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            
            path_msg.poses.append(pose)
            self.path_computed=True
        
        return path_msg

    def inflate_map(self, data, inflation_radius):
        cells = int(inflation_radius / self.map_resolution)
        inflated = data.copy()

        for y in range(self.map_height):
            for x in range(self.map_width):
                if data[y, x] > 50: 
                    x_min = max(0, x - cells)
                    x_max = min(self.map_width, x + cells)
                    y_min = max(0, y - cells)
                    y_max = min(self.map_height, y + cells)
                    inflated[y_min:y_max, x_min:x_max] = 100 

        return inflated

    def world_to_grid(self, x, y):
        if self.map_resolution == 0.0:
            self.logger.error("world_to_grid: map_resolution = 0! Mapa nie zainicjalizowana.")
            return 0, 0  # Zwróć bezpieczne wartości
        gx = int((x - self.map_origin[0]) / self.map_resolution)
        gy = int((y - self.map_origin[1]) / self.map_resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = gx * self.map_resolution + self.map_origin[0]
        y = gy * self.map_resolution + self.map_origin[1]
        return x, y

    def is_free(self, x, y):
        gx, gy = self.world_to_grid(x, y)
        if gx < 0 or gy < 0 or gx >= self.map_width or gy >= self.map_height:
            return False
        return self.map_data[gy][gx] == 0
    
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
    
    def is_really_free(self, x, y):
        if not self.is_free(x, y):
            return False

       
        if hasattr(self, "costmap_msg"):
            if not self.is_cell_free(x, y):
                return False

        return True

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def angle_difference(self, a, b):
        diff = a - b
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return abs(diff)
    
# astar:
    def astar(self, start_x, start_y, start_theta, goal_x, goal_y):
        iteration_limit = 20000
        iterations = 0
        open_set = []
        closed_set= set()
        came_from = {}
        start_node = (round(start_x/self.res_xy)*self.res_xy, 
                      round(start_y/self.res_xy)*self.res_xy, 
                      round(start_theta/self.theta_res)*self.theta_res)
        self.start_pose=start_node
        g = {start_node: 0}
        h = {start_node: self.distance(start_x, start_y, goal_x, goal_y)}
        f = {start_node: h[start_node]}
        heapq.heappush(open_set, (f[start_node], start_node))

        while open_set and iterations < iteration_limit:
            iterations +=1
            min_f = heapq.heappop(open_set)
            dist = self.distance(min_f[1][0], min_f[1][1], goal_x, goal_y)
            if dist<0.1:
                return self.reconstruct_path(came_from, min_f[1])
            if min_f[1] in closed_set:
                continue
            closed_set.add(min_f[1])
            next_nodes = self.find_next_nodes(closed_set, min_f[1])
            for next_node in next_nodes:
                turn_penalty = 0.0
                if next_node[2] != min_f[1][2]:
                    turn_penalty = 0.2
                g_element = g[min_f[1]] + self.distance(min_f[1][0], min_f[1][1], next_node[0], next_node[1])+turn_penalty
                h_element = self.distance(next_node[0], next_node[1], goal_x, goal_y)
                f_element = g_element + h_element
                if next_node in closed_set:
                    continue
                if next_node not in g or g_element < g[next_node]:
                    came_from[next_node] = min_f[1]
                    g[next_node] = g_element
                    h[next_node] = h_element
                    f[next_node] = f_element
                    heapq.heappush(open_set, (f_element, next_node))
                

    def reconstruct_path(self, came_from, current):
        states = [current]
        while current in came_from:
            current = came_from[current]
            states.append(current)
        states.reverse()

        final_path = []

        for i in range(len(states)-1):
            x1, y1, t1 = states[i]
            x2, y2, t2 = states[i+1]

            dtheta = self.angle_difference(t2, t1)

            if abs(dtheta - math.pi/2) < 0.01:
                turn = t2 - t1
                if turn > 0:
                    angle_diff = math.pi/2 
                else:
                    angle_diff = -math.pi/2

                radius = self.distance(x1,y1,x2,y2)/ math.sqrt(2)
                final_path.append((x1, y1, t1))

                arc_points = self.generate_arc(x1, y1, t1, radius, angle_diff)
                final_path.extend(arc_points)

            else:
                final_path.append((x1, y1, t1))

        final_path.append(states[-1])
        return final_path


    def find_next_nodes(self, closed_set, current):   
        movements = []
        x, y, theta = current
        # start_x, start_y, start_theta = self.start_pose
        # if self.start_pose is not None:
        #     dist_to_start = self.distance(x, y, start_x, start_y)
        #     if dist_to_start < self.res_xy: 
        #         for i in [math.pi/4, math.pi/2, -math.pi/4, -math.pi/2]:
        #             new_theta = (theta + i) % (2*math.pi)
        #             new_theta_rounded = round(new_theta / self.theta_res) * self.theta_res
        #             candidate = (x, y, new_theta_rounded)
        #             if candidate not in closed_set:
        #                 movements.append(candidate)

        x_new = x + self.res_xy * math.cos(theta)
        y_new = y + self.res_xy * math.sin(theta)

        x_new = round(x_new / self.res_xy) * self.res_xy
        y_new = round(y_new / self.res_xy) * self.res_xy
        theta_new = round(theta / self.theta_res) * self.theta_res

        if self.is_really_free(x_new, y_new) and (x_new, y_new, theta_new) not in closed_set:
            movements.append((x_new, y_new, theta_new))

        dtheta = math.pi/2
        for radius in np.linspace(self.min_turn_radius,self.max_turn_radius,1):
            theta_left = theta+dtheta
            x_left = x+radius*(math.sin(theta_left)-math.sin(theta))
            y_left = y-radius*(math.cos(theta_left)-math.cos(theta))
            x_left = round(x_left/self.res_xy)*self.res_xy
            y_left = round(y_left/self.res_xy)*self.res_xy
            theta_left = round(theta_left/self.theta_res)*self.theta_res
            left_points = self.generate_arc(x,y,theta,radius,dtheta)
            arc_free = True
            for px,py,t in left_points:
                if not self.is_really_free(px, py):
                    arc_free = False
                    break
            if arc_free and (x_left, y_left, theta_left) not in closed_set:
                movements.append((x_left, y_left, theta_left))

            theta_right = theta-dtheta
            x_right = x-radius*(math.sin(theta_right)-math.sin(theta))
            y_right = y+radius*(math.cos(theta_right)-math.cos(theta))
            x_right = round(x_right/self.res_xy)*self.res_xy
            y_right = round(y_right/self.res_xy)*self.res_xy
            theta_right = round(theta_right/self.theta_res)*self.theta_res
            right_points = self.generate_arc(x,y,theta,radius,-dtheta)
            arc_free = True
            for px,py,t in right_points:
                if not self.is_really_free(px, py):
                    arc_free = False
                    break
            if arc_free and (x_right, y_right, theta_right) not in closed_set:
                movements.append((x_right, y_right, theta_right))

        return movements
    
    def generate_arc(self, x_start, y_start, theta_start, radius, angle_diff=math.pi/2, point_spacing=0.1):
        points=[]
        num_points = int(abs(angle_diff) * radius / point_spacing)
        num_points = max(1, num_points)
        angle_step = angle_diff/num_points

        for i in range(1, num_points+1):
            if angle_diff>0:
                theta = theta_start+i*angle_step
                x=x_start+radius*(math.sin(theta)-math.sin(theta_start))
                y=y_start-radius*(math.cos(theta)-math.cos(theta_start))
            else:
                theta = theta_start+i*angle_step
                x=x_start-radius*(math.sin(theta)-math.sin(theta_start))
                y=y_start+radius*(math.cos(theta)-math.cos(theta_start))
            points.append((x,y,theta))
        
        return points
