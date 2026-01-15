#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import String, Float32
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, TwistStamped, PoseStamped
from stan_publisher import MasterPublisher, StanRobota
from stan_subscriber import MasterSubscriber
from connection_test import ConnectionTest 
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path


try:
    from project_controller.path_controller import PathHandler
except ImportError:
    import sys
    import os
    from path_controller import PathHandler 

class Master(Node):
    def __init__(self):
        super().__init__('master_node')

        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.declare_parameter("robot_name", "robot1")
        robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        
        self.robot_names = ["robot1", "robot2"]
        self.odleglosci_do_odbioru = {name: None for name in self.robot_names}
        self.aktywne_sciezki = {name: None for name in self.robot_names}
        self.pozycje_robotow = {name: None for name in self.robot_names}
        self.path_planners = {}
        self.rotation_targets = {}
        self.cmd_vel_pubs = {}

        for name in self.robot_names:
            self.cmd_vel_pubs[name] = self.create_publisher(TwistStamped, f'/{name}/project_controller/cmd_vel', 10)
        
        # self.goal_publishers = {}
        # for name in self.robot_names:
        #     self.goal_publishers[name] = self.create_publisher(
        #         PoseStamped,
        #         f'/{name}/project_controller/goal',
        #         10
        #     )
        self.collision_subs = {}

        for name in self.robot_names:
            self.collision_subs[name] = self.create_subscription(
                String,
                f"/{name}/collision_alert",
                lambda msg, n=name: self.collision_callback(n, msg),
                10
            )

        for name in self.robot_names:
            self.path_planners[name] = PathHandler(self, name)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.connection_test = ConnectionTest(self, self.robot_names)

        self.master_publisher = MasterPublisher(self, self.robot_names)
        self.kandydaci_pub = self.create_publisher(String, 'debug_kandydaci', 10)
        self.master_subscriber = MasterSubscriber(self, self.robot_names)
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, map_qos)

        # Debugowe publishery do wizualizacji na wykresie
        self.pub_debug_current = self.create_publisher(Float32, '/debug/current_angle', 10)
        self.pub_debug_target = self.create_publisher(Float32, '/debug/target_angle', 10)
        self.pub_debug_error = self.create_publisher(Float32, '/debug/error', 10)

        self.czasy_oczekiwania = {}
        self.rotation_done = {name: False for name in self.robot_names}

        self.dock1 = (-1.0, 0.0)
        self.dock2 = (-1.0, 2.0)
        self.dowoz = (2.0, 1.0)
        self.odbior = (1.0, 0.0)

        self.logic_timer = self.create_timer(0.1, self.main_control_loop)
        self.state_timer = self.create_timer(2.0, self.publish_states)
        self.tf_timer = self.create_timer(0.5, self.check_robot_positions)

    def map_callback(self, msg: OccupancyGrid):
        for planner in self.path_planners.values():
            planner.update_map(msg)

    def collision_callback(self, robot_name, msg):
        self.get_logger().warn(f"KOLIZJA zgłoszona przez {robot_name}")

        
        trans = self.get_robot_transform(f"{robot_name}_base_link")
        if not trans:
            return

        rx = trans.transform.translation.x
        ry = trans.transform.translation.y
        q = trans.transform.rotation
        _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])

        
        stan = self.master_publisher.states.get(robot_name)

        if stan == StanRobota.ODBIOR:
            goal_x, goal_y = self.odbior
        elif stan == StanRobota.DOWOZ:
            goal_x, goal_y = self.dowoz
        elif stan == StanRobota.DOCK:
            goal_x, goal_y = self.dock1 if robot_name == "robot1" else self.dock2
        else:
            self.get_logger().warn(f"{robot_name}: kolizja w stanie {stan}, brak replanu")
            return

        
        planner = self.path_planners[robot_name]
        new_path = self.plan_safe_path(robot_name, rx, ry, theta, goal_x, goal_y)

        if new_path:
            new_path.header.stamp = self.get_clock().now().to_msg()
            self.master_publisher.publish_path(robot_name, new_path)
            self.get_logger().warn(f"{robot_name}: NOWA ŚCIEŻKA OPUBLIKOWANA")
        else:
            self.get_logger().error(f"{robot_name}: NIE UDAŁO SIĘ REPLANOWAĆ")


    def main_control_loop(self):
        aktywne = self.connection_test.aktywne_roboty 
        kandydaci_do_odbioru = []

        arrival_threshold = 0.2
        strefa_odbioru_zajeta = False
        
        for robot_name in self.robot_names:
            stan = self.master_publisher.states.get(robot_name)
            if stan in [StanRobota.ODBIOR, StanRobota.BUSY]:
                strefa_odbioru_zajeta = True
                break

        for robot_name in self.robot_names:
            obecny_stan = self.master_publisher.states.get(robot_name)

            trans = self.get_robot_transform(f"{robot_name}_base_link")

            if trans is None:
                continue
            
            rx = trans.transform.translation.x
            ry = trans.transform.translation.y
            
            q = trans.transform.rotation
            _, _, current_theta = euler_from_quaternion([q.x, q.y, q.z, q.w])

            self.pozycje_robotow[robot_name] = (rx, ry)

            is_stopped = not self.master_subscriber.is_robot_moving(robot_name)
            is_rotating = robot_name in self.rotation_targets

            # --- Obsługa stanu IDLE ---
            if robot_name in aktywne and obecny_stan == StanRobota.NONE and not self.master_subscriber.is_robot_moving(robot_name):
                self.master_publisher.set_state(robot_name, StanRobota.IDLE)
                dystans = self.odleglosci_do_odbioru.get(robot_name)
                if dystans is not None:
                    kandydaci_do_odbioru.append((dystans, robot_name))

            elif robot_name in aktywne and obecny_stan == StanRobota.IDLE:
                dystans = self.odleglosci_do_odbioru.get(robot_name)
                if dystans is not None:
                    kandydaci_do_odbioru.append((dystans, robot_name))

            # --- Obsługa dotarcia do ODBIORU i planowanie DOWOZU ---
            if obecny_stan == StanRobota.ODBIOR:
                cel_x, cel_y = self.odbior
                

                dist_to_target = math.sqrt((rx - cel_x)**2 + (ry - cel_y)**2)

                if dist_to_target < arrival_threshold and (is_stopped or is_rotating):
                    self.aktywne_sciezki[robot_name] = None
                    start_czasu = self.czasy_oczekiwania.get(robot_name)

                    if start_czasu is None:
                        self.get_logger().info(f"{robot_name} dotarł do ODBIOR, rozpoczynam ładowanie.")
                        self.czasy_oczekiwania[robot_name] = self.get_clock().now()
                        continue
                    else:
                        teraz = self.get_clock().now()
                        elapsed = (teraz - start_czasu).nanoseconds / 1e9
                        self.get_logger().info(f"{robot_name} czeka od {elapsed:.1f}s na ODBIOR.")

                        if elapsed < 3.0:
                            continue
                        else:
                            # --- SEKCJA OBROTU I PLANOWANIA NOWEJ TRASY ---
                            if self.obrot_180(robot_name):
                                self.get_logger().info(f"{robot_name}: Obrót zakończony. Zatrzymuję i planuję DOWOZ.")
                                self.czasy_oczekiwania.pop(robot_name, None) 
                                
                                dowoz_x, dowoz_y = self.dowoz
                                path_planner = self.path_planners.get(robot_name)

                                if path_planner:
                                    sciezka = self.plan_safe_path(robot_name,rx, ry, current_theta, dowoz_x, dowoz_y)

                                    if sciezka:
                                        sciezka.header.stamp = self.get_clock().now().to_msg()
                                        self.master_publisher.publish_path(robot_name, sciezka)
                                        self.master_publisher.set_state(robot_name, StanRobota.DOWOZ)
                                        self.get_logger().info(f"Zmieniono stan {robot_name} na DOWOZ.")
                                    else:
                                        self.get_logger().warn(f"BŁĄD: Nie udało się wyznaczyć ścieżki do DOWOZ dla {robot_name}!")
                                continue 

                            else:
                                return 
                else:
                    if dist_to_target >= arrival_threshold:
                        #self.get_logger().info(f"{robot_name} odległość do ODBIORU: {dist_to_target:.2f}.")
                        self.czasy_oczekiwania.pop(robot_name, None)

            # --- Obsługa dotarcia do DOWOZU i planowanie powrotu do DOCK ---
            elif obecny_stan == StanRobota.DOWOZ:
                cel_x, cel_y = self.dowoz
                

                dist_to_target = math.sqrt((rx - cel_x)**2 + (ry - cel_y)**2)

                if dist_to_target < arrival_threshold and (is_stopped or is_rotating):
                    self.aktywne_sciezki[robot_name] = None
                    start_czasu = self.czasy_oczekiwania.get(robot_name)

                    if start_czasu is None:
                        self.get_logger().info(f"{robot_name} dotarł do DOWOZ. Czekam 10s na rozładunek...")
                        self.czasy_oczekiwania[robot_name] = self.get_clock().now()
                        continue
                    else:
                        teraz = self.get_clock().now()
                        elapsed = (teraz - start_czasu).nanoseconds / 1e9
                        if elapsed < 3.0:
                            continue 
                        else:
                            if self.obrot_180(robot_name):
                                self.get_logger().info(f"{robot_name} rozładowany -> powrót do DOCK.")
                                self.czasy_oczekiwania.pop(robot_name, None)
                                
                                if robot_name == "robot1":
                                    dock_x, dock_y = self.dock1
                                elif robot_name == "robot2":
                                    dock_x, dock_y = self.dock2
                                path_planner = self.path_planners.get(robot_name)

                                if path_planner:
                                    sciezka = self.plan_safe_path(robot_name,rx, ry, current_theta, dock_x, dock_y)
                                    if sciezka:
                                        sciezka.header.stamp = self.get_clock().now().to_msg()
                                        self.master_publisher.publish_path(robot_name, sciezka)
                                        self.master_publisher.set_state(robot_name, StanRobota.DOCK)
                                        self.get_logger().info(f"Zmieniono stan {robot_name} na DOCK.")
                                    else:
                                        self.get_logger().warn(f"BŁĄD trasy DOCK dla {robot_name}")
                else:
                    self.czasy_oczekiwania.pop(robot_name, None)    
            
            # --- Obsługa dotarcia do DOCK ---
            elif obecny_stan == StanRobota.DOCK:
                if robot_name == "robot1":
                    cel_x, cel_y = self.dock1
                elif robot_name == "robot2":
                    cel_x, cel_y = self.dock2
                else:
                    cel_x, cel_y = 0.0, 0.0 
                

                dist_to_target = math.sqrt((rx - cel_x)**2 + (ry - cel_y)**2)

                if dist_to_target < arrival_threshold and is_stopped:
                    self.aktywne_sciezki[robot_name] = None
                    self.master_publisher.set_state(robot_name, StanRobota.IDLE)
                    stop_msg = TwistStamped()
                    self.cmd_vel_pubs[robot_name].publish(stop_msg)
                    self.czasy_oczekiwania.pop(robot_name, None)   
                    self.get_logger().info(f"{robot_name} zaparkował w DOCK. Przełączam w stan IDLE.")     
                    
            # --- Sprawdzanie połączenia ---    
            if robot_name not in aktywne:
                if obecny_stan != StanRobota.DISCONNECTED:
                    self.get_logger().error(f"Utracono połączenie z {robot_name}! Ustawiam DISCONNECTED.")
                    self.master_publisher.set_state(robot_name, StanRobota.DISCONNECTED)
            elif robot_name in aktywne:
                if obecny_stan == StanRobota.DISCONNECTED:
                    self.get_logger().info(f"{robot_name} ponownie połączony. Ustawiam stan na NONE.")
                    self.master_publisher.set_state(robot_name, StanRobota.NONE)

        if strefa_odbioru_zajeta:
            kandydaci_do_odbioru = []

        msg = String()
        msg.data = f"Kandydaci: {str(kandydaci_do_odbioru)}" 
        self.kandydaci_pub.publish(msg)

        # --- WYBÓR ZWYCIĘZCY I PLANOWANIE STARTU ---
        if kandydaci_do_odbioru:
            # Zawsze wybieraj robot2 jeśli jest wśród kandydatów
            robot2_candidate = None
            for dist, name in kandydaci_do_odbioru:
                if name == "robot2":
                    robot2_candidate = (dist, name)
                    break
            
            if robot2_candidate:
                _, zwyciezca_nazwa = robot2_candidate
            else:
                # Jeśli robot2 nie jest w kandydatach, wybierz najbliższego
                kandydaci_do_odbioru.sort(key=lambda x: x[0])  
                _, zwyciezca_nazwa = kandydaci_do_odbioru[0]
            
            stan_zwyciezcy = self.master_publisher.states.get(zwyciezca_nazwa)
            if stan_zwyciezcy == StanRobota.IDLE:
                self.get_logger().info(f"Wybrano {zwyciezca_nazwa} do zadania ODBIOR.")
            
                pozycja = self.pozycje_robotow.get(zwyciezca_nazwa)

                if pozycja:
                    rx, ry = pozycja
                    pozycja_startowe = self.get_robot_transform(f"{zwyciezca_nazwa}_base_link")
                    if pozycja_startowe:
                        q = pozycja_startowe.transform.rotation
                        _, _, start_theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
                        cel_x, cel_y = self.odbior

                        sciezka_startowa = self.plan_safe_path(zwyciezca_nazwa, rx, ry, start_theta, cel_x, cel_y)
                    
                        if sciezka_startowa:
                            sciezka_startowa.header.stamp = self.get_clock().now().to_msg()        
                            self.master_publisher.publish_path(zwyciezca_nazwa, sciezka_startowa)
                            self.master_publisher.set_state(zwyciezca_nazwa, StanRobota.ODBIOR)
                        else:
                            self.get_logger().error(f"Nie udało się wyznaczyć ścieżki startowej dla {zwyciezca_nazwa}!")
                else:
                    self.get_logger().error(f"Brak transformacji dla {zwyciezca_nazwa}, nie mogę zaplanować startu.")

    def publish_states(self):
        self.master_publisher.publish_all_states()

    def get_robot_transform(self, robot_frame):
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'map', robot_frame, rclpy.time.Time())
            return trans
        except Exception as e:
            return None
        
    def check_robot_positions(self):     
        cel_x, cel_y = self.odbior
        for robot_name in self.robot_names:
            frame_id = f"{robot_name}_base_link"
            trans = self.get_robot_transform(frame_id)
            if trans is not None:
                tx = trans.transform.translation.x
                ty = trans.transform.translation.y
                dist = math.sqrt((tx - cel_x)**2 + (ty - cel_y)**2)
                self.odleglosci_do_odbioru[robot_name] = dist
            else:
                self.odleglosci_do_odbioru[robot_name] = None

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle
    
    def obrot_180(self, robot_name):
        trans = self.get_robot_transform(f"{robot_name}_base_link")
        if not trans:
            return False 
        
        q = trans.transform.rotation
        _, _, current_theta = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if robot_name not in self.rotation_targets:
            self.rotation_targets[robot_name] = self.normalize_angle(current_theta + math.pi/2)
            self.get_logger().info(f"{robot_name}: Start procedury obrotu o 180 st.")

        target_angle = self.rotation_targets[robot_name]
        error = self.normalize_angle(target_angle - current_theta)

        self.pub_debug_current.publish(Float32(data=float(current_theta)))
        self.pub_debug_target.publish(Float32(data=float(target_angle)))
        self.pub_debug_error.publish(Float32(data=float(error)))

        if abs(error) < 0.05:
            stop_msg = TwistStamped()
            self.cmd_vel_pubs[robot_name].publish(stop_msg)

            del self.rotation_targets[robot_name]
            return True 
        else:
            kP = 1.0
            angular_vel = kP * error
            
            max_vel = 0.5
            angular_vel = max(min(angular_vel, max_vel), -max_vel)

            min_vel = 0.15
            if abs(angular_vel) < min_vel:
                angular_vel = math.copysign(min_vel, angular_vel)

            cmd = TwistStamped()
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = angular_vel
            
            self.cmd_vel_pubs[robot_name].publish(cmd)
            
            return False 

    def check_path_intersection(self, path_msg_1, path_msg_2, collision_threshold=0.5):
            if path_msg_1 is None or path_msg_2 is None:
                return False
            
            step = 5 
            skip_points = 5

            poses_1 = path_msg_1.poses[skip_points::step]
            poses_2 = path_msg_2.poses[::step]

            for p1 in poses_1:
                x1, y1 = p1.pose.position.x, p1.pose.position.y
                for p2 in poses_2:
                    x2, y2 = p2.pose.position.x, p2.pose.position.y
                    
                    dist = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
                    
                    if dist < collision_threshold:
                        return True
            return False

    def plan_safe_path(self, robot_name, start_x, start_y, start_theta, goal_x, goal_y):
            planner = self.path_planners.get(robot_name)
            if not planner:
                return None

            # 1. Wygeneruj kandydata ścieżki
            candidate_path = planner.plan_path(start_x, start_y, start_theta, goal_x, goal_y)
            
            if not candidate_path:
                return None
            
            collision_detected = False
            step = 5
            collision_threshold_robot = 0.6
            skip_start_points = 20 

            for other_robot_name, other_path in self.aktywne_sciezki.items():
                if other_robot_name == robot_name or other_path is None:
                    continue 

                other_pos = self.pozycje_robotow.get(other_robot_name)
                
                path_to_check = other_path 
                
                if other_pos is not None:
                    curr_x, curr_y = other_pos

                    min_dist = float('inf')
                    closest_idx = 0
                    
                    for i, pose in enumerate(other_path.poses[::5]): 
                        px = pose.pose.position.x
                        py = pose.pose.position.y
                        dist = math.sqrt((px - curr_x)**2 + (py - curr_y)**2)
                        if dist < min_dist:
                            min_dist = dist
                            closest_idx = i * 5
                    
                    start_slice = max(0, closest_idx - 5)
                    
                    future_poses = other_path.poses[start_slice:]
                    
                    if not future_poses:
                        continue 

                    path_to_check = Path()
                    path_to_check.header = other_path.header
                    path_to_check.poses = future_poses

                if self.check_path_intersection(candidate_path, path_to_check, collision_threshold=0.4):
                    self.get_logger().warn(f"KOLIZJA! Ścieżka {robot_name} przecina PRZYSZŁĄ trasę {other_robot_name}.")
                    collision_detected = True
                    break
            
            if collision_detected:
                return None

            if len(candidate_path.poses) > skip_start_points:
                candidate_poses = candidate_path.poses[skip_start_points::step]
            else:
                candidate_poses = candidate_path.poses[-1:]

            for other_robot_name, position in self.pozycje_robotow.items():
                if other_robot_name == robot_name or position is None:
                    continue
                
                other_x, other_y = position 
                
                for pose in candidate_poses:
                    path_x = pose.pose.position.x
                    path_y = pose.pose.position.y
                    
                    dist = math.sqrt((path_x - other_x)**2 + (path_y - other_y)**2)
                    
                    if dist < collision_threshold_robot:
                        self.get_logger().warn(f"KOLIZJA! Ścieżka {robot_name} wchodzi w robota {other_robot_name} (dist={dist:.2f})")
                        collision_detected = True
                        break
                
                if collision_detected:
                    break
            
            if collision_detected:
                return None
            else:
                self.aktywne_sciezki[robot_name] = candidate_path
                return candidate_path
            



        
def main(args=None):
    rclpy.init(args=args)
    node = Master()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()