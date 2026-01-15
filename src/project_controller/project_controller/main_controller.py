#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from odometry_controller import OdometryHandler
from navigation_controller import NavigationHandler
from path_controller import PathHandler
from dynamic_path_controller import DynamicPathHandler
from nav_msgs.msg import Path, Odometry
from tf_transformations import euler_from_quaternion

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        self.declare_parameter("robot_name", "robot1")
        self.declare_parameter("wheel_radius", 0.025)
        self.declare_parameter("wheel_separation", 0.145)

        robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value
        
        ####tu można dodawać klasy###
        self.odom = OdometryHandler(parent_node=self, robot_name=robot_name, wheel_radius=wheel_radius, wheel_separation=wheel_separation)
        self.nav = NavigationHandler(parent_node=self, robot_name=robot_name, wheel_radius=wheel_radius, wheel_separation=wheel_separation)
        self.dynamic_path = DynamicPathHandler(parent_node=self, robot_name=robot_name, wheel_radius=wheel_radius, wheel_separation=wheel_separation)
        self.path = PathHandler(parent_node=self, robot_name=robot_name, wheel_radius=wheel_radius, wheel_separation=wheel_separation)
        #############################
        
        # Subskrypcja do odometrii (opcjonalnie, dla backupu)
        self.create_subscription(
            Odometry,
            f'/{robot_name}/odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info(f"Controller for {robot_name} initialized")
    
    def odom_callback(self, msg: Odometry):
        """Aktualizuje pozycję z odometrii (opcjonalne, backup)"""
        # Ta funkcja może być przydatna jeśli chcesz mieć backupowy sposób
        # pobierania pozycji, ale głównym źródłem jest self.odom
        pass
    
    def get_robot_position(self):
        """Zwraca aktualną pozycję robota z OdometryHandler"""
        return self.odom.x_, self.odom.y_, self.odom.theta_

def main(args=None):
    rclpy.init(args=args)
    controller = None  
    try:
        controller = RobotController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("Controller stopped by user")
    except Exception as e:
        print(f"Error in controller: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if controller is not None:  
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()