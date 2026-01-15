#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String, Bool
from enum import Enum, auto
from stan_publisher import StanRobota

class MasterSubscriber:
    def __init__(self,parent_node, robot_namespaces):
        self.node = parent_node
        self.robot_namespaces = robot_namespaces

        #Przechowywanie odkrytych stanów
        self.robot_states = {ns: None for ns in robot_namespaces}
        self.moving_status = {ns: False for ns in robot_namespaces}

        # Subscriber dla każdego robota
        self.robot_subscribers = {}
        self.moving_subscribers = {}

        for ns in robot_namespaces:
            topic = f'/{ns}/state'
            self.robot_subscribers[ns] = self.node.create_subscription(
                String,
                topic,
                lambda msg, ns=ns: self.state_callback(msg, ns),
                10
            )
            topic_moving = f'/{ns}/is_moving'
            self.moving_subscribers[ns] = self.node.create_subscription(
                Bool,
                topic_moving,
                lambda msg, ns=ns: self.moving_callback(msg, ns),
                10
            )

    def state_callback(self, msg: String, robot_ns: str):
        try:
            namespace, state_name = msg.data.split(':')
            if namespace != robot_ns:
                self.node.get_logger().warn(f"Namespace mismatch: expected {robot_ns}, got {namespace}")
                return

            #zapisanie aktulanego stanu robota do listy:
            state = StanRobota[state_name]
            self.robot_states[robot_ns] = state 
    
            #self.node.get_logger().info(f"Received state from {robot_ns}: {state.name}")
        except Exception as e:
            self.node.get_logger().error(f"Error parsing message: {msg.data} ({e})")


    def get_state(self, robot_ns: str):
        return self.robot_states.get(robot_ns, None)

    def is_in_state(self, robot_ns: str, state: StanRobota) -> bool:
        return self.robot_states.get(robot_ns) == state
    
    def moving_callback(self, msg, ns):
        if self.moving_status[ns] != msg.data:
            self.moving_status[ns] = msg.data
            self.node.get_logger().info(f"UPDATE: Robot {ns} moving: {msg.data}")

    def is_robot_moving(self, robot_name):
        return self.moving_status.get(robot_name, False)