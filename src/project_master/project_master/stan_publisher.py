#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from enum import Enum, auto

class StanRobota(Enum):
    NONE = auto()
    IDLE = auto()
    BUSY = auto()
    ODBIOR = auto()
    DOWOZ = auto()
    DOCK = auto()
    STOP = auto()
    OBROT = auto()
    DISCONNECTED = auto()

class MasterPublisher():
    def __init__(self, parent_node, robot_names):
        self.node = parent_node
        self.robot_names = robot_names
        self.states = {name: StanRobota.NONE for name in robot_names}

        # Publisher dla każdego robota
        self.robot_publishers = {}
        self.path_publishers = {}

        for ns in robot_names:
            topic = f'/{ns}/state'
            self.robot_publishers[ns] = self.node.create_publisher(String, topic, 10)
            topic_path = f'/{ns}/planned_path'
            self.path_publishers[ns] = self.node.create_publisher(Path, topic_path, 10)

    def publish_all_states(self):
        for robot_ns, state in self.states.items():
            self._publish_single(robot_ns, state)

    def _publish_single(self, robot_ns, state: StanRobota):
        if robot_ns in self.robot_publishers:
            msg = String()
            msg.data = f"{robot_ns}:{state.name}"
            self.robot_publishers[robot_ns].publish(msg)

    def set_state(self, robot_ns, state: StanRobota):
        if robot_ns not in self.robot_publishers:
            self.node.get_logger().warn(f"Unknown robot namespace: {robot_ns}")
            return

        self.states[robot_ns] = state

        msg = String()
        msg.data = f"{robot_ns}:{state.name}"
        self.robot_publishers[robot_ns].publish(msg)
        self.node.get_logger().info(f"Ustawiam stan {robot_ns} na {state.name}.")

    def publish_path(self, robot_ns, path_msg):
        if robot_ns in self.path_publishers:
            self.path_publishers[robot_ns].publish(path_msg)
            self.node.get_logger().info(f"Opublikowano ścieżkę dla {robot_ns}")
        else:
            self.node.get_logger().warn(f"Nie znaleziono publishera ścieżki dla: {robot_ns}")
