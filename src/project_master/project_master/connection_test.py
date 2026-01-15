import rclpy
from std_msgs.msg import Empty, String
from functools import partial
from geometry_msgs.msg import Pose2D

class ConnectionTest:
    def __init__(self, node, robot_names, timeout_sec=10.0):
        
        self.node = node
        self.robot_names = robot_names
        self.TIMEOUT_SEC = timeout_sec

        # Zmienne stanu połączenia
        self.aktywne_roboty = set()
        self.ostatni_kontakt = {name: None for name in self.robot_names}
        self.licznik_ping = {name: 1 for name in self.robot_names}

        # Publisher listy aktywnych robotów
        self.active_list_pub = self.node.create_publisher(
            String, 
            '/master/active_robots', 
            10
        )

        # Subskrybcje pingów od robotów
        self.master_ping_subs = {}
        for robot_name in self.robot_names:
            self.master_ping_subs[robot_name] = self.node.create_subscription(
                Empty,
                f"/{robot_name}/project_controller/master_ping",
                partial(self.ping_master_callback, robot_name=robot_name),
                10
            )
        self.real_ping = self.node.create_subscription(
            Pose2D,
            f"/pos",
            partial(self.ping_master_callback, robot_name="robot1"),
            10
        )

        # Timer do sprawdzania aktywności robotów (Watchdog) - co 1.0s
        self.watchdog_timer = self.node.create_timer(1.0, self.sprawdz_aktywnosc_robotow)

    def ping_master_callback(self, msg, robot_name):
        self.ostatni_kontakt[robot_name] = self.node.get_clock().now()

        if robot_name not in self.aktywne_roboty:
            self.aktywne_roboty.add(robot_name)
            self.node.get_logger().info(f"[ConnectionTest] {robot_name} dołączył do sieci.")

        self.licznik_ping[robot_name] += 1
        if self.licznik_ping[robot_name] > 5:
            self.licznik_ping[robot_name] = 1

    def sprawdz_aktywnosc_robotow(self):
        teraz = self.node.get_clock().now()
    
        for robot_name in self.robot_names:
            ostatni_czas = self.ostatni_kontakt[robot_name]

            if ostatni_czas is None:
                continue

            roznica_czasu = (teraz - ostatni_czas).nanoseconds / 1e9

            if roznica_czasu > self.TIMEOUT_SEC and robot_name in self.aktywne_roboty:
                self.aktywne_roboty.remove(robot_name)
                self.node.get_logger().warn(f"[ConnectionTest] {robot_name} utracił połączenie! (brak pingu od {roznica_czasu:.1f}s)")

        # Publikacja aktualnej listy
        msg = String()
        msg.data = str(sorted(list(self.aktywne_roboty)))
        self.active_list_pub.publish(msg)

    def get_active_robots(self):
        return list(self.aktywne_roboty)