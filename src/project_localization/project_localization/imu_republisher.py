#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuRepublisher(Node):
    def __init__(self):
        super().__init__('imu_republisher_node')

        # parametry z launch file
        self.declare_parameter("input_topic", "imu")
        self.declare_parameter("output_topic", "imu_ekf")
        self.declare_parameter("frame_id", "base_footprint")

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        # publisher i subscriber
        self.imu_pub = self.create_publisher(Imu, output_topic, 10)
        self.imu_sub = self.create_subscription(Imu, input_topic, self.imu_callback, 10)

        self.get_logger().info(f"Republishing {input_topic} -> {output_topic} with frame_id={self.frame_id}")

    def imu_callback(self, imu):
        imu.header.frame_id = self.frame_id
        self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = ImuRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
