#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class QoSChecker(Node):
    def __init__(self):
        super().__init__('qos_checker')
        info = self.get_publishers_info_by_topic('/mavros/local_position/odom')
        for i, publisher_info in enumerate(info):
            self.get_logger().info(f"Publisher {i}:")
            self.get_logger().info(f"  QoS: Reliability: {publisher_info.qos_profile.reliability}")
            self.get_logger().info(f"       Durability: {publisher_info.qos_profile.durability}")
            self.get_logger().info(f"       History: {publisher_info.qos_profile.history}")
            self.get_logger().info(f"       Depth: {publisher_info.qos_profile.depth}")

def main():
    rclpy.init()
    node = QoSChecker()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
