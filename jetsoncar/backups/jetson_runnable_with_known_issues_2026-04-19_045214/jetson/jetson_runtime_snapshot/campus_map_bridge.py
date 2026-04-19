#!/usr/bin/env python3
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class CampusMapBridge(Node):
    def __init__(self):
        super().__init__('campus_map_bridge')
        self.declare_parameter('input_topic', '/map')
        self.declare_parameter('output_topic', '/map_campus')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        sub_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        pub_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.pub = self.create_publisher(OccupancyGrid, output_topic, pub_qos)
        self.sub = self.create_subscription(
            OccupancyGrid, input_topic, self._on_map, sub_qos
        )
        self.count = 0
        self.get_logger().info(
            f'Campus map bridge: {input_topic} -> {output_topic} '
            '(RELIABLE local -> BEST_EFFORT campus)'
        )

    def _on_map(self, msg):
        self.pub.publish(msg)
        self.count += 1
        if self.count == 1 or self.count % 20 == 0:
            self.get_logger().info(
                f'Bridged map #{self.count} ({msg.info.width}x{msg.info.height})'
            )


def main():
    rclpy.init()
    node = CampusMapBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
