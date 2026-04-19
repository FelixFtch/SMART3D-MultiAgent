#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

class ImuQosBridge(Node):
    def __init__(self):
        super().__init__("imu_qos_bridge")
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)
        pub_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(Imu, "/imu_reliable", pub_qos)
        self.sub = self.create_subscription(Imu, "/IMU_data", self._cb, sub_qos)
        self.count = 0
        self.get_logger().info("Bridge started: /IMU_data (BEST_EFFORT) -> /imu_reliable (RELIABLE)")
    def _cb(self, msg):
        self.pub.publish(msg)
        self.count += 1
        if self.count % 500 == 0:
            self.get_logger().info(f"Bridged {self.count} IMU msgs")

def main():
    rclpy.init()
    node = ImuQosBridge()
    try:
        rclpy.spin(node)
    except:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
