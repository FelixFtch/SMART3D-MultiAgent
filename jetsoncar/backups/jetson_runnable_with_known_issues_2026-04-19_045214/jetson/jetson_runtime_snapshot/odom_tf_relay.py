#!/usr/bin/env python3
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfRelay(Node):
    def __init__(self) -> None:
        super().__init__("odom_tf_relay")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_parent = None
        self.last_child = None
        self.create_subscription(Odometry, "/odom", self.handle_odom, 10)
        self.get_logger().info("Relaying /odom into TF (odom -> base_link)")

    def handle_odom(self, msg: Odometry) -> None:
        parent = msg.header.frame_id or "odom"
        child = msg.child_frame_id or "base_link"

        if (parent, child) != (self.last_parent, self.last_child):
            self.last_parent = parent
            self.last_child = child
            self.get_logger().info(f"Using TF frames {parent} -> {child}")

        tf_msg = TransformStamped()
        tf_msg.header = msg.header
        tf_msg.header.frame_id = parent
        tf_msg.child_frame_id = child
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)


def main() -> None:
    rclpy.init()
    node = OdomTfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
