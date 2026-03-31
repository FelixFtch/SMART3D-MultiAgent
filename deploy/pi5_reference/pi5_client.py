#!/usr/bin/env python3
"""
SMART-3D Pi 5 Thin Client — sensor relay + path follower

This node runs on Pi 5 inside ros2-humble container.
It does NOT do any planning. It only:
  1. Relays sensor data (already published by existing drivers)
  2. Publishes robot pose from Cartographer TF
  3. Subscribes to /carX/path from the i9 server
  4. Follows path with Pure Pursuit → /cmd_vel
  5. Emergency stop: front 0.3m → override cmd_vel to zero

CPU usage: ~0.1 core. Cartographer gets ALL the headroom it needs.

Usage:
  python3 pi5_client.py --ros-args -p car_ns:=car1 -p robot_speed:=0.2
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
import tf2_ros


class Pi5Client(Node):
    def __init__(self):
        super().__init__('pi5_client')

        self.declare_parameter('car_ns', 'car1')
        self.declare_parameter('robot_speed', 0.2)

        self.ns = self.get_parameter('car_ns').value
        self.speed = self.get_parameter('robot_speed').value

        # Robot state
        self.rx = self.ry = self.ryaw = 0.0
        self.have_pose = False

        # Path from server
        self.path = []       # [[x,y], ...]
        self.pidx = 0
        self.navigating = False

        # Emergency
        self.emergency_active = False

        # Scan (for emergency check only)
        self.last_scan = None

        # TF
        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publish pose for server (10 Hz, in nav_loop)
        self.pose_pub = self.create_publisher(
            PoseStamped, f'/{self.ns}/pose', 10)

        # Subscribe to path from server
        self.create_subscription(
            Path, f'/{self.ns}/path', self._on_path,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))

        # Subscribe to local scan (BEST_EFFORT to match M10P!)
        self.create_subscription(
            LaserScan, '/scan', self._on_scan,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Goal relay: forward local goal to server namespace
        self.goal_pub = self.create_publisher(
            PoseStamped, f'/{self.ns}/goal_pose', 10)
        self.create_subscription(
            PoseStamped, '/goal_pose', self._on_goal, 10)

        # Timers
        self.create_timer(0.1, self.nav_loop)    # 10 Hz

        self.get_logger().info(
            f'Pi5 client [{self.ns}] | speed={self.speed} | waiting for path...')

    # ---- Callbacks ----

    def _on_scan(self, msg):
        self.last_scan = msg

    def _on_goal(self, msg):
        """Forward goal from local RViz to namespaced topic for server."""
        self.goal_pub.publish(msg)
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self.get_logger().info(f'Goal forwarded: ({gx:.2f}, {gy:.2f})')

    def _on_path(self, msg):
        """Receive path from i9 server."""
        self.path = [
            [ps.pose.position.x, ps.pose.position.y]
            for ps in msg.poses
        ]
        self.pidx = 0
        self.navigating = len(self.path) > 1
        self.get_logger().info(f'Path received: {len(self.path)} pts')

    # ---- TF → Pose ----

    def _update_pose(self):
        try:
            t = self.tf_buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.rx = t.transform.translation.x
            self.ry = t.transform.translation.y
            q = t.transform.rotation
            self.ryaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.have_pose = True
        except:
            pass

    def _publish_pose(self):
        """Publish current pose for server to use."""
        if not self.have_pose:
            return
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.rx
        msg.pose.position.y = self.ry
        # Encode yaw back to quaternion
        msg.pose.orientation.z = math.sin(self.ryaw / 2.0)
        msg.pose.orientation.w = math.cos(self.ryaw / 2.0)
        self.pose_pub.publish(msg)

    # ---- Emergency Stop (local, no network dependency) ----

    def _check_emergency(self):
        """Front 0.3m arc check. ALWAYS runs, overrides server cmd_vel."""
        if self.last_scan is None:
            return False
        scan = self.last_scan
        angle = scan.angle_min
        for r in scan.ranges:
            if (abs(angle) < 0.52 and
                    scan.range_min < r < 0.30 and
                    not math.isnan(r) and not math.isinf(r)):
                return True
            angle += scan.angle_increment
        return False

    # ---- Pure Pursuit ----

    def _follow_path(self):
        if not self.path or self.pidx >= len(self.path):
            self.cmd_pub.publish(Twist())
            return

        # Skip passed waypoints
        while self.pidx < len(self.path) - 1:
            px, py = self.path[self.pidx]
            if math.sqrt((px - self.rx)**2 + (py - self.ry)**2) < 0.4:
                self.pidx += 1
            else:
                break

        # Lookahead
        la = self.pidx
        for i in range(self.pidx, min(len(self.path), self.pidx + 8)):
            d = math.sqrt((self.path[i][0] - self.rx)**2 +
                          (self.path[i][1] - self.ry)**2)
            if d >= 0.6:
                la = i
                break
        else:
            la = min(len(self.path) - 1, self.pidx + 1)

        tx, ty = self.path[la]
        dx, dy = tx - self.rx, ty - self.ry
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 0.01:
            self.cmd_pub.publish(Twist())
            return

        diff = math.atan2(dy, dx) - self.ryaw
        while diff > math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi

        cmd = Twist()
        if abs(diff) > 1.0:
            cmd.angular.z = 0.5 * (1.0 if diff > 0 else -1.0)
        elif abs(diff) > 0.4:
            cmd.linear.x = self.speed * 0.4
            cmd.angular.z = 1.5 * diff
        else:
            cmd.linear.x = self.speed
            cmd.angular.z = 1.0 * diff

        self.cmd_pub.publish(cmd)

    # ---- Main Loop (10 Hz) ----

    def nav_loop(self):
        self._update_pose()
        self._publish_pose()

        if not self.have_pose:
            return

        # Emergency stop always takes priority (local, no network)
        if self._check_emergency():
            self.cmd_pub.publish(Twist())
            if not self.emergency_active:
                self.get_logger().warn('⚠ EMERGENCY STOP')
                self.emergency_active = True
            return
        else:
            if self.emergency_active:
                self.get_logger().info('Emergency cleared')
                self.emergency_active = False

        if not self.navigating:
            return

        # Goal reached?
        if self.path:
            last = self.path[-1]
            gd = math.sqrt((last[0] - self.rx)**2 + (last[1] - self.ry)**2)
            if gd < 0.5:
                self.get_logger().info('✓ GOAL REACHED!')
                self.cmd_pub.publish(Twist())
                self.navigating = False
                self.path = []
                return

        self._follow_path()


def main():
    rclpy.init()
    node = Pi5Client()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
