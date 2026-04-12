#!/usr/bin/env python3
"""
SMART-3D M3 PRO Thin Client — sensor relay + path follower + depth safety

Runs natively on Jetson Orin NX (no Docker).
Requires: start_agent.sh already running (micro-ROS for STM32 base),
          Cartographer SLAM active (providing /map + TF map→base_link),
          CycloneDDS configured for cross-network comm with i9.

This node does NOT plan. It only:
  1. Publishes robot pose from Cartographer TF  → /{ns}/pose
  2. Subscribes to /{ns}/path from i9 server
  3. Follows path with Pure Pursuit             → /cmd_vel
  4. Emergency stop: LiDAR front arc 0.30m OR depth center 0.33m
  5. Relays local /scan to /{ns}/scan for i9 planner's dynamic obs detection

CPU usage: ~0.1 core. Cartographer gets all headroom.

Usage:
  python3 m3pro_client.py --ros-args -p car_ns:=m3pro

  With custom speed:
  python3 m3pro_client.py --ros-args -p car_ns:=m3pro -p robot_speed:=0.25
"""

import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Path
import tf2_ros


class M3ProClient(Node):
    def __init__(self):
        super().__init__('m3pro_client')

        # ---- Parameters ----
        self.declare_parameter('car_ns', 'm3pro')
        self.declare_parameter('robot_speed', 0.25)
        self.declare_parameter('emergency_lidar_m', 0.30)
        self.declare_parameter('emergency_lidar_arc_rad', 0.52)  # ±30°
        self.declare_parameter('emergency_depth_m', 0.33)
        self.declare_parameter('depth_roi_px', 12)
        self.declare_parameter('depth_scale', 0.001)        # mm → m
        self.declare_parameter('goal_reached_m', 0.45)
        self.declare_parameter('waypoint_skip_m', 0.35)
        self.declare_parameter('lookahead_m', 0.55)
        self.declare_parameter('scan_relay', True)           # relay /scan → /{ns}/scan

        self.ns = self.get_parameter('car_ns').value
        self.speed = self.get_parameter('robot_speed').value
        self.e_lidar_m = self.get_parameter('emergency_lidar_m').value
        self.e_lidar_arc = self.get_parameter('emergency_lidar_arc_rad').value
        self.e_depth_m = self.get_parameter('emergency_depth_m').value
        self.depth_roi_px = self.get_parameter('depth_roi_px').value
        self.depth_scale = self.get_parameter('depth_scale').value
        self.goal_reached_m = self.get_parameter('goal_reached_m').value
        self.waypoint_skip_m = self.get_parameter('waypoint_skip_m').value
        self.lookahead_m = self.get_parameter('lookahead_m').value
        self.scan_relay = self.get_parameter('scan_relay').value

        # ---- Robot state ----
        self.rx = self.ry = self.ryaw = 0.0
        self.have_pose = False

        # ---- Path from i9 server ----
        self.path = []       # [[x, y], ...]
        self.pidx = 0
        self.navigating = False

        # ---- Emergency ----
        self.emergency_lidar = False
        self.emergency_depth = False
        self.emergency_active = False

        # ---- Sensor cache ----
        self.last_scan = None
        self.depth_center_m = float('inf')

        # ---- TF ----
        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)

        # ---- Publishers ----
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, f'/{self.ns}/pose', 10)

        # Scan relay: so i9 planner can detect dynamic obstacles near this car
        if self.scan_relay:
            self.scan_relay_pub = self.create_publisher(
                LaserScan, f'/{self.ns}/scan',
                QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        else:
            self.scan_relay_pub = None

        # ---- Subscribers ----
        # Path from i9 server
        self.create_subscription(
            Path, f'/{self.ns}/path', self._on_path,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))

        # Local scan (BEST_EFFORT to match typical LiDAR QoS)
        self.create_subscription(
            LaserScan, '/scan', self._on_scan,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Depth camera (Dabai DCW2 on M3 PRO)
        self.create_subscription(
            Image, '/camera/depth/image_raw', self._on_depth, 10)

        # Goal relay: forward local 2D Goal Pose to server namespace
        self.goal_pub = self.create_publisher(
            PoseStamped, f'/{self.ns}/goal_pose', 10)
        self.create_subscription(
            PoseStamped, '/goal_pose', self._on_goal, 10)

        # ---- Timers ----
        self.create_timer(0.1, self.nav_loop)    # 10 Hz

        self.get_logger().info(
            f'M3Pro client [{self.ns}] | speed={self.speed} '
            f'| e_lidar={self.e_lidar_m}m e_depth={self.e_depth_m}m '
            f'| waiting for path...')

    # ========== Callbacks ==========

    def _on_scan(self, msg):
        self.last_scan = msg
        # Relay scan to namespaced topic for i9 planner
        if self.scan_relay_pub is not None:
            relay = LaserScan()
            relay.header = msg.header
            relay.header.frame_id = f'{self.ns}/laser_link'
            relay.angle_min = msg.angle_min
            relay.angle_max = msg.angle_max
            relay.angle_increment = msg.angle_increment
            relay.time_increment = msg.time_increment
            relay.scan_time = msg.scan_time
            relay.range_min = msg.range_min
            relay.range_max = msg.range_max
            relay.ranges = msg.ranges
            relay.intensities = msg.intensities
            self.scan_relay_pub.publish(relay)

    def _on_depth(self, msg):
        """Extract center ROI median distance for emergency stop."""
        try:
            H, W = msg.height, msg.width
            enc = (msg.encoding or '').lower()

            if '32f' in enc:
                depth = np.frombuffer(msg.data, dtype=np.float32).reshape(H, W)
            else:
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(H, W)
                depth = arr.astype(np.float32) * self.depth_scale

            k = max(2, self.depth_roi_px // 2)
            cy, cx = H // 2, W // 2
            roi = depth[cy - k:cy + k, cx - k:cx + k]
            valid = roi[np.isfinite(roi) & (roi > 0.05) & (roi < 6.0)]

            if valid.size > 0:
                self.depth_center_m = float(np.median(valid))
            else:
                self.depth_center_m = float('inf')
        except Exception:
            self.depth_center_m = float('inf')

    def _on_goal(self, msg):
        """Forward goal from local RViz to namespaced topic for i9 server."""
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

    # ========== TF → Pose ==========

    def _update_pose(self):
        try:
            t = self.tf_buf.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            self.rx = t.transform.translation.x
            self.ry = t.transform.translation.y
            q = t.transform.rotation
            self.ryaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.have_pose = True
        except Exception:
            pass

    def _publish_pose(self):
        if not self.have_pose:
            return
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.rx
        msg.pose.position.y = self.ry
        msg.pose.orientation.z = math.sin(self.ryaw / 2.0)
        msg.pose.orientation.w = math.cos(self.ryaw / 2.0)
        self.pose_pub.publish(msg)

    # ========== Emergency Stop (local, no network dependency) ==========

    def _check_emergency_lidar(self):
        """Front arc LiDAR check. Runs locally, no network."""
        if self.last_scan is None:
            return False
        scan = self.last_scan
        angle = scan.angle_min
        count = 0
        for r in scan.ranges:
            if (abs(angle) < self.e_lidar_arc and
                    scan.range_min < r < self.e_lidar_m and
                    not math.isnan(r) and not math.isinf(r)):
                count += 1
                if count >= 3:  # need ≥3 points to avoid false triggers
                    return True
            angle += scan.angle_increment
        return False

    def _check_emergency_depth(self):
        """Depth camera center check. Runs locally, no network."""
        return self.depth_center_m < self.e_depth_m

    # ========== Pure Pursuit ==========

    def _follow_path(self):
        if not self.path or self.pidx >= len(self.path):
            self.cmd_pub.publish(Twist())
            return

        # Skip passed waypoints
        while self.pidx < len(self.path) - 1:
            px, py = self.path[self.pidx]
            if math.sqrt((px - self.rx)**2 + (py - self.ry)**2) < self.waypoint_skip_m:
                self.pidx += 1
            else:
                break

        # Lookahead: find a point ~lookahead_m ahead
        la = self.pidx
        for i in range(self.pidx, min(len(self.path), self.pidx + 10)):
            d = math.sqrt((self.path[i][0] - self.rx)**2 +
                          (self.path[i][1] - self.ry)**2)
            if d >= self.lookahead_m:
                la = i
                break
        else:
            la = min(len(self.path) - 1, self.pidx + 1)

        tx, ty = self.path[la]
        dx, dy = tx - self.rx, ty - self.ry
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < 0.01:
            self.cmd_pub.publish(Twist())
            return

        # Angle error
        diff = math.atan2(dy, dx) - self.ryaw
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi

        cmd = Twist()
        if abs(diff) > 1.0:
            # Large angle error: rotate in place
            cmd.angular.z = 0.6 * (1.0 if diff > 0 else -1.0)
        elif abs(diff) > 0.4:
            # Medium angle: slow forward + turn
            cmd.linear.x = self.speed * 0.4
            cmd.angular.z = 1.5 * diff
        else:
            # Small angle: full speed + gentle correction
            cmd.linear.x = self.speed
            cmd.angular.z = 1.0 * diff

        self.cmd_pub.publish(cmd)

    # ========== Main Loop (10 Hz) ==========

    def nav_loop(self):
        self._update_pose()
        self._publish_pose()

        if not self.have_pose:
            return

        # Emergency stop: LiDAR OR depth — always local, no network
        e_lidar = self._check_emergency_lidar()
        e_depth = self._check_emergency_depth()
        emergency = e_lidar or e_depth

        if emergency:
            self.cmd_pub.publish(Twist())
            if not self.emergency_active:
                reason = []
                if e_lidar:
                    reason.append(f'LiDAR<{self.e_lidar_m}m')
                if e_depth:
                    reason.append(f'depth={self.depth_center_m:.2f}m')
                self.get_logger().warn(
                    f'⚠ EMERGENCY STOP [{", ".join(reason)}]')
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
            gd = math.sqrt((last[0] - self.rx)**2 +
                           (last[1] - self.ry)**2)
            if gd < self.goal_reached_m:
                self.get_logger().info('✓ GOAL REACHED!')
                self.cmd_pub.publish(Twist())
                self.navigating = False
                self.path = []
                return

        self._follow_path()


def main():
    rclpy.init()
    node = M3ProClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Always stop motors on exit
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
