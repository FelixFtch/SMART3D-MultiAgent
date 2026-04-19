#!/usr/bin/env python3
"""
SMART-3D Pi 5 Thin Client — sensor relay + path follower

This node runs on Pi 5 inside ros2-humble container.
It does NOT do any planning. It only:
  1. Relays sensor data (already published by existing drivers)
  2. Publishes robot pose from Cartographer TF
  3. Subscribes to /carX/path from the i9 server
  4. Follows path with Pure Pursuit → /cmd_vel
  5. Emergency stop: front arc within configurable stop distance → override cmd_vel to zero

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
        self.declare_parameter('waypoint_reached_dist', 0.20)
        self.declare_parameter('lookahead_dist', 0.35)
        self.declare_parameter('turn_only_angle', 0.85)
        self.declare_parameter('slow_turn_angle', 0.30)
        self.declare_parameter('emergency_stop_dist', 0.30)
        self.declare_parameter('campus_mode', False)
        self.declare_parameter('debug_path_probe', True)
        self.declare_parameter('debug_path_probe_period', 1.0)
        self.declare_parameter('debug_stale_path_sec', 2.0)
        self.declare_parameter('path_anchor_search_points', 64)

        self.ns = self.get_parameter('car_ns').value
        self.speed = self.get_parameter('robot_speed').value
        self.waypoint_reached_dist = self.get_parameter('waypoint_reached_dist').value
        self.lookahead_dist = self.get_parameter('lookahead_dist').value
        self.turn_only_angle = self.get_parameter('turn_only_angle').value
        self.slow_turn_angle = self.get_parameter('slow_turn_angle').value
        self.emergency_stop_dist = self.get_parameter('emergency_stop_dist').value
        self.campus_mode = bool(self.get_parameter('campus_mode').value)
        self.debug_path_probe = bool(self.get_parameter('debug_path_probe').value)
        self.debug_path_probe_period = float(self.get_parameter('debug_path_probe_period').value)
        self.debug_stale_path_sec = float(self.get_parameter('debug_stale_path_sec').value)
        self.path_anchor_search_points = int(
            self.get_parameter('path_anchor_search_points').value
        )

        # Robot state
        self.rx = self.ry = self.ryaw = 0.0
        self.have_pose = False

        # Path from server
        self.path = []       # [[x,y], ...]
        self.pidx = 0
        self.navigating = False
        self.path_stamp_sec = 0.0
        self.last_path_probe_sec = 0.0

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
        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=(ReliabilityPolicy.BEST_EFFORT
                         if self.campus_mode else ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            Path, f'/{self.ns}/path', self._on_path, path_qos)

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
            f'Pi5 client [{self.ns}] | speed={self.speed} | '
            f'lookahead={self.lookahead_dist:.2f} | '
            f'waypoint={self.waypoint_reached_dist:.2f} | '
            f'estop={self.emergency_stop_dist:.2f} | waiting for path...')
        if self.campus_mode:
            self.get_logger().info(
                'Campus mode enabled: path subscription uses BEST_EFFORT')

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
        self.path_stamp_sec = (
            float(msg.header.stamp.sec) +
            float(msg.header.stamp.nanosec) * 1e-9
        )
        self.pidx = 0
        if self.have_pose and self.path:
            self.pidx = self._best_path_anchor_index(0)
        self.navigating = len(self.path) > 1
        age = self._path_age_sec()
        self.get_logger().info(
            f'Path received: {len(self.path)} pts | age={age:.2f}s')

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
        """Front arc check. ALWAYS runs, overrides server cmd_vel."""
        if self.last_scan is None:
            return False
        scan = self.last_scan
        trigger_dist = max(float(self.emergency_stop_dist), float(scan.range_min))
        angle = scan.angle_min
        for r in scan.ranges:
            if (abs(angle) < 0.52 and
                    0.0 < r <= trigger_dist and
                    not math.isnan(r) and not math.isinf(r)):
                return True
            angle += scan.angle_increment
        return False

    def _path_age_sec(self):
        if self.path_stamp_sec <= 0.0:
            return float('inf')
        now = self.get_clock().now().nanoseconds / 1e9
        return max(0.0, now - self.path_stamp_sec)

    def _wrap_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _path_probe(self, lookahead_idx):
        if not self.debug_path_probe or not self.have_pose or not self.path:
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_path_probe_sec < self.debug_path_probe_period:
            return
        self.last_path_probe_sec = now_sec

        first_x, first_y = self.path[0]
        first_dx = first_x - self.rx
        first_dy = first_y - self.ry
        first_dist = math.hypot(first_dx, first_dy)
        first_bearing = self._wrap_angle(math.atan2(first_dy, first_dx) - self.ryaw)
        first_behind = abs(first_bearing) > (math.pi / 2.0)

        near_idx = min(
            range(len(self.path)),
            key=lambda i: (self.path[i][0] - self.rx) ** 2 + (self.path[i][1] - self.ry) ** 2,
        )
        near_x, near_y = self.path[near_idx]
        near_dist = math.hypot(near_x - self.rx, near_y - self.ry)

        la_idx = max(0, min(lookahead_idx, len(self.path) - 1))
        la_x, la_y = self.path[la_idx]
        la_dist = math.hypot(la_x - self.rx, la_y - self.ry)
        la_bearing = self._wrap_angle(math.atan2(la_y - self.ry, la_x - self.rx) - self.ryaw)

        age = self._path_age_sec()
        stale = age > self.debug_stale_path_sec
        self.get_logger().info(
            'PathProbe '
            f'age={age:.2f}s stale={stale} '
            f'first_idx=0 first_dist={first_dist:.2f} first_bearing_deg={math.degrees(first_bearing):.1f} '
            f'first_behind={first_behind} '
            f'near_idx={near_idx} near_dist={near_dist:.2f} '
            f'pidx={self.pidx} la_idx={la_idx} la_dist={la_dist:.2f} '
            f'la_bearing_deg={math.degrees(la_bearing):.1f}')

    def _best_path_anchor_index(self, start_idx=0):
        if not self.have_pose or not self.path:
            return max(0, min(start_idx, len(self.path) - 1))

        start_idx = max(0, min(start_idx, len(self.path) - 1))
        end_idx = min(len(self.path), start_idx + max(1, self.path_anchor_search_points))

        best_any_idx = start_idx
        best_any_dist = float('inf')
        best_forward_idx = None
        best_forward_dist = float('inf')

        for i in range(start_idx, end_idx):
            px, py = self.path[i]
            dx = px - self.rx
            dy = py - self.ry
            dist = math.hypot(dx, dy)
            bearing = self._wrap_angle(math.atan2(dy, dx) - self.ryaw)
            behind = abs(bearing) > (math.pi / 2.0)

            if dist < best_any_dist:
                best_any_dist = dist
                best_any_idx = i

            if not behind and dist < best_forward_dist:
                best_forward_dist = dist
                best_forward_idx = i

        if best_forward_idx is not None:
            return best_forward_idx
        return best_any_idx

    # ---- Pure Pursuit ----

    def _follow_path(self):
        if not self.path or self.pidx >= len(self.path):
            self.cmd_pub.publish(Twist())
            return

        # Skip passed waypoints
        while self.pidx < len(self.path) - 1:
            px, py = self.path[self.pidx]
            if math.sqrt((px - self.rx)**2 + (py - self.ry)**2) < self.waypoint_reached_dist:
                self.pidx += 1
            else:
                break

        anchor_idx = self._best_path_anchor_index(self.pidx)
        if anchor_idx > self.pidx:
            self.pidx = anchor_idx

        # Lookahead
        la = self.pidx
        for i in range(self.pidx, min(len(self.path), self.pidx + 8)):
            d = math.sqrt((self.path[i][0] - self.rx)**2 +
                          (self.path[i][1] - self.ry)**2)
            if d >= self.lookahead_dist:
                la = i
                break
        else:
            la = min(len(self.path) - 1, self.pidx + 1)

        self._path_probe(la)

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
        if abs(diff) > self.turn_only_angle:
            cmd.angular.z = 0.45 * (1.0 if diff > 0 else -1.0)
        elif abs(diff) > self.slow_turn_angle:
            cmd.linear.x = self.speed * 0.22
            cmd.angular.z = 1.2 * diff
        else:
            cmd.linear.x = self.speed
            cmd.angular.z = 0.8 * diff

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
