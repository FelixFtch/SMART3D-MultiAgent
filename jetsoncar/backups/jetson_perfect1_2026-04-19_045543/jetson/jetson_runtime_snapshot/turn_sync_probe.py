#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener


def yaw_from_q(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class TurnSyncProbe(Node):
    def __init__(self):
        super().__init__('turn_sync_probe')
        self.declare_parameter('imu_topic', '/IMU_data')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('pose_topic', '/car1/pose')
        self.declare_parameter('path_topic', '/car1/path')
        self.declare_parameter('period_sec', 0.5)

        self.imu_state = None
        self.odom_state = None
        self.pose_state = None
        self.path_state = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_tf_mb = None

        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self.on_imu, 50)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.on_odom, 20)
        self.create_subscription(
            PoseStamped, self.get_parameter('pose_topic').value, self.on_pose, 20)
        self.create_subscription(
            Path, self.get_parameter('path_topic').value, self.on_path, path_qos)

        self.create_timer(self.get_parameter('period_sec').value, self.report)

    def stamp_sec(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def on_imu(self, msg):
        self.imu_state = {
            't': self.stamp_sec(msg.header.stamp),
            'yaw': yaw_from_q(msg.orientation),
            'wz': msg.angular_velocity.z,
        }

    def on_odom(self, msg):
        self.odom_state = {
            't': self.stamp_sec(msg.header.stamp),
            'yaw': yaw_from_q(msg.pose.pose.orientation),
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'wz': msg.twist.twist.angular.z,
        }

    def on_pose(self, msg):
        self.pose_state = {
            't': self.stamp_sec(msg.header.stamp),
            'yaw': yaw_from_q(msg.pose.orientation),
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
        }

    def on_path(self, msg):
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.path_state = {
            't': self.stamp_sec(msg.header.stamp),
            'pts': pts,
        }

    def lookup_tf(self, parent, child):
        try:
            tf = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
        except Exception:
            return None
        return {
            't': self.stamp_sec(tf.header.stamp),
            'x': tf.transform.translation.x,
            'y': tf.transform.translation.y,
            'z': tf.transform.translation.z,
            'yaw': yaw_from_q(tf.transform.rotation),
        }

    def report(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if not self.pose_state:
            return

        parts = []
        if self.imu_state:
            imu_age = now - self.imu_state['t']
            parts.append(
                f"imu_age={imu_age:.2f}s imu_yaw={math.degrees(self.imu_state['yaw']):.1f} "
                f"imu_wz_deg={math.degrees(self.imu_state['wz']):.1f}"
            )
        if self.odom_state:
            odom_age = now - self.odom_state['t']
            parts.append(
                f"odom_age={odom_age:.2f}s odom_xy=({self.odom_state['x']:.2f},{self.odom_state['y']:.2f}) "
                f"odom_yaw={math.degrees(self.odom_state['yaw']):.1f} "
                f"odom_wz_deg={math.degrees(self.odom_state['wz']):.1f}"
            )
        pose_age = now - self.pose_state['t']
        parts.append(
            f"pose_age={pose_age:.2f}s pose_xy=({self.pose_state['x']:.2f},{self.pose_state['y']:.2f}) "
            f"pose_yaw={math.degrees(self.pose_state['yaw']):.1f}"
        )

        if self.imu_state and self.odom_state:
            parts.append(
                f"d_imu_odom_deg={math.degrees(wrap(self.imu_state['yaw'] - self.odom_state['yaw'])):.1f}"
            )
        if self.odom_state:
            parts.append(
                f"d_pose_odom_deg={math.degrees(wrap(self.pose_state['yaw'] - self.odom_state['yaw'])):.1f}"
            )

        if self.path_state and self.path_state['pts']:
            path_age = now - self.path_state['t']
            px, py = self.path_state['pts'][0]
            dx = px - self.pose_state['x']
            dy = py - self.pose_state['y']
            bearing = wrap(math.atan2(dy, dx) - self.pose_state['yaw'])
            parts.append(
                f"path_age={path_age:.2f}s path_n={len(self.path_state['pts'])} "
                f"path0_dist={math.hypot(dx, dy):.2f} "
                f"path0_bearing_deg={math.degrees(bearing):.1f} "
                f"path0_behind={abs(bearing) > math.pi / 2.0}"
            )

        tf_mo = self.lookup_tf('map', 'odom')
        tf_ob = self.lookup_tf('odom', 'base_link')
        tf_mb = self.lookup_tf('map', 'base_link')
        if tf_mo:
            parts.append(
                f"tf_mo_xy=({tf_mo['x']:.2f},{tf_mo['y']:.2f}) "
                f"tf_mo_yaw={math.degrees(tf_mo['yaw']):.1f} "
                f"tf_mo_age={now - tf_mo['t']:.2f}s"
            )
        if tf_ob:
            parts.append(
                f"tf_ob_xy=({tf_ob['x']:.2f},{tf_ob['y']:.2f}) "
                f"tf_ob_yaw={math.degrees(tf_ob['yaw']):.1f} "
                f"tf_ob_age={now - tf_ob['t']:.2f}s"
            )
        if tf_mb:
            radius = math.hypot(tf_mb['x'], tf_mb['y'])
            parts.append(
                f"tf_mb_xy=({tf_mb['x']:.2f},{tf_mb['y']:.2f}) "
                f"tf_mb_yaw={math.degrees(tf_mb['yaw']):.1f} "
                f"tf_mb_age={now - tf_mb['t']:.2f}s"
            )
            parts.append(f"tf_mb_r={radius:.2f}")
            if self.last_tf_mb:
                dt = max(now - self.last_tf_mb['sample_t'], 1e-6)
                dxy = math.hypot(
                    tf_mb['x'] - self.last_tf_mb['x'],
                    tf_mb['y'] - self.last_tf_mb['y'],
                )
                dyaw = math.degrees(wrap(tf_mb['yaw'] - self.last_tf_mb['yaw']))
                parts.append(f"tf_mb_dxy={dxy:.3f}")
                parts.append(f"tf_mb_dyaw={dyaw:.2f}")
                parts.append(f"tf_mb_v={dxy/dt:.2f}")
                if dxy > 0.50 or abs(dyaw) > 20.0:
                    parts.append("JUMP_DETECTED=1")
            self.last_tf_mb = {
                'x': tf_mb['x'],
                'y': tf_mb['y'],
                'yaw': tf_mb['yaw'],
                'sample_t': now,
            }

        self.get_logger().info('TurnProbe ' + ' | '.join(parts))


def main():
    rclpy.init()
    node = TurnSyncProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
