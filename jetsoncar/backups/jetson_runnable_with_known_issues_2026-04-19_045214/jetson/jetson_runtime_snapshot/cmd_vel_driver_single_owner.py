#!/usr/bin/env python3
"""cmd_vel -> DDSM115 RS485 motor driver with wheel odometry.

This node is the single normal owner of the DDSM115 RS485 adapter. It sends
velocity commands and reads motor RPM feedback over the same serial handle, then
publishes a wheel odometry prior for downstream odometry experiments.
"""

import math
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

import serial

LEFT_IDS = (1, 3)
RIGHT_IDS = (2, 4)


def crc8(data):
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc


def yaw_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class CmdVelDriver(Node):
    def __init__(self):
        super().__init__("cmd_vel_driver")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("wheel_odom_topic", "/wheel_odom")
        self.declare_parameter("wheel_odom_frame_id", "odom_wheel")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_wheel_odom", True)
        self.declare_parameter("publish_wheel_odom_tf", False)
        self.declare_parameter("wheel_radius", 0.050)
        self.declare_parameter("track_width", 0.400)
        self.declare_parameter("max_rpm", 80)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("command_period_sec", 0.05)
        self.declare_parameter("odom_period_sec", 0.10)
        self.declare_parameter("use_commanded_odom_fallback", True)

        self.serial_port = self.get_parameter("serial_port").value
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.odom_topic = self.get_parameter("wheel_odom_topic").value
        self.odom_frame = self.get_parameter("wheel_odom_frame_id").value
        self.base_frame = self.get_parameter("base_frame_id").value
        self.publish_odom = bool(self.get_parameter("publish_wheel_odom").value)
        self.publish_tf = bool(self.get_parameter("publish_wheel_odom_tf").value)
        self.wr = float(self.get_parameter("wheel_radius").value)
        self.tw = float(self.get_parameter("track_width").value)
        self.max_rpm = int(self.get_parameter("max_rpm").value)
        self.timeout = float(self.get_parameter("command_timeout_sec").value)
        command_period = float(self.get_parameter("command_period_sec").value)
        odom_period = float(self.get_parameter("odom_period_sec").value)
        self.use_commanded_fallback = bool(
            self.get_parameter("use_commanded_odom_fallback").value
        )

        self.ser = None
        self.lk = threading.Lock()
        self.last_cmd = time.time()
        self.target_left = 0
        self.target_right = 0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_odom_time = self.get_clock().now()
        self.last_left_vel = 0.0
        self.last_right_vel = 0.0
        self.last_left_stamp = time.time()
        self.last_right_stamp = time.time()
        self.query_count = 0
        self.query_fail = 0
        self.last_odom_source = "none"
        self.odom_publish_error_logged = False
        self.tf_publish_error_logged = False

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.03)
            self.get_logger().info("RS485 opened {}".format(self.serial_port))
        except Exception as e:
            self.get_logger().error("Cannot open RS485: {}".format(e))

        self.odom_pub = (
            self.create_publisher(Odometry, self.odom_topic, 10)
            if self.publish_odom
            else None
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Twist, self.cmd_topic, self.on_cmd, 10)
        self.create_timer(command_period, self.send_motors)
        self.create_timer(odom_period, self.update_wheel_odom)
        self.create_timer(10.0, self.log_stats)
        self.get_logger().info(
            "cmd_vel driver started | cmd={} wheel_odom={} tf={} {}->{}".format(
                self.cmd_topic,
                self.odom_topic,
                self.publish_tf,
                self.odom_frame,
                self.base_frame,
            )
        )

    def on_cmd(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        vl = v - w * self.tw / 2.0
        vr = v + w * self.tw / 2.0
        rpm_l = vl / (2.0 * math.pi * self.wr) * 60.0
        rpm_r = vr / (2.0 * math.pi * self.wr) * 60.0
        self.target_left = int(max(-self.max_rpm, min(self.max_rpm, rpm_l)))
        self.target_right = int(max(-self.max_rpm, min(self.max_rpm, rpm_r)))
        self.last_cmd = time.time()

    def command_frame(self, mid, rpm):
        rpm_bytes = struct.pack(">h", int(rpm))
        frame = bytes([mid, 0x64]) + rpm_bytes + bytes([0, 0, 0, 0, 0])
        return frame + bytes([crc8(frame)])

    def query_frame(self, mid):
        frame = bytes([mid, 0x74, 0, 0, 0, 0, 0, 0, 0])
        return frame + bytes([crc8(frame)])

    def set_rpm(self, mid, rpm):
        if not self.ser:
            return
        with self.lk:
            try:
                self.ser.write(self.command_frame(mid, rpm))
            except Exception:
                pass

    def send_motors(self):
        if time.time() - self.last_cmd > self.timeout:
            self.target_left = 0
            self.target_right = 0
        self.set_rpm(1, self.target_left)
        self.set_rpm(3, self.target_left)
        self.set_rpm(2, -self.target_right)
        self.set_rpm(4, -self.target_right)

    def query_rpm(self, mid):
        if not self.ser or not self.ser.is_open:
            return None
        self.query_count += 1
        with self.lk:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(self.query_frame(mid))
                resp = self.ser.read(10)
            except Exception:
                self.query_fail += 1
                return None
        if len(resp) == 10 and resp[0] == mid and crc8(resp[:9]) == resp[9]:
            return float(struct.unpack_from(">h", resp, 4)[0])
        self.query_fail += 1
        return None

    def side_velocity(self, ids, negate, previous, previous_stamp, commanded_rpm):
        rpms = []
        for mid in ids:
            rpm = self.query_rpm(mid)
            if rpm is not None:
                rpms.append(rpm)
        now_sec = time.time()
        if rpms:
            avg_rpm = sum(rpms) / len(rpms)
            if negate:
                avg_rpm = -avg_rpm
            return avg_rpm * 2.0 * math.pi * self.wr / 60.0, now_sec, "feedback"
        if self.use_commanded_fallback:
            cmd_vel = commanded_rpm * 2.0 * math.pi * self.wr / 60.0
            return cmd_vel, now_sec, "commanded"
        if now_sec - previous_stamp < 0.35:
            return previous, previous_stamp, "stale"
        return 0.0, now_sec, "zero"

    def update_wheel_odom(self):
        if not (self.publish_odom or self.publish_tf):
            return
        if not rclpy.ok():
            return
        if not self.ser or not self.ser.is_open:
            return

        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now
        if dt <= 0.0 or dt > 0.5:
            return

        vl, self.last_left_stamp, left_source = self.side_velocity(
            LEFT_IDS,
            False,
            self.last_left_vel,
            self.last_left_stamp,
            float(self.target_left),
        )
        vr, self.last_right_stamp, right_source = self.side_velocity(
            RIGHT_IDS,
            True,
            self.last_right_vel,
            self.last_right_stamp,
            float(self.target_right),
        )
        self.last_left_vel = vl
        self.last_right_vel = vr
        if left_source == "feedback" and right_source == "feedback":
            self.last_odom_source = "feedback"
        elif "commanded" in (left_source, right_source):
            self.last_odom_source = "commanded_fallback"
        elif "stale" in (left_source, right_source):
            self.last_odom_source = "stale"
        else:
            self.last_odom_source = "zero"

        v = (vl + vr) / 2.0
        w = (vr - vl) / self.tw

        if abs(w) < 1e-6:
            self.x += v * math.cos(self.th) * dt
            self.y += v * math.sin(self.th) * dt
        else:
            radius = v / w
            self.x += radius * (math.sin(self.th + w * dt) - math.sin(self.th))
            self.y -= radius * (math.cos(self.th + w * dt) - math.cos(self.th))
        self.th = math.atan2(math.sin(self.th + w * dt), math.cos(self.th + w * dt))

        quat = yaw_quaternion(self.th)

        if self.publish_odom and self.odom_pub:
            msg = Odometry()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = self.odom_frame
            msg.child_frame_id = self.base_frame
            msg.pose.pose.position.x = self.x
            msg.pose.pose.position.y = self.y
            msg.pose.pose.orientation = quat
            msg.twist.twist.linear.x = v
            msg.twist.twist.angular.z = w
            pose_cov = [0.0] * 36
            pose_cov[0] = 0.10
            pose_cov[7] = 0.10
            pose_cov[14] = 1e6
            pose_cov[21] = 1e6
            pose_cov[28] = 1e6
            pose_cov[35] = 0.30
            twist_cov = [0.0] * 36
            twist_cov[0] = 0.05
            twist_cov[7] = 1e6
            twist_cov[35] = 0.15
            msg.pose.covariance = pose_cov
            msg.twist.covariance = twist_cov
            try:
                self.odom_pub.publish(msg)
            except Exception as exc:
                if not self.odom_publish_error_logged:
                    self.get_logger().warning(
                        "wheel_odom publish skipped: {}".format(exc)
                    )
                    self.odom_publish_error_logged = True
                return
            self.odom_publish_error_logged = False

        if self.publish_tf and self.tf_broadcaster:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now.to_msg()
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.rotation = quat
            try:
                self.tf_broadcaster.sendTransform(tf_msg)
            except Exception as exc:
                if not self.tf_publish_error_logged:
                    self.get_logger().warning(
                        "wheel_odom TF publish skipped: {}".format(exc)
                    )
                    self.tf_publish_error_logged = True
                return
            self.tf_publish_error_logged = False

    def log_stats(self):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warning("wheel_odom inactive: RS485 not open")
            return
        fail_pct = 100.0 * self.query_fail / self.query_count if self.query_count else 0.0
        self.get_logger().info(
            "wheel_odom src={} q={} fail={:.1f}% pos=({:.3f},{:.3f}) yaw={:.2f}deg vl={:.3f} vr={:.3f}".format(
                self.last_odom_source,
                self.query_count,
                fail_pct,
                self.x,
                self.y,
                math.degrees(self.th),
                self.last_left_vel,
                self.last_right_vel,
            )
        )

    def stop_motors_once(self):
        self.target_left = 0
        self.target_right = 0
        self.send_motors()


def main():
    rclpy.init()
    node = CmdVelDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors_once()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
