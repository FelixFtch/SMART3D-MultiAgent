#!/usr/bin/env python3
"""Hard-brake guard driven by STP23L front range data.

If /front_range stays under the trigger distance continuously for the hold
window, this node escalates to a low-level RS485 hard-brake loop. The guard is
meant to be independent of planner/network state and to override the drivetrain
when the robot has effectively pushed into a wall.
"""

from __future__ import annotations

import math
import struct
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
import serial


def crc8_maxim(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
            crc &= 0xFF
    return crc


def brake_frame(motor_id: int, brake: bool) -> bytes:
    brake_byte = 0xFF if brake else 0x00
    body = bytes([motor_id, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, brake_byte, 0x00])
    return body + bytes([crc8_maxim(body)])


class STP23LHardBrakeGuard(Node):
    def __init__(self) -> None:
        super().__init__("stp23l_hard_brake_guard")

        self.declare_parameter("range_topic", "/front_range")
        self.declare_parameter("trigger_distance_m", 0.05)
        self.declare_parameter("hold_time_sec", 10.0)
        self.declare_parameter("clear_distance_m", 0.08)
        self.declare_parameter("clear_time_sec", 1.0)
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("brake_period_sec", 0.20)
        self.declare_parameter("publish_zero_cmd_vel", True)
        self.declare_parameter("stop_rs485_owners_on_trigger", True)

        self.range_topic = str(self.get_parameter("range_topic").value)
        self.trigger_distance_m = float(self.get_parameter("trigger_distance_m").value)
        self.hold_time_sec = float(self.get_parameter("hold_time_sec").value)
        self.clear_distance_m = float(self.get_parameter("clear_distance_m").value)
        self.clear_time_sec = float(self.get_parameter("clear_time_sec").value)
        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.brake_period_sec = float(self.get_parameter("brake_period_sec").value)
        self.publish_zero_cmd_vel = bool(self.get_parameter("publish_zero_cmd_vel").value)
        self.stop_rs485_owners_on_trigger = bool(
            self.get_parameter("stop_rs485_owners_on_trigger").value
        )

        self.zero_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.state_pub = self.create_publisher(Bool, "/stp23l_hard_brake_active", 10)
        self.create_subscription(Range, self.range_topic, self._on_range, qos_profile_sensor_data)

        self.low_since: float | None = None
        self.clear_since: float | None = None
        self.last_range_stamp: float | None = None
        self.last_range: float | None = None
        self.active = False
        self.ser: serial.Serial | None = None

        self.create_timer(self.brake_period_sec, self._tick)
        self.get_logger().info(
            "STP23L hard-brake guard ready: "
            f"topic={self.range_topic} trigger={self.trigger_distance_m:.3f}m "
            f"hold={self.hold_time_sec:.1f}s port={self.serial_port}"
        )

    def _on_range(self, msg: Range) -> None:
        now = time.monotonic()
        dist = float(msg.range)
        self.last_range_stamp = now
        self.last_range = dist

        valid = (
            not math.isnan(dist)
            and not math.isinf(dist)
            and msg.min_range <= dist <= msg.max_range
        )
        if not valid:
            self.low_since = None
            return

        if dist <= self.trigger_distance_m:
            if self.low_since is None:
                self.low_since = now
                self.get_logger().warn(
                    f"Front range under trigger: {dist:.3f}m <= {self.trigger_distance_m:.3f}m"
                )
            self.clear_since = None
            if not self.active and now - self.low_since >= self.hold_time_sec:
                self._activate()
            return

        self.low_since = None

        if self.active and dist >= self.clear_distance_m:
            if self.clear_since is None:
                self.clear_since = now
            elif now - self.clear_since >= self.clear_time_sec:
                self._deactivate()
        else:
            self.clear_since = None

    def _activate(self) -> None:
        if self.active:
            return
        self.active = True
        self.clear_since = None
        self.get_logger().error(
            "STP23L hard-brake triggered after sustained close-range wall contact"
        )
        if self.stop_rs485_owners_on_trigger:
            subprocess.run("pkill -f '[c]md_vel_driver.py' 2>/dev/null || true", shell=True)
            subprocess.run("pkill -f '[d]dsm115_odom.py' 2>/dev/null || true", shell=True)

    def _close_serial(self) -> None:
        if self.ser is None:
            return
        try:
            if self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    def _deactivate(self) -> None:
        if not self.active:
            return
        self.active = False
        self.low_since = None
        self.clear_since = None
        self._close_serial()
        self.get_logger().info(
            "STP23L hard-brake cleared after front range moved above clear distance"
        )

    def _ensure_serial(self) -> bool:
        if self.ser is not None and self.ser.is_open:
            return True
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.02)
            return True
        except Exception as exc:
            self.get_logger().error(f"Failed opening brake port {self.serial_port}: {exc}")
            self.ser = None
            return False

    def _send_brake_burst(self) -> None:
        if not self._ensure_serial():
            return
        zero_frames = [brake_frame(mid, False) for mid in range(1, 5)]
        brake_frames = [brake_frame(mid, True) for mid in range(1, 5)]
        try:
            for _ in range(4):
                for pkt in zero_frames:
                    self.ser.write(pkt)
                time.sleep(0.01)
                for pkt in brake_frames:
                    self.ser.write(pkt)
                self.ser.flush()
                time.sleep(0.02)
        except Exception as exc:
            self.get_logger().error(f"Brake write failed: {exc}")
            self._close_serial()

    def _tick(self) -> None:
        state = Bool()
        state.data = self.active
        self.state_pub.publish(state)

        if not self.active:
            return

        if self.publish_zero_cmd_vel:
            self.zero_pub.publish(Twist())

        self._send_brake_burst()

        if self.last_range_stamp is not None:
            age = time.monotonic() - self.last_range_stamp
            if age > 1.0:
                self.get_logger().warn(
                    f"Hard-brake active and /front_range is stale for {age:.1f}s"
                )

    def destroy_node(self) -> bool:
        self._close_serial()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = STP23LHardBrakeGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
