#!/usr/bin/env python3
"""cmd_vel -> DDSM115 RS485 motor driver for differential drive.
Subscribes /cmd_vel, sends RPM commands to 4 motors via RS485.
Motor IDs: Left front=1, Left rear=3, Right front=2, Right rear=4
Right side reversed. Wheel radius=0.05m, track width=0.4m.
"""
import math
import struct
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

MAX_RPM = 80
TIMEOUT = 0.5


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


class CmdVelDriver(Node):
    def __init__(self):
        super().__init__('cmd_vel_driver')
        self.wr = 0.050
        self.tw = 0.400
        self.ser = None
        self.lk = threading.Lock()
        self.last_cmd = time.time()
        self.target_left = 0
        self.target_right = 0

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.01)
            self.get_logger().info('RS485 opened /dev/ttyUSB0')
        except Exception as e:
            self.get_logger().error('Cannot open RS485: {}'.format(e))

        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_timer(0.05, self.send_motors)
        self.get_logger().info('cmd_vel driver started')

    def on_cmd(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        vl = v - w * self.tw / 2
        vr = v + w * self.tw / 2
        rpm_l = vl / (2 * math.pi * self.wr) * 60
        rpm_r = vr / (2 * math.pi * self.wr) * 60
        self.target_left = int(max(-MAX_RPM, min(MAX_RPM, rpm_l)))
        self.target_right = int(max(-MAX_RPM, min(MAX_RPM, rpm_r)))
        self.last_cmd = time.time()

    def set_rpm(self, mid, rpm):
        if not self.ser:
            return
        rpm_bytes = struct.pack('>h', int(rpm))
        frame = bytes([mid, 0x64]) + rpm_bytes + bytes([0, 0, 0, 0, 0])
        frame = frame + bytes([crc8(frame)])
        with self.lk:
            try:
                self.ser.write(frame)
            except Exception:
                pass

    def send_motors(self):
        if time.time() - self.last_cmd > TIMEOUT:
            self.target_left = 0
            self.target_right = 0
        self.set_rpm(1, self.target_left)
        self.set_rpm(3, self.target_left)
        self.set_rpm(2, -self.target_right)
        self.set_rpm(4, -self.target_right)


def main():
    rclpy.init()
    node = CmdVelDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.target_left = 0
        node.target_right = 0
        node.send_motors()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
