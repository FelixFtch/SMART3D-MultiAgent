#!/usr/bin/env python3
"""
STP-23L Single-Point Range Sensor ROS2 Node
============================================
Reads DTOF distance data from LDROBOT STP-23L via UART and publishes
sensor_msgs/Range for front obstacle detection / emergency stop.

Hardware: STP-23L on /dev/ttyACM0 (serial 5A6C069316)
Protocol: UART 230400 8N1, frame header 0xAAAAAAAAAA
Output:   /front_range (sensor_msgs/Range) at ~120 Hz

Usage:
  python3 /root/stp23l_range.py --ros-args \
    -p serial_port:=/dev/ttyACM0 \
    -p baud_rate:=230400 \
    -p frame_id:=stp23l_link \
    -p min_range:=0.03 \
    -p max_range:=7.5 \
    -p emergency_stop_dist:=0.25
"""
import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
import serial


class STP23LNode(Node):
    """Parse STP-23L DTOF frames and publish Range + emergency stop flag."""

    # Protocol constants
    HEADER = b'\xaa\xaa\xaa\xaa'
    CMD_DISTANCE = 0x02

    # Each measurement point: distance(2) + noise(2) + peak(4) +
    #   confidence(1) + intg(4) + reftof(2) = 15 bytes
    POINT_SIZE = 15
    POINTS_PER_FRAME = 12
    TIMESTAMP_SIZE = 4
    # Total data length = 12*15 + 4 = 184 = 0xB8
    EXPECTED_DATA_LEN = POINTS_PER_FRAME * POINT_SIZE + TIMESTAMP_SIZE

    def __init__(self):
        super().__init__('stp23l_range')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 230400)
        self.declare_parameter('frame_id', 'stp23l_link')
        self.declare_parameter('min_range', 0.03)
        self.declare_parameter('max_range', 7.5)
        self.declare_parameter('emergency_stop_dist', 0.25)
        self.declare_parameter('topic', '/front_range')
        self.declare_parameter('confidence_threshold', 50)

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.estop_dist = self.get_parameter('emergency_stop_dist').value
        topic = self.get_parameter('topic').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value

        # Publishers
        self.range_pub = self.create_publisher(Range, topic, qos_profile_sensor_data)
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)

        # Serial port
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(
                f'STP-23L connected: {self.port} @ {self.baud}')
        except Exception as e:
            self.get_logger().error(f'Cannot open {self.port}: {e}')
            raise

        # Buffer for accumulating serial data
        self.buf = bytearray()

        # Timer: process serial at 200 Hz
        self.create_timer(0.005, self._read_serial)

    def _read_serial(self):
        """Read available serial bytes and parse frames."""
        try:
            avail = self.ser.in_waiting
            if avail > 0:
                self.buf.extend(self.ser.read(min(avail, 1024)))
        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')
            return

        # Parse all complete frames in buffer
        while True:
            # Find header 0xAAAAAAAA
            idx = self.buf.find(self.HEADER)
            if idx < 0:
                # Keep last 3 bytes (partial header possible)
                self.buf = self.buf[-3:] if len(self.buf) > 3 else self.buf
                break

            # Discard bytes before header
            if idx > 0:
                self.buf = self.buf[idx:]

            # Need at least: header(4) + addr(1) + cmd(1) + offset(2) +
            #   len(2) = 10 bytes for the header portion
            if len(self.buf) < 10:
                break

            cmd = self.buf[5]
            data_len = self.buf[8] | (self.buf[9] << 8)

            # Total frame length: 10 (header portion) + data_len + 1 (checksum)
            frame_len = 10 + data_len + 1

            if len(self.buf) < frame_len:
                break  # Wait for more data

            # Extract frame
            frame = bytes(self.buf[:frame_len])
            self.buf = self.buf[frame_len:]

            # Only process distance data frames
            if cmd == self.CMD_DISTANCE and data_len == self.EXPECTED_DATA_LEN:
                self._parse_distance_frame(frame)

    def _parse_distance_frame(self, frame):
        """Parse 12 measurement points, publish median valid distance."""
        data_start = 10  # After header portion
        distances = []

        for i in range(self.POINTS_PER_FRAME):
            offset = data_start + i * self.POINT_SIZE
            # distance: int16 (mm), confidence: uint8 at offset+8
            dist_mm = struct.unpack_from('<h', frame, offset)[0]
            confidence = frame[offset + 8]

            if confidence >= self.conf_thresh and dist_mm > 0:
                distances.append(dist_mm / 1000.0)  # Convert to meters

        if not distances:
            return

        # Use median for robustness
        distances.sort()
        median_dist = distances[len(distances) // 2]

        # Publish Range
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.05  # ~3 degrees (narrow beam DTOF)
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = median_dist
        self.range_pub.publish(msg)

        # Publish emergency stop
        estop = Bool()
        estop.data = median_dist < self.estop_dist
        self.estop_pub.publish(estop)

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = STP23LNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
