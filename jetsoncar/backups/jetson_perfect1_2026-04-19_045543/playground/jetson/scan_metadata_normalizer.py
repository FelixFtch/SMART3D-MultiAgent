#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanMetadataNormalizer(Node):
    def __init__(self) -> None:
        super().__init__("scan_metadata_normalizer")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_slam")
        self.declare_parameter("fallback_scan_hz", 12.0)
        self.declare_parameter("adjust_angle_increment", False)
        self.declare_parameter("prefer_stamp_delta_for_scan_time", True)
        self.declare_parameter("derive_time_increment_from_scan_time", True)
        self.declare_parameter("minimum_monotonic_gap_sec", 1.0e-4)
        self.declare_parameter("warn_overlap_sec", 1.0e-3)
        self.declare_parameter("drop_overlap_sec", 1.5e-2)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.fallback_scan_hz = float(self.get_parameter("fallback_scan_hz").value)
        self.adjust_angle_increment = bool(
            self.get_parameter("adjust_angle_increment").value
        )
        self.prefer_stamp_delta_for_scan_time = bool(
            self.get_parameter("prefer_stamp_delta_for_scan_time").value
        )
        self.derive_time_increment_from_scan_time = bool(
            self.get_parameter("derive_time_increment_from_scan_time").value
        )
        self.minimum_monotonic_gap_sec = float(
            self.get_parameter("minimum_monotonic_gap_sec").value
        )
        self.warn_overlap_sec = float(self.get_parameter("warn_overlap_sec").value)
        self.drop_overlap_sec = float(self.get_parameter("drop_overlap_sec").value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(LaserScan, self.output_topic, qos)
        self.create_subscription(LaserScan, self.input_topic, self.handle_scan, qos)

        self.last_warning_key = None
        self.last_warning_time = 0.0
        self.last_stamp = None
        self.last_scan_time = (
            1.0 / self.fallback_scan_hz if self.fallback_scan_hz > 0.0 else 1.0 / 12.0
        )
        self.last_output_start = None
        self.last_output_end = None
        self.last_timing_warning_time = 0.0
        self.retimed_count = 0
        self.dropped_count = 0
        self.get_logger().info(
            "Normalizing LaserScan metadata: "
            f"{self.input_topic} -> {self.output_topic} "
            f"(adjust_angle_increment={self.adjust_angle_increment}, "
            f"prefer_stamp_delta_for_scan_time={self.prefer_stamp_delta_for_scan_time}, "
            f"derive_time_increment_from_scan_time={self.derive_time_increment_from_scan_time}, "
            f"minimum_monotonic_gap_sec={self.minimum_monotonic_gap_sec}, "
            f"warn_overlap_sec={self.warn_overlap_sec}, "
            f"drop_overlap_sec={self.drop_overlap_sec})"
        )

    def _warn_timing(self, message: str) -> None:
        now = time.monotonic()
        if (now - self.last_timing_warning_time) >= 2.0:
            self.last_timing_warning_time = now
            self.get_logger().warn(message)

    def handle_scan(self, msg: LaserScan) -> None:
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = list(msg.ranges)
        out.intensities = list(msg.intensities)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        stamp_delta = None
        if self.last_stamp is not None:
            stamp_delta = stamp - self.last_stamp
        self.last_stamp = stamp

        if len(out.ranges) > 1 and self.adjust_angle_increment:
            raw_expected = ((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1.0
            expected = int(round(raw_expected))
            if expected != len(out.ranges):
                span = msg.angle_max - msg.angle_min
                out.angle_increment = span / float(len(out.ranges) - 1)
                warning_key = (expected, len(out.ranges))
                now = time.monotonic()
                if (
                    warning_key != self.last_warning_key
                    or (now - self.last_warning_time) >= 15.0
                ):
                    self.last_warning_key = warning_key
                    self.last_warning_time = now
                    self.get_logger().warn(
                        "Adjusted LaserScan angle_increment "
                        f"(expected={expected}, actual={len(out.ranges)}, "
                        f"old_inc={msg.angle_increment:.9f}, new_inc={out.angle_increment:.9f})"
                    )
            else:
                out.angle_increment = msg.angle_increment
        else:
            out.angle_increment = msg.angle_increment

        if out.intensities and len(out.intensities) != len(out.ranges):
            if len(out.intensities) > len(out.ranges):
                out.intensities = out.intensities[: len(out.ranges)]
            else:
                out.intensities.extend(
                    [math.nan] * (len(out.ranges) - len(out.intensities))
                )

        sane_scan_time = float(msg.scan_time)
        valid_stamp_delta = stamp_delta is not None and 0.0 < stamp_delta <= 0.5
        if self.prefer_stamp_delta_for_scan_time and valid_stamp_delta:
            sane_scan_time = float(stamp_delta)
        elif (
            not math.isfinite(sane_scan_time)
            or sane_scan_time <= 0.0
            or sane_scan_time > 0.5
        ):
            if valid_stamp_delta:
                sane_scan_time = float(stamp_delta)
            else:
                sane_scan_time = self.last_scan_time

        sane_time_increment = float(msg.time_increment)
        max_time_increment = sane_scan_time
        if len(out.ranges) > 1:
            max_time_increment = sane_scan_time / float(len(out.ranges) - 1)
        if self.derive_time_increment_from_scan_time and len(out.ranges) > 1:
            sane_time_increment = max_time_increment
        elif (
            not math.isfinite(sane_time_increment)
            or sane_time_increment < 0.0
            or sane_time_increment > sane_scan_time
            or sane_time_increment > (max_time_increment * 2.0)
        ):
            sane_time_increment = max_time_increment if len(out.ranges) > 1 else 0.0

        out.scan_time = sane_scan_time
        out.time_increment = sane_time_increment
        self.last_scan_time = sane_scan_time

        scan_span = 0.0
        if len(out.ranges) > 1:
            scan_span = sane_time_increment * float(len(out.ranges) - 1)
        elif sane_scan_time > 0.0:
            scan_span = sane_scan_time

        output_stamp = stamp
        if not math.isfinite(output_stamp) or output_stamp <= 0.0:
            if self.last_output_end is not None:
                output_stamp = self.last_output_end + self.minimum_monotonic_gap_sec
            else:
                output_stamp = self.get_clock().now().nanoseconds / 1e9

        if self.last_output_end is not None:
            overlap_sec = self.last_output_end - output_stamp
            if overlap_sec >= self.drop_overlap_sec:
                self.dropped_count += 1
                self._warn_timing(
                    "Dropping LaserScan due to severe timestamp overlap "
                    f"(overlap={overlap_sec:.6f}s, dropped={self.dropped_count})"
                )
                return
            if overlap_sec >= -self.minimum_monotonic_gap_sec:
                adjusted = self.last_output_end + self.minimum_monotonic_gap_sec
                self.retimed_count += 1
                if overlap_sec >= self.warn_overlap_sec:
                    self._warn_timing(
                        "Retiming LaserScan to preserve strict monotonic order "
                        f"(overlap={overlap_sec:.6f}s, retimed={self.retimed_count})"
                    )
                output_stamp = adjusted

        sec = int(math.floor(output_stamp))
        nanosec = int(round((output_stamp - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        out.header.stamp.sec = sec
        out.header.stamp.nanosec = nanosec

        self.last_output_start = output_stamp
        self.last_output_end = output_stamp + scan_span

        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ScanMetadataNormalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
