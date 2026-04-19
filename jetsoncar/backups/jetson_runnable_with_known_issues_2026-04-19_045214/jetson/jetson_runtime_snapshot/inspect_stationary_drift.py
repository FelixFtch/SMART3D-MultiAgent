#!/usr/bin/env python3
import math
import statistics
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


def quat_to_yaw_deg(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


@dataclass
class ScanStats:
    stamps: List[float] = field(default_factory=list)
    scan_times: List[float] = field(default_factory=list)
    time_increments: List[float] = field(default_factory=list)
    range_counts: List[int] = field(default_factory=list)
    angle_increments: List[float] = field(default_factory=list)
    expected_count_mismatches: int = 0
    non_monotonic_stamps: int = 0

    def record(self, msg: LaserScan) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if self.stamps and stamp <= self.stamps[-1]:
            self.non_monotonic_stamps += 1
        self.stamps.append(stamp)
        self.scan_times.append(float(msg.scan_time))
        self.time_increments.append(float(msg.time_increment))
        self.range_counts.append(len(msg.ranges))
        self.angle_increments.append(float(msg.angle_increment))

        if len(msg.ranges) > 1 and abs(msg.angle_increment) > 1e-12:
            raw_expected = ((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1.0
            expected = int(round(raw_expected))
            if expected != len(msg.ranges):
                self.expected_count_mismatches += 1

    def summary(self) -> Dict[str, object]:
        deltas = [b - a for a, b in zip(self.stamps, self.stamps[1:])]
        return {
            "count": len(self.stamps),
            "stamp_delta_min": min(deltas) if deltas else None,
            "stamp_delta_max": max(deltas) if deltas else None,
            "stamp_delta_mean": statistics.mean(deltas) if deltas else None,
            "scan_time_min": min(self.scan_times) if self.scan_times else None,
            "scan_time_max": max(self.scan_times) if self.scan_times else None,
            "scan_time_mean": statistics.mean(self.scan_times) if self.scan_times else None,
            "time_increment_min": min(self.time_increments) if self.time_increments else None,
            "time_increment_max": max(self.time_increments) if self.time_increments else None,
            "time_increment_mean": statistics.mean(self.time_increments) if self.time_increments else None,
            "range_count_values": sorted(set(self.range_counts)),
            "angle_increment_values": sorted({round(v, 12) for v in self.angle_increments}),
            "expected_count_mismatches": self.expected_count_mismatches,
            "non_monotonic_stamps": self.non_monotonic_stamps,
        }


@dataclass
class TransformStats:
    samples: List[Tuple[float, float, float, float]] = field(default_factory=list)

    def record(self, t: TFMessage, frame_id: str, child_frame_id: str, start_wall: float) -> None:
        now = time.time() - start_wall
        for transform in t.transforms:
            if transform.header.frame_id != frame_id or transform.child_frame_id != child_frame_id:
                continue
            tx = float(transform.transform.translation.x)
            ty = float(transform.transform.translation.y)
            yaw = quat_to_yaw_deg(
                float(transform.transform.rotation.x),
                float(transform.transform.rotation.y),
                float(transform.transform.rotation.z),
                float(transform.transform.rotation.w),
            )
            self.samples.append((now, tx, ty, yaw))

    def summary(self) -> Dict[str, object]:
        if not self.samples:
            return {"count": 0}
        xs = [s[1] for s in self.samples]
        ys = [s[2] for s in self.samples]
        yaws = [s[3] for s in self.samples]
        return {
            "count": len(self.samples),
            "x_span_m": max(xs) - min(xs),
            "y_span_m": max(ys) - min(ys),
            "yaw_span_deg": max(yaws) - min(yaws),
            "first": self.samples[0],
            "last": self.samples[-1],
        }


@dataclass
class PoseStats:
    samples: List[Tuple[float, float, float, float]] = field(default_factory=list)

    def record(self, msg: PoseStamped, start_wall: float) -> None:
        yaw = quat_to_yaw_deg(
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
        )
        self.samples.append(
            (
                time.time() - start_wall,
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                yaw,
            )
        )

    def summary(self) -> Dict[str, object]:
        if not self.samples:
            return {"count": 0}
        xs = [s[1] for s in self.samples]
        ys = [s[2] for s in self.samples]
        yaws = [s[3] for s in self.samples]
        return {
            "count": len(self.samples),
            "x_span_m": max(xs) - min(xs),
            "y_span_m": max(ys) - min(ys),
            "yaw_span_deg": max(yaws) - min(yaws),
            "first": self.samples[0],
            "last": self.samples[-1],
        }


class StationaryDriftInspector(Node):
    def __init__(self) -> None:
        super().__init__("stationary_drift_inspector")
        self.declare_parameter("duration_sec", 8.0)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("pose_topic", "/car1/pose")

        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.start_wall = time.time()

        scan_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        tf_qos = QoSProfile(
            depth=200,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        pose_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.scan_stats = ScanStats()
        self.map_odom_stats = TransformStats()
        self.odom_base_stats = TransformStats()
        self.pose_stats = PoseStats()

        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, scan_qos)
        self.create_subscription(TFMessage, "/tf", self.on_tf, tf_qos)
        self.create_subscription(PoseStamped, self.pose_topic, self.on_pose, pose_qos)

    def on_scan(self, msg: LaserScan) -> None:
        self.scan_stats.record(msg)

    def on_tf(self, msg: TFMessage) -> None:
        self.map_odom_stats.record(msg, "map", "odom", self.start_wall)
        self.odom_base_stats.record(msg, "odom", "base_link", self.start_wall)

    def on_pose(self, msg: PoseStamped) -> None:
        self.pose_stats.record(msg, self.start_wall)

    def done(self) -> bool:
        return (time.time() - self.start_wall) >= self.duration_sec

    def print_report(self) -> None:
        print("=== stationary-drift report ===")
        print("scan_topic:", self.scan_topic)
        print("pose_topic:", self.pose_topic)
        print("duration_sec:", self.duration_sec)
        print("scan:", self.scan_stats.summary())
        print("tf_map_odom:", self.map_odom_stats.summary())
        print("tf_odom_base_link:", self.odom_base_stats.summary())
        print("pose:", self.pose_stats.summary())


def main() -> None:
    rclpy.init()
    node = StationaryDriftInspector()
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.2)
        node.print_report()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
