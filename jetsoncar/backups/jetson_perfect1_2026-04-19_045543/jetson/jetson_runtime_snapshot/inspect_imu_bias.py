#!/usr/bin/env python3
import math
import statistics
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


def quat_to_rpy_deg(x: float, y: float, z: float, w: float) -> Dict[str, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.degrees(math.copysign(math.pi / 2.0, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return {"roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw}


def summarize(values: List[float]) -> Dict[str, Optional[float]]:
    if not values:
        return {
            "count": 0,
            "min": None,
            "max": None,
            "mean": None,
            "stdev": None,
        }
    return {
        "count": len(values),
        "min": min(values),
        "max": max(values),
        "mean": statistics.mean(values),
        "stdev": statistics.pstdev(values) if len(values) > 1 else 0.0,
    }


@dataclass
class ImuStats:
    header_stamps: List[float] = field(default_factory=list)
    ang_x: List[float] = field(default_factory=list)
    ang_y: List[float] = field(default_factory=list)
    ang_z: List[float] = field(default_factory=list)
    acc_x: List[float] = field(default_factory=list)
    acc_y: List[float] = field(default_factory=list)
    acc_z: List[float] = field(default_factory=list)
    roll_deg: List[float] = field(default_factory=list)
    pitch_deg: List[float] = field(default_factory=list)
    yaw_deg: List[float] = field(default_factory=list)
    non_monotonic_stamps: int = 0

    def record(self, msg: Imu) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if self.header_stamps and stamp <= self.header_stamps[-1]:
            self.non_monotonic_stamps += 1
        self.header_stamps.append(stamp)

        self.ang_x.append(float(msg.angular_velocity.x))
        self.ang_y.append(float(msg.angular_velocity.y))
        self.ang_z.append(float(msg.angular_velocity.z))
        self.acc_x.append(float(msg.linear_acceleration.x))
        self.acc_y.append(float(msg.linear_acceleration.y))
        self.acc_z.append(float(msg.linear_acceleration.z))

        rpy = quat_to_rpy_deg(
            float(msg.orientation.x),
            float(msg.orientation.y),
            float(msg.orientation.z),
            float(msg.orientation.w),
        )
        self.roll_deg.append(rpy["roll_deg"])
        self.pitch_deg.append(rpy["pitch_deg"])
        self.yaw_deg.append(rpy["yaw_deg"])

    def summary(self) -> Dict[str, object]:
        deltas = [b - a for a, b in zip(self.header_stamps, self.header_stamps[1:])]
        return {
            "count": len(self.header_stamps),
            "header_delta_sec": summarize(deltas),
            "non_monotonic_stamps": self.non_monotonic_stamps,
            "angular_velocity_x": summarize(self.ang_x),
            "angular_velocity_y": summarize(self.ang_y),
            "angular_velocity_z": summarize(self.ang_z),
            "linear_acceleration_x": summarize(self.acc_x),
            "linear_acceleration_y": summarize(self.acc_y),
            "linear_acceleration_z": summarize(self.acc_z),
            "roll_deg": summarize(self.roll_deg),
            "pitch_deg": summarize(self.pitch_deg),
            "yaw_deg": summarize(self.yaw_deg),
        }


class ImuBiasInspector(Node):
    def __init__(self) -> None:
        super().__init__("imu_bias_inspector")
        self.declare_parameter("topic", "/IMU_data")
        self.declare_parameter("duration_sec", 20.0)

        self.topic = str(self.get_parameter("topic").value)
        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.start_wall = time.time()
        self.stats = ImuStats()

        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, self.topic, self.on_imu, qos)

    def on_imu(self, msg: Imu) -> None:
        self.stats.record(msg)

    def done(self) -> bool:
        return (time.time() - self.start_wall) >= self.duration_sec

    def print_report(self) -> None:
        print("=== imu-bias report ===")
        print("topic:", self.topic)
        print("duration_sec:", self.duration_sec)
        print(self.stats.summary())


def main() -> None:
    rclpy.init()
    node = ImuBiasInspector()
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.2)
        node.print_report()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
