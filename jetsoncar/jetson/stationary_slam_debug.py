#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import statistics
from dataclasses import dataclass, field
from typing import List, Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def unwrap_angle(prev: Optional[float], angle: float) -> float:
    if prev is None:
        return angle
    delta = angle - prev
    while delta > math.pi:
        angle -= 2.0 * math.pi
        delta = angle - prev
    while delta < -math.pi:
        angle += 2.0 * math.pi
        delta = angle - prev
    return angle


@dataclass
class TopicStats:
    name: str
    stamps: List[float] = field(default_factory=list)
    ages: List[float] = field(default_factory=list)
    scan_times: List[float] = field(default_factory=list)
    time_increments: List[float] = field(default_factory=list)
    angle_increments: List[float] = field(default_factory=list)
    ranges_sizes: List[int] = field(default_factory=list)
    backwards: int = 0

    def add_scan(self, msg: LaserScan, now_sec: float) -> None:
        stamp = stamp_to_sec(msg.header.stamp)
        if self.stamps and stamp < self.stamps[-1]:
            self.backwards += 1
        self.stamps.append(stamp)
        self.ages.append(now_sec - stamp)
        self.scan_times.append(float(msg.scan_time))
        self.time_increments.append(float(msg.time_increment))
        self.angle_increments.append(float(msg.angle_increment))
        self.ranges_sizes.append(len(msg.ranges))

    def summary(self) -> List[str]:
        lines = [f"[{self.name}] msgs={len(self.stamps)} backwards={self.backwards}"]
        if len(self.stamps) >= 2:
            dts = [b - a for a, b in zip(self.stamps, self.stamps[1:])]
            lines.append(
                "  stamp_dt mean={:.6f}s std={:.6f}s min={:.6f}s max={:.6f}s".format(
                    statistics.fmean(dts),
                    statistics.pstdev(dts) if len(dts) > 1 else 0.0,
                    min(dts),
                    max(dts),
                )
            )
        if self.ages:
            lines.append(
                "  age(now-stamp) mean={:.6f}s std={:.6f}s min={:.6f}s max={:.6f}s".format(
                    statistics.fmean(self.ages),
                    statistics.pstdev(self.ages) if len(self.ages) > 1 else 0.0,
                    min(self.ages),
                    max(self.ages),
                )
            )
        if self.scan_times:
            lines.append(
                "  scan_time mean={:.6f}s min={:.6f}s max={:.6f}s".format(
                    statistics.fmean(self.scan_times),
                    min(self.scan_times),
                    max(self.scan_times),
                )
            )
        if self.time_increments:
            lines.append(
                "  time_increment mean={:.9f}s min={:.9f}s max={:.9f}s".format(
                    statistics.fmean(self.time_increments),
                    min(self.time_increments),
                    max(self.time_increments),
                )
            )
        if self.angle_increments:
            lines.append(
                "  angle_increment mean={:.9f}rad".format(
                    statistics.fmean(self.angle_increments)
                )
            )
        if self.ranges_sizes:
            lines.append(
                "  ranges count values={}".format(
                    ",".join(str(v) for v in sorted(set(self.ranges_sizes)))
                )
            )
        return lines


class StationaryDebugNode(Node):
    def __init__(
        self,
        duration_sec: float,
        tf_parent: str,
        tf_child: str,
        tf_sample_hz: float,
    ) -> None:
        super().__init__("stationary_slam_debug")
        self.duration_sec = duration_sec
        self.deadline = self.get_clock().now() + Duration(seconds=duration_sec)

        self.scan = TopicStats("/scan")
        self.scan_slam = TopicStats("/scan_slam")
        self.imu_stamps: List[float] = []
        self.imu_ages: List[float] = []

        self.tf_parent = tf_parent
        self.tf_child = tf_child
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.tf_failures = 0
        self.tf_samples = 0
        self.tf_ages: List[float] = []
        self.tf_x: List[float] = []
        self.tf_y: List[float] = []
        self.tf_yaw: List[float] = []
        self._last_yaw: Optional[float] = None

        self.create_subscription(LaserScan, "/scan", self.on_scan, 50)
        self.create_subscription(LaserScan, "/scan_slam", self.on_scan_slam, 50)
        self.create_subscription(Imu, "/IMU_data", self.on_imu, 100)
        self.create_timer(1.0 / tf_sample_hz, self.sample_tf)
        self.create_timer(0.25, self.check_done)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_scan(self, msg: LaserScan) -> None:
        self.scan.add_scan(msg, self.now_sec())

    def on_scan_slam(self, msg: LaserScan) -> None:
        self.scan_slam.add_scan(msg, self.now_sec())

    def on_imu(self, msg: Imu) -> None:
        stamp = stamp_to_sec(msg.header.stamp)
        if self.imu_stamps and stamp < self.imu_stamps[-1]:
            pass
        self.imu_stamps.append(stamp)
        self.imu_ages.append(self.now_sec() - stamp)

    def sample_tf(self) -> None:
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.tf_parent, self.tf_child, rclpy.time.Time()
            )
        except TransformException:
            self.tf_failures += 1
            return

        self.tf_samples += 1
        stamp = stamp_to_sec(tf_msg.header.stamp)
        now = self.now_sec()
        self.tf_ages.append(now - stamp)
        self.tf_x.append(float(tf_msg.transform.translation.x))
        self.tf_y.append(float(tf_msg.transform.translation.y))
        q = tf_msg.transform.rotation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        yaw = unwrap_angle(self._last_yaw, yaw)
        self._last_yaw = yaw
        self.tf_yaw.append(yaw)

    def check_done(self) -> None:
        if self.get_clock().now() >= self.deadline:
            self.report()
            rclpy.shutdown()

    def report(self) -> None:
        print("")
        print("==== Stationary SLAM Debug ====")
        print(f"duration={self.duration_sec:.1f}s")
        for line in self.scan.summary():
            print(line)
        for line in self.scan_slam.summary():
            print(line)

        print(f"[/IMU_data] msgs={len(self.imu_stamps)}")
        if len(self.imu_stamps) >= 2:
            dts = [b - a for a, b in zip(self.imu_stamps, self.imu_stamps[1:])]
            print(
                "  stamp_dt mean={:.6f}s std={:.6f}s min={:.6f}s max={:.6f}s".format(
                    statistics.fmean(dts),
                    statistics.pstdev(dts) if len(dts) > 1 else 0.0,
                    min(dts),
                    max(dts),
                )
            )
        if self.imu_ages:
            print(
                "  age(now-stamp) mean={:.6f}s std={:.6f}s min={:.6f}s max={:.6f}s".format(
                    statistics.fmean(self.imu_ages),
                    statistics.pstdev(self.imu_ages) if len(self.imu_ages) > 1 else 0.0,
                    min(self.imu_ages),
                    max(self.imu_ages),
                )
            )

        print(
            f"[TF {self.tf_parent}->{self.tf_child}] samples={self.tf_samples} failures={self.tf_failures}"
        )
        if self.tf_ages:
            print(
                "  age(now-stamp) mean={:.6f}s std={:.6f}s min={:.6f}s max={:.6f}s".format(
                    statistics.fmean(self.tf_ages),
                    statistics.pstdev(self.tf_ages) if len(self.tf_ages) > 1 else 0.0,
                    min(self.tf_ages),
                    max(self.tf_ages),
                )
            )
        if self.tf_x and self.tf_y:
            drift = math.hypot(self.tf_x[-1] - self.tf_x[0], self.tf_y[-1] - self.tf_y[0])
            span = math.hypot(max(self.tf_x) - min(self.tf_x), max(self.tf_y) - min(self.tf_y))
            print(f"  xy drift start->end={drift:.4f}m span={span:.4f}m")
        if self.tf_yaw:
            yaw_span = math.degrees(max(self.tf_yaw) - min(self.tf_yaw))
            yaw_drift = math.degrees(self.tf_yaw[-1] - self.tf_yaw[0])
            print(f"  yaw drift start->end={yaw_drift:.3f}deg span={yaw_span:.3f}deg")

        print("")
        print("==== Hints ====")
        if self.scan.backwards or self.scan_slam.backwards:
            print("- Header stamps went backwards on /scan or /scan_slam. That confirms a timestamp ordering problem.")
        if self.scan_slam.ages and max(self.scan_slam.ages) < -0.01:
            print("- /scan_slam is stamped in the future relative to now. That matches RViz future extrapolation symptoms.")
        if self.scan.stamps and self.scan_slam.stamps:
            raw_dt = statistics.fmean(
                [b - a for a, b in zip(self.scan.stamps, self.scan.stamps[1:])]
            ) if len(self.scan.stamps) >= 2 else None
            slam_dt = statistics.fmean(
                [b - a for a, b in zip(self.scan_slam.stamps, self.scan_slam.stamps[1:])]
            ) if len(self.scan_slam.stamps) >= 2 else None
            if raw_dt and slam_dt and abs(raw_dt - slam_dt) > 0.01:
                print("- /scan and /scan_slam publish cadence differ noticeably. The normalizer is a strong suspect.")
        if self.tf_x and self.tf_y and self.tf_yaw:
            span = math.hypot(max(self.tf_x) - min(self.tf_x), max(self.tf_y) - min(self.tf_y))
            yaw_span = math.degrees(max(self.tf_yaw) - min(self.tf_yaw))
            if span > 0.10 or yaw_span > 3.0:
                print("- map->laser_link drift is large while the robot is stationary. The SLAM pose chain is unstable.")
            else:
                print("- map->laser_link drift stayed small during this sample.")
        print("- If raw /scan looks stable in RViz with Fixed Frame=laser_link, but map-fixed view drifts, the fault is in SLAM/TF, not raw lidar geometry.")
        print("- If switching Cartographer input from /scan_slam back to /scan removes drift, scan_metadata_normalizer.py is the main culprit.")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--tf-parent", default="map")
    parser.add_argument("--tf-child", default="laser_link")
    parser.add_argument("--tf-sample-hz", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = StationaryDebugNode(
        duration_sec=args.duration,
        tf_parent=args.tf_parent,
        tf_child=args.tf_child,
        tf_sample_hz=args.tf_sample_hz,
    )
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
