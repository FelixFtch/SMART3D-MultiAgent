#!/usr/bin/env python3
import datetime as dt
import os
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


def iso_now():
    return dt.datetime.now().astimezone().isoformat(timespec='seconds')


def capture(cmd):
    try:
        proc = subprocess.run(
            cmd,
            shell=True,
            check=False,
            text=True,
            capture_output=True,
            timeout=4,
        )
        text = (proc.stdout or '') + (proc.stderr or '')
        return text.strip() or '<no output>'
    except Exception as exc:
        return f'<capture failed: {exc}>'


def mem_summary_mb():
    total = None
    avail = None
    try:
        with open('/proc/meminfo', 'r', encoding='utf-8') as handle:
            for line in handle:
                if line.startswith('MemTotal:'):
                    total = int(line.split()[1]) // 1024
                elif line.startswith('MemAvailable:'):
                    avail = int(line.split()[1]) // 1024
        return total, avail
    except Exception:
        return None, None


def wifi_summary():
    if Path('/proc/net/wireless').exists():
        return capture("awk 'NR==1 || NR==2 || /wlp3s0:/' /proc/net/wireless")
    return capture('iw dev wlp3s0 link 2>/dev/null || true')


class I9RosWatchdog(Node):
    def __init__(self):
        super().__init__('i9_ros_watchdog')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('pose_topic', '/car1/pose')
        self.declare_parameter('map_topic', '/map_campus')
        self.declare_parameter('tf_topic', '/tf')
        self.declare_parameter('heartbeat_sec', 5.0)
        self.declare_parameter('scan_gap_warn_sec', 1.0)
        self.declare_parameter('pose_gap_warn_sec', 2.0)
        self.declare_parameter('tf_gap_warn_sec', 2.0)
        self.declare_parameter('event_cooldown_sec', 6.0)
        self.declare_parameter('load_warn', 8.0)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.tf_topic = str(self.get_parameter('tf_topic').value)
        self.heartbeat_sec = float(self.get_parameter('heartbeat_sec').value)
        self.scan_gap_warn_sec = float(
            self.get_parameter('scan_gap_warn_sec').value
        )
        self.pose_gap_warn_sec = float(
            self.get_parameter('pose_gap_warn_sec').value
        )
        self.tf_gap_warn_sec = float(self.get_parameter('tf_gap_warn_sec').value)
        self.event_cooldown_sec = float(
            self.get_parameter('event_cooldown_sec').value
        )
        self.load_warn = float(self.get_parameter('load_warn').value)

        now = time.monotonic()
        self.last_seen = {
            'scan': now,
            'pose': now,
            'map': now,
            'tf': now,
        }
        self.counts = {key: 0 for key in self.last_seen}
        self.last_heartbeat = 0.0
        self.last_event = {}

        best_effort_qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(
            LaserScan, self.scan_topic, self._mark_scan, best_effort_qos
        )
        self.create_subscription(
            PoseStamped, self.pose_topic, self._mark_pose, best_effort_qos
        )
        self.create_subscription(
            OccupancyGrid, self.map_topic, self._mark_map, map_qos
        )
        self.create_subscription(
            TFMessage, self.tf_topic, self._mark_tf, best_effort_qos
        )

        self.create_timer(1.0, self._on_timer)
        self.get_logger().info(
            'i9 ROS watchdog started '
            f'(scan={self.scan_topic}, pose={self.pose_topic}, '
            f'map={self.map_topic}, tf={self.tf_topic})'
        )

    def _mark(self, key):
        self.last_seen[key] = time.monotonic()
        self.counts[key] += 1

    def _mark_scan(self, _msg):
        self._mark('scan')

    def _mark_pose(self, _msg):
        self._mark('pose')

    def _mark_map(self, _msg):
        self._mark('map')

    def _mark_tf(self, _msg):
        self._mark('tf')

    def _gap(self, key):
        return max(0.0, time.monotonic() - self.last_seen[key])

    def _emit_snapshot(self, reason, detail):
        now = time.monotonic()
        last = self.last_event.get(reason, 0.0)
        if now - last < self.event_cooldown_sec:
            return
        self.last_event[reason] = now

        load1, load5, load15 = os.getloadavg()
        total_mb, avail_mb = mem_summary_mb()
        print(
            f'[{iso_now()}] EVENT {reason}: {detail} '
            f'load=({load1:.2f},{load5:.2f},{load15:.2f}) '
            f'mem_avail_mb={avail_mb if avail_mb is not None else "NA"} '
            f'mem_total_mb={total_mb if total_mb is not None else "NA"}',
            flush=True,
        )
        print('--- planner procs ---', flush=True)
        print(
            capture(
                "ps -eo pid,ppid,pcpu,pmem,comm,args --sort=-pcpu | "
                "grep -E 'i9_planner.py|rviz2|i9_ros_watchdog.py' | grep -v grep"
            ),
            flush=True,
        )
        print('--- top ---', flush=True)
        print(
            capture(
                "ps -eo pid,ppid,pcpu,pmem,comm,args --sort=-pcpu | head -20"
            ),
            flush=True,
        )
        print('--- wifi ---', flush=True)
        print(wifi_summary(), flush=True)
        print('--- ip stats ---', flush=True)
        print(capture('ip -s link show wlp3s0'), flush=True)
        print('--- udp sockets ---', flush=True)
        print(
            capture(
                "ss -uapn 2>/dev/null | grep -E 'python3|rviz2|7400|7410|dds|cyclone' | head -60"
            ),
            flush=True,
        )
        print('--- recent logs ---', flush=True)
        print(
            capture(
                'for f in /tmp/i9_planner_campus.log /tmp/rviz_campus.log; do '
                '  [ -f "$f" ] || continue; '
                '  echo "== $f =="; '
                '  tail -n 50 "$f"; '
                'done'
            ),
            flush=True,
        )

    def _on_timer(self):
        load1, load5, load15 = os.getloadavg()
        gaps = {key: self._gap(key) for key in self.last_seen}
        now = time.monotonic()

        if now - self.last_heartbeat >= self.heartbeat_sec:
            self.last_heartbeat = now
            print(
                f'[{iso_now()}] HEARTBEAT '
                f'scan_gap={gaps["scan"]:.2f}s scan_count={self.counts["scan"]} '
                f'pose_gap={gaps["pose"]:.2f}s pose_count={self.counts["pose"]} '
                f'map_gap={gaps["map"]:.2f}s map_count={self.counts["map"]} '
                f'tf_gap={gaps["tf"]:.2f}s tf_count={self.counts["tf"]} '
                f'load=({load1:.2f},{load5:.2f},{load15:.2f})',
                flush=True,
            )

        if gaps['scan'] > self.scan_gap_warn_sec:
            self._emit_snapshot('scan_gap', f'no /scan for {gaps["scan"]:.2f}s')
        if gaps['pose'] > self.pose_gap_warn_sec:
            self._emit_snapshot('pose_gap', f'no /car1/pose for {gaps["pose"]:.2f}s')
        if gaps['tf'] > self.tf_gap_warn_sec:
            self._emit_snapshot('tf_gap', f'no /tf for {gaps["tf"]:.2f}s')

        planner_running = capture("pgrep -af 'i9_planner.py' || true")
        if not planner_running or planner_running == '<no output>':
            self._emit_snapshot('planner_missing', 'i9_planner.py not running')
        if load1 > self.load_warn:
            self._emit_snapshot('load_spike', f'load1={load1:.2f}')


def main():
    rclpy.init()
    node = I9RosWatchdog()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
