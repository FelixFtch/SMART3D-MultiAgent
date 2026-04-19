#!/usr/bin/env python3
import argparse
import json
import math
import os
import time
from pathlib import Path

import psutil
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def mean(values):
    return sum(values) / len(values) if values else 0.0


def sample_processes(substrings):
    self_pid = os.getpid()
    totals = {"cpu_percent": 0.0, "rss_bytes": 0, "matches": []}
    for proc in psutil.process_iter(["pid", "cmdline", "memory_info"]):
        try:
            if proc.pid == self_pid:
                continue
            cmdline = " ".join(proc.info["cmdline"] or [])
            if "measure_odom_candidate.py" in cmdline:
                continue
            if not any(s in cmdline for s in substrings):
                continue
            totals["cpu_percent"] += proc.cpu_percent(None)
            totals["rss_bytes"] += int(proc.info["memory_info"].rss)
            totals["matches"].append({"pid": proc.pid, "cmdline": cmdline})
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return totals


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/odom")
    parser.add_argument("--duration-sec", type=float, default=60.0)
    parser.add_argument("--process-substring", action="append", default=[])
    parser.add_argument("--output")
    args = parser.parse_args()

    if args.process_substring:
        self_pid = os.getpid()
        for proc in psutil.process_iter(["pid", "cmdline"]):
            try:
                if proc.pid == self_pid:
                    continue
                cmdline = " ".join(proc.info["cmdline"] or [])
                if "measure_odom_candidate.py" in cmdline:
                    continue
                if any(s in cmdline for s in args.process_substring):
                    proc.cpu_percent(None)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

    rclpy.init()
    node = rclpy.create_node("measure_odom_candidate")
    messages = []

    def callback(msg):
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        messages.append(
            {
                "stamp_sec": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "yaw": yaw,
            }
        )

    node.create_subscription(
        Odometry,
        args.topic,
        callback,
        QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT),
    )

    cpu_samples = []
    rss_samples = []
    process_snapshots = []
    deadline = time.time() + args.duration_sec
    next_sample = time.time() + 1.0

    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if args.process_substring and time.time() >= next_sample:
            sample = sample_processes(args.process_substring)
            cpu_samples.append(sample["cpu_percent"])
            rss_samples.append(sample["rss_bytes"])
            process_snapshots = sample["matches"]
            next_sample += 1.0

    node.destroy_node()
    rclpy.shutdown()

    result = {
        "topic": args.topic,
        "duration_sec": args.duration_sec,
        "message_count": len(messages),
        "avg_rate_hz": 0.0,
        "final_translation_drift_m": None,
        "final_yaw_drift_rad": None,
        "max_translation_from_start_m": None,
        "x_span_m": None,
        "y_span_m": None,
        "yaw_span_rad": None,
        "process_metrics": {
            "avg_cpu_percent": round(mean(cpu_samples), 2) if cpu_samples else 0.0,
            "max_cpu_percent": round(max(cpu_samples), 2) if cpu_samples else 0.0,
            "avg_rss_mb": round(mean(rss_samples) / (1024 * 1024), 2) if rss_samples else 0.0,
            "max_rss_mb": round(max(rss_samples) / (1024 * 1024), 2) if rss_samples else 0.0,
            "matches": process_snapshots,
        },
    }

    if messages:
        t0 = messages[0]["stamp_sec"]
        t1 = messages[-1]["stamp_sec"]
        result["avg_rate_hz"] = round(len(messages) / max(t1 - t0, 1e-6), 2)

        x0 = messages[0]["x"]
        y0 = messages[0]["y"]
        yaw0 = messages[0]["yaw"]
        x1 = messages[-1]["x"]
        y1 = messages[-1]["y"]
        yaw1 = messages[-1]["yaw"]

        result["final_translation_drift_m"] = round(math.hypot(x1 - x0, y1 - y0), 6)
        result["final_yaw_drift_rad"] = round(yaw1 - yaw0, 6)
        result["max_translation_from_start_m"] = round(
            max(math.hypot(m["x"] - x0, m["y"] - y0) for m in messages),
            6,
        )
        result["x_span_m"] = round(max(m["x"] for m in messages) - min(m["x"] for m in messages), 6)
        result["y_span_m"] = round(max(m["y"] for m in messages) - min(m["y"] for m in messages), 6)
        result["yaw_span_rad"] = round(max(m["yaw"] for m in messages) - min(m["yaw"] for m in messages), 6)

    output = json.dumps(result, indent=2)
    print(output)
    if args.output:
        Path(args.output).write_text(output + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
