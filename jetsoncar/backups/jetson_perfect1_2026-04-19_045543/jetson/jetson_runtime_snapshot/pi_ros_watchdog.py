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
from sensor_msgs.msg import Imu, LaserScan


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


def run_bash(command, timeout=15):
    try:
        proc = subprocess.run(
            ['bash', '-lc', command],
            check=False,
            text=True,
            capture_output=True,
            timeout=timeout,
        )
        text = (proc.stdout or '') + (proc.stderr or '')
        return proc.returncode, text.strip() or '<no output>'
    except Exception as exc:
        return 255, f'<bash failed: {exc}>'


def read_temp_c():
    thermal = Path('/sys/class/thermal/thermal_zone0/temp')
    try:
        return float(thermal.read_text().strip()) / 1000.0
    except Exception:
        return None


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


def default_lidar_restart_cmd(
    ros_distro: str,
    runtime_root: str,
    lidar_ws: str,
    lidar_params_file: str,
    cyclone_uri: str,
):
    return f"""
export ROS_DOMAIN_ID="${{ROS_DOMAIN_ID:-42}}";
export RMW_IMPLEMENTATION="${{RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}}";
export CYCLONEDDS_URI="${{CYCLONEDDS_URI:-{cyclone_uri}}}";
python3 - <<'PY'
import os
import signal
import subprocess
import time

def matching_pids():
    proc = subprocess.run(
        ['pgrep', '-af', 'lslidar_driver_node'],
        check=False,
        capture_output=True,
        text=True,
    )
    pids = []
    self_pid = os.getpid()
    parent_pid = os.getppid()
    for line in proc.stdout.splitlines():
        parts = line.split(None, 1)
        if not parts:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        if pid not in (self_pid, parent_pid):
            pids.append(pid)
    return pids

for pid in matching_pids():
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        pass

time.sleep(0.5)

for pid in matching_pids():
    try:
        os.kill(pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
PY
sleep 1;
{runtime_root}/mk_m10p_dev.sh >/tmp/mk_m10p_dev.log 2>&1 || true;
nohup bash -lc 'source /opt/ros/{ros_distro}/setup.bash;
if [ -f {lidar_ws}/install/setup.bash ]; then source {lidar_ws}/install/setup.bash; fi;
export ROS_DOMAIN_ID="${{ROS_DOMAIN_ID:-42}}";
export RMW_IMPLEMENTATION="${{RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}}";
export CYCLONEDDS_URI="${{CYCLONEDDS_URI}}";
exec ros2 run lslidar_driver lslidar_driver_node --ros-args --params-file {lidar_params_file}' >/tmp/lidar.log 2>&1 < /dev/null &
sleep 2;
tail -n 60 /tmp/lidar.log 2>/dev/null || true
""".strip()


class PiRosWatchdog(Node):
    def __init__(self):
        super().__init__('pi_ros_watchdog')

        runtime_root = os.environ.get('SMART3D_RUNTIME_ROOT', '/root')
        lidar_ws = os.environ.get('SMART3D_LIDAR_WS', '/root/m10p_ws')
        lidar_params_file = os.environ.get(
            'SMART3D_LIDAR_PARAMS', f'{lidar_ws}/m10p_uart.yaml'
        )
        ros_distro = os.environ.get('ROS_DISTRO', 'humble')
        cyclone_uri = os.environ.get(
            'SMART3D_CYCLONEDDS_URI', 'file:///tmp/cyclone-campus-pi.xml'
        )

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/IMU_data')
        self.declare_parameter('pose_topic', '/car1/pose')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('heartbeat_sec', 5.0)
        self.declare_parameter('scan_gap_warn_sec', 1.0)
        self.declare_parameter('imu_gap_warn_sec', 1.0)
        self.declare_parameter('pose_gap_warn_sec', 2.0)
        self.declare_parameter('event_cooldown_sec', 6.0)
        self.declare_parameter('load_warn', 4.0)
        self.declare_parameter('temp_warn_c', 78.0)
        self.declare_parameter('restart_lidar_on_scan_gap', True)
        self.declare_parameter('scan_restart_gap_sec', 3.0)
        self.declare_parameter('scan_restart_cooldown_sec', 20.0)
        self.declare_parameter('runtime_root', runtime_root)
        self.declare_parameter('lidar_ws', lidar_ws)
        self.declare_parameter('lidar_params_file', lidar_params_file)
        self.declare_parameter('ros_distro', ros_distro)
        self.declare_parameter('cyclonedds_uri_fallback', cyclone_uri)
        self.declare_parameter(
            'lidar_restart_cmd',
            default_lidar_restart_cmd(
                ros_distro=ros_distro,
                runtime_root=runtime_root,
                lidar_ws=lidar_ws,
                lidar_params_file=lidar_params_file,
                cyclone_uri=cyclone_uri,
            ),
        )

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.heartbeat_sec = float(self.get_parameter('heartbeat_sec').value)
        self.scan_gap_warn_sec = float(
            self.get_parameter('scan_gap_warn_sec').value
        )
        self.imu_gap_warn_sec = float(
            self.get_parameter('imu_gap_warn_sec').value
        )
        self.pose_gap_warn_sec = float(
            self.get_parameter('pose_gap_warn_sec').value
        )
        self.event_cooldown_sec = float(
            self.get_parameter('event_cooldown_sec').value
        )
        self.load_warn = float(self.get_parameter('load_warn').value)
        self.temp_warn_c = float(self.get_parameter('temp_warn_c').value)
        self.restart_lidar_on_scan_gap = bool(
            self.get_parameter('restart_lidar_on_scan_gap').value
        )
        self.scan_restart_gap_sec = float(
            self.get_parameter('scan_restart_gap_sec').value
        )
        self.scan_restart_cooldown_sec = float(
            self.get_parameter('scan_restart_cooldown_sec').value
        )
        self.lidar_restart_cmd = str(
            self.get_parameter('lidar_restart_cmd').value
        )

        now = time.monotonic()
        self.last_seen = {
            'scan': now,
            'imu': now,
            'pose': now,
            'map': now,
        }
        self.counts = {key: 0 for key in self.last_seen}
        self.last_heartbeat = 0.0
        self.last_event = {}
        self.last_lidar_restart = 0.0

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        pose_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(
            LaserScan, self.scan_topic, self._mark_scan, sensor_qos
        )
        self.create_subscription(Imu, self.imu_topic, self._mark_imu, sensor_qos)
        self.create_subscription(
            PoseStamped, self.pose_topic, self._mark_pose, pose_qos
        )
        self.create_subscription(
            OccupancyGrid, self.map_topic, self._mark_map, map_qos
        )

        self.create_timer(1.0, self._on_timer)
        self.get_logger().info(
            'Pi ROS watchdog started '
            f'(scan={self.scan_topic}, imu={self.imu_topic}, '
            f'pose={self.pose_topic}, map={self.map_topic})'
        )

    def _mark(self, key):
        self.last_seen[key] = time.monotonic()
        self.counts[key] += 1

    def _mark_scan(self, _msg):
        self._mark('scan')

    def _mark_imu(self, _msg):
        self._mark('imu')

    def _mark_pose(self, _msg):
        self._mark('pose')

    def _mark_map(self, _msg):
        self._mark('map')

    def _gap(self, key):
        return max(0.0, time.monotonic() - self.last_seen[key])

    def _emit_snapshot(self, reason, detail):
        now = time.monotonic()
        last = self.last_event.get(reason, 0.0)
        if now - last < self.event_cooldown_sec:
            return
        self.last_event[reason] = now

        load1, load5, load15 = os.getloadavg()
        temp_c = read_temp_c()
        total_mb, avail_mb = mem_summary_mb()

        print(
            f'[{iso_now()}] EVENT {reason}: {detail} '
            f'load=({load1:.2f},{load5:.2f},{load15:.2f}) '
            f'temp_c={temp_c if temp_c is not None else "NA"} '
            f'mem_avail_mb={avail_mb if avail_mb is not None else "NA"} '
            f'mem_total_mb={total_mb if total_mb is not None else "NA"}',
            flush=True,
        )
        print('--- ps top ---', flush=True)
        print(
            capture(
                "ps -eo pid,ppid,pcpu,pmem,comm,args --sort=-pcpu | head -20"
            ),
            flush=True,
        )
        print('--- loadavg ---', flush=True)
        print(capture('cat /proc/loadavg'), flush=True)
        print('--- meminfo ---', flush=True)
        print(
            capture(
                "grep -E 'MemTotal|MemAvailable|SwapTotal|SwapFree' /proc/meminfo"
            ),
            flush=True,
        )
        print('--- recent logs ---', flush=True)
        print(
            capture(
                'for f in /tmp/start_full_campus.log '
                '/tmp/cartographer.log /tmp/lidar.log /tmp/imu_bridge.log '
                '/tmp/odom.log /tmp/pi5_client_campus.log '
                '/tmp/cmd_vel_driver_campus.log; do '
                '  [ -f "$f" ] || continue; '
                '  echo "== $f =="; '
                '  tail -n 40 "$f"; '
                'done'
            ),
            flush=True,
        )

    def _maybe_restart_lidar(self, gap):
        if not self.restart_lidar_on_scan_gap:
            return
        if gap < self.scan_restart_gap_sec:
            return
        now = time.monotonic()
        if now - self.last_lidar_restart < self.scan_restart_cooldown_sec:
            return

        self.last_lidar_restart = now
        print(
            f'[{iso_now()}] ACTION scan_gap restart_lidar '
            f'gap={gap:.2f}s cooldown={self.scan_restart_cooldown_sec:.1f}s',
            flush=True,
        )
        print('--- lidar restart cmd ---', flush=True)
        print(self.lidar_restart_cmd, flush=True)
        rc, out = run_bash(self.lidar_restart_cmd, timeout=20)
        print(f'--- lidar restart rc={rc} ---', flush=True)
        print(out, flush=True)
        print('--- post-restart m10p probe ---', flush=True)
        print(
            capture(
                "bash -lc 'ls -l /dev/m10p 2>/dev/null || true; "
                "ls -l /dev/ttyACM* 2>/dev/null || true; "
                "ps -ef | grep -E \"[l]slidar_driver_node|[s]tart_full.sh\" || true'"
            ),
            flush=True,
        )

    def _on_timer(self):
        load1, load5, load15 = os.getloadavg()
        temp_c = read_temp_c()
        gaps = {key: self._gap(key) for key in self.last_seen}

        now = time.monotonic()
        if now - self.last_heartbeat >= self.heartbeat_sec:
            self.last_heartbeat = now
            print(
                f'[{iso_now()}] HEARTBEAT '
                f'scan_gap={gaps["scan"]:.2f}s scan_count={self.counts["scan"]} '
                f'imu_gap={gaps["imu"]:.2f}s imu_count={self.counts["imu"]} '
                f'pose_gap={gaps["pose"]:.2f}s pose_count={self.counts["pose"]} '
                f'map_gap={gaps["map"]:.2f}s map_count={self.counts["map"]} '
                f'load=({load1:.2f},{load5:.2f},{load15:.2f}) '
                f'temp_c={temp_c if temp_c is not None else "NA"}',
                flush=True,
            )

        if gaps['scan'] > self.scan_gap_warn_sec:
            self._emit_snapshot('scan_gap', f'no /scan for {gaps["scan"]:.2f}s')
            self._maybe_restart_lidar(gaps['scan'])
        if gaps['imu'] > self.imu_gap_warn_sec:
            self._emit_snapshot('imu_gap', f'no /IMU_data for {gaps["imu"]:.2f}s')
        if gaps['pose'] > self.pose_gap_warn_sec:
            self._emit_snapshot(
                'pose_gap', f'no /car1/pose for {gaps["pose"]:.2f}s'
            )
        if load1 > self.load_warn:
            self._emit_snapshot('load_spike', f'load1={load1:.2f}')
        if temp_c is not None and temp_c > self.temp_warn_c:
            self._emit_snapshot('temp_spike', f'temp_c={temp_c:.1f}')


def main():
    rclpy.init()
    node = PiRosWatchdog()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
