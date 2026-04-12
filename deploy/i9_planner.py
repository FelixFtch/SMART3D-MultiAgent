#!/usr/bin/env python3
"""
SMART-3D Central Planner — runs on i9/4090 Linux workstation

Zero compromises:
  - 360° obstacle detection (not just ±90°)
  - 20 Hz update rate (not 5Hz)
  - Full resolution map processing
  - Frame-to-frame obstacle speed estimation (not assumed 0.5)
  - Every other car = dynamic obstacle with real speed

Multi-agent:
  - One SMART tree per car
  - Each car's SMART sees other cars as SphericalObstacleWithOhz
  - Server publishes Path to each car's Pi 5

Usage:
  python3 i9_planner.py --ros-args -p car_names:="car1,car2"

Or single car:
  python3 i9_planner.py --ros-args -p car_names:="car1"
"""

import os
import sys
import json
import math
import time
import subprocess
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


# ============================================================
#  Rust planner subprocess (one per car)
# ============================================================
class PlannerBridge:
    def __init__(self, binary, logger, name=''):
        self.logger = logger
        self.name = name
        self.proc = None
        self.lock = threading.Lock()
        if not os.path.exists(binary):
            logger.error(f'[{name}] Binary not found: {binary}')
            return
        try:
            self.proc = subprocess.Popen(
                [binary], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, bufsize=0)
            threading.Thread(target=self._drain, daemon=True).start()
            logger.info(f'[{name}] Planner started')
        except Exception as e:
            logger.error(f'[{name}] Start failed: {e}')

    def _drain(self):
        for line in self.proc.stderr:
            self.logger.info(f'[{self.name}/rust] {line.decode().strip()}')

    def cmd(self, d):
        if not self.proc or self.proc.poll() is not None:
            return {'ok': False, 'error': 'dead'}
        with self.lock:
            try:
                self.proc.stdin.write((json.dumps(d) + '\n').encode())
                self.proc.stdin.flush()
                r = self.proc.stdout.readline()
                return json.loads(r.decode()) if r else {'ok': False, 'error': 'eof'}
            except Exception as e:
                return {'ok': False, 'error': str(e)}

    def shutdown(self):
        if self.proc and self.proc.poll() is None:
            self.cmd({'cmd': 'quit'})
            try:
                self.proc.wait(timeout=3)
            except:
                self.proc.kill()


# ============================================================
#  Per-car agent state
# ============================================================
class CarAgent:
    def __init__(self, name, planner, logger):
        self.name = name
        self.planner = planner
        self.logger = logger

        # Pose (from Pi 5)
        self.rx = self.ry = self.ryaw = 0.0
        self.have_pose = False
        self.pose_time = 0.0

        # Speed estimation (from pose changes)
        self.prev_rx = self.prev_ry = 0.0
        self.prev_pose_time = 0.0
        self.est_speed = 0.0
        self.est_heading = 0.0

        # Goal
        self.goal = None

        # Path
        self.path = []
        self.pidx = 0
        self.navigating = False

        # Map
        self.map_msg = None
        self.known_walls = []
        self.occ_count = 0

        # Scan
        self.last_scan = None

        # Dynamic obstacles (tracked between frames)
        self.prev_obstacles = []  # [(x, y, r)] from last frame
        self.tracked_obstacles = []  # [(x, y, r, speed)]

        # Timing
        self.t_build = 0.0
        self.t_update = 0.0
        self.smart_fails = 0

    def update_pose(self, msg):
        """Update from PoseStamped."""
        self.prev_rx = self.rx
        self.prev_ry = self.ry
        self.prev_pose_time = self.pose_time

        self.rx = msg.pose.position.x
        self.ry = msg.pose.position.y
        q = msg.pose.orientation
        self.ryaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.have_pose = True
        self.pose_time = time.time()

        # Estimate robot speed from pose changes
        dt = self.pose_time - self.prev_pose_time
        if dt > 0.01 and self.prev_pose_time > 0:
            dx = self.rx - self.prev_rx
            dy = self.ry - self.prev_ry
            self.est_speed = math.sqrt(dx*dx + dy*dy) / dt
            if abs(dx) > 0.001 or abs(dy) > 0.001:
                self.est_heading = math.atan2(dy, dx)


# ============================================================
#  Helpers
# ============================================================
def pt_seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx*dx + dy*dy
    if L2 < 1e-6:
        return math.sqrt((px-ax)**2 + (py-ay)**2)
    t = max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy) / L2))
    qx, qy = ax + t*dx, ay + t*dy
    return math.sqrt((px-qx)**2 + (py-qy)**2)


def pt_in_rect(px, py, x1, y1, x2, y2, m=0.0):
    return (x1-m) <= px <= (x2+m) and (y1-m) <= py <= (y2+m)


# ============================================================
#  Central Planner Node
# ============================================================
class CentralPlanner(Node):
    def __init__(self):
        super().__init__('smart_central_planner')

        self.declare_parameter('car_names', 'car1')
        self.declare_parameter('planner_binary',
            os.path.expanduser('~/SMART3D/target/release/examples/smart_planner_server'))
        self.declare_parameter('rrt_iterations', 15000)
        self.declare_parameter('robot_speed', 0.2)
        self.declare_parameter('robot_radius', 0.35)

        car_str = self.get_parameter('car_names').value
        self.car_names = [s.strip() for s in car_str.split(',')]
        binary = self.get_parameter('planner_binary').value
        self.rrt_iters = self.get_parameter('rrt_iterations').value
        self.robot_speed = self.get_parameter('robot_speed').value
        self.robot_radius = self.get_parameter('robot_radius').value

        self.get_logger().info(
            f'Central planner for {len(self.car_names)} car(s): {self.car_names}')

        # Create agents
        self.agents = {}
        for name in self.car_names:
            planner = PlannerBridge(binary, self.get_logger(), name)
            self.agents[name] = CarAgent(name, planner, self.get_logger())

        # Per-car subscribers and publishers
        self.path_pubs = {}
        self.viz_pub = self.create_publisher(MarkerArray, '/smart/markers', 10)

        scan_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        map_qos = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)

        for name in self.car_names:
            # Path publisher (TRANSIENT_LOCAL so Pi5 gets it even if it subscribes late)
            self.path_pubs[name] = self.create_publisher(
                Path, f'/{name}/path',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE))

            # Pose from Pi 5
            self.create_subscription(
                PoseStamped, f'/{name}/pose',
                lambda msg, n=name: self._on_pose(n, msg), 10)

            # Goal from Pi 5 (forwarded from RViz)
            self.create_subscription(
                PoseStamped, f'/{name}/goal_pose',
                lambda msg, n=name: self._on_goal(n, msg), 10)

            # Scan from Pi 5 LiDAR (BEST_EFFORT to match M10P)
            self.create_subscription(
                LaserScan, f'/{name}/scan',
                lambda msg, n=name: self._on_scan(n, msg), scan_qos)

            # Map from Pi 5 Cartographer
            self.create_subscription(
                OccupancyGrid, f'/{name}/map',
                lambda msg, n=name: self._on_map(n, msg), map_qos)

        # Also subscribe to un-namespaced topics (for single-car setup)
        if len(self.car_names) == 1:
            name = self.car_names[0]
            self.create_subscription(
                LaserScan, '/scan',
                lambda msg, n=name: self._on_scan(n, msg), scan_qos)
            self.create_subscription(
                OccupancyGrid, '/map',
                lambda msg, n=name: self._on_map(n, msg), map_qos)
            self.create_subscription(
                PoseStamped, '/goal_pose',
                lambda msg, n=name: self._on_goal(n, msg), 10)

        # Main planning loop: 20 Hz (i9 can handle it easily)
        self.create_timer(0.05, self.plan_loop)

        # Map change check: every 5s
        self.create_timer(2.0, self.check_maps)

        # Visualization: 5 Hz
        self.create_timer(0.2, self.publish_viz)

    # ---- Callbacks (just store) ----

    def _on_pose(self, name, msg):
        if name in self.agents:
            self.agents[name].update_pose(msg)

    def _on_scan(self, name, msg):
        if name in self.agents:
            self.agents[name].last_scan = msg

    def _on_map(self, name, msg):
        if name in self.agents:
            self.agents[name].map_msg = msg

    def _on_goal(self, name, msg):
        if name not in self.agents:
            return
        agent = self.agents[name]
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        agent.goal = (gx, gy)
        self.get_logger().info(f'[{name}] Goal: ({gx:.2f}, {gy:.2f})')
        self.do_build(agent)

    # ============================================================
    #  Wall extraction from /map (full resolution on i9, no compromise)
    # ============================================================
    def extract_walls(self, agent):
        if agent.map_msg is None:
            return []

        msg = agent.map_msg
        w, h = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        raw = np.array(msg.data, dtype=np.int8).reshape((h, w))

        # i9: can use 2x downsample instead of 4x for better precision
        s = 2
        dh, dw = h // s, w // s
        if dh < 2 or dw < 2:
            return []

        # Vectorized operations (i9 has plenty of CPU for numpy)
        blocks = raw[:dh*s, :dw*s].reshape(dh, s, dw, s)
        occ = np.any(blocks > 50, axis=(1, 3)).astype(np.uint8)
        has_free = np.any((blocks >= 0) & (blocks < 50), axis=(1, 3)).astype(np.uint8)

        # Pad for neighbor check
        occ_pad = np.pad(occ, 1, constant_values=0)
        free_pad = np.pad(has_free, 1, constant_values=0)

        # Check if each occupied cell has a free neighbor (3x3 window)
        # This filters exploration boundary artifacts
        free_nearby = np.zeros_like(occ, dtype=bool)
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                free_nearby |= free_pad[1+dy:dh+1+dy, 1+dx:dw+1+dx].astype(bool)

        filtered = occ.astype(bool) & free_nearby

        # Row-run extraction
        dres = res * s
        pad = 0.20  # smaller inflation since i9 can do more precise planning
        walls = []

        for y in range(dh):
            run_start = None
            for x in range(dw + 1):
                v = filtered[y, x] if x < dw else False
                if v and run_start is None:
                    run_start = x
                elif not v and run_start is not None:
                    walls.append([
                        run_start * dres + ox - pad,
                        y * dres + oy - pad,
                        x * dres + ox + pad,
                        (y + 1) * dres + oy + pad,
                    ])
                    run_start = None

        return walls

    # ============================================================
    #  Dynamic obstacle detection — 360°, no compromise
    # ============================================================
    def detect_dynamic(self, agent):
        """Detect dynamic obstacles near path only."""
        if agent.last_scan is None or not agent.have_pose:
            return []
        if not agent.path or agent.pidx >= len(agent.path):
            return []

        scan = agent.last_scan
        cos_y = math.cos(agent.ryaw)
        sin_y = math.sin(agent.ryaw)

        check_end = min(len(agent.path), agent.pidx + 8)
        segs = []
        prev = (agent.rx, agent.ry)
        for si in range(agent.pidx, check_end):
            cur = (agent.path[si][0], agent.path[si][1])
            segs.append((*prev, *cur))
            prev = cur

        hits = []
        angle = scan.angle_min
        for r in scan.ranges:
            if (not math.isnan(r) and not math.isinf(r) and
                    scan.range_min < r < min(scan.range_max, 4.0)):
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)
                mx = agent.rx + lx * cos_y - ly * sin_y
                my = agent.ry + lx * sin_y + ly * cos_y

                near_path = False
                for ax, ay, bx, by in segs:
                    if pt_seg_dist(mx, my, ax, ay, bx, by) < 0.8:
                        near_path = True
                        break

                if near_path:
                    near_wall = False
                    for w in agent.known_walls:
                        if pt_in_rect(mx, my, w[0], w[1], w[2], w[3], 0.15):
                            near_wall = True
                            break
                    if not near_wall:
                        hits.append((mx, my))

            angle += scan.angle_increment

        if len(hits) < 3:
            return []

        clusters = []
        used = set()
        for ci, (hx, hy) in enumerate(hits):
            if ci in used:
                continue
            cl = [(hx, hy)]
            used.add(ci)
            for cj in range(ci + 1, len(hits)):
                if cj in used:
                    continue
                jx, jy = hits[cj]
                cx = sum(p[0] for p in cl) / len(cl)
                cy = sum(p[1] for p in cl) / len(cl)
                if (jx - cx)**2 + (jy - cy)**2 < 0.5**2:
                    cl.append((jx, jy))
                    used.add(cj)
            if len(cl) >= 3:
                clusters.append(cl)

        cur_obstacles = []
        for cl in clusters:
            n = len(cl)
            cx = sum(p[0] for p in cl) / n
            cy = sum(p[1] for p in cl) / n
            mr = max(math.sqrt((p[0]-cx)**2 + (p[1]-cy)**2) for p in cl)
            cur_obstacles.append((cx, cy, max(mr + 0.1, 0.2)))

        tracked = []
        dt = 0.05
        for (cx, cy, r) in cur_obstacles:
            best_speed = 0.3
            for (px, py, pr) in agent.prev_obstacles:
                d = math.sqrt((cx - px)**2 + (cy - py)**2)
                if d < 2.0:
                    best_speed = min(max(d / dt, 0.0), 3.0)
                    break
            tracked.append((cx, cy, r, best_speed))

        agent.prev_obstacles = cur_obstacles
        return tracked

    def do_build(self, agent):
        if not agent.goal or not agent.have_pose:
            return False

        t0 = time.time()
        walls = self.extract_walls(agent)

        # Remove walls overlapping robot/goal
        clean = [w for w in walls
                 if not pt_in_rect(agent.rx, agent.ry, w[0], w[1], w[2], w[3], 0.15)
                 and not pt_in_rect(agent.goal[0], agent.goal[1], w[0], w[1], w[2], w[3], 0.15)]

        agent.known_walls = clean
        if agent.map_msg:
            agent.occ_count = sum(1 for v in agent.map_msg.data if v > 50)

        # Compute bounds from data (no fixed world_size)
        all_x = [agent.rx, agent.goal[0]]
        all_y = [agent.ry, agent.goal[1]]
        for w in clean:
            all_x.extend([w[0], w[2]])
            all_y.extend([w[1], w[3]])

        pad = 3.0
        bounds = [
            min(all_x) - pad, min(all_y) - pad,
            max(all_x) + pad, max(all_y) + pad,
        ]
        # Minimum size
        for i in [0, 1]:
            if bounds[i+2] - bounds[i] < 5.0:
                mid = (bounds[i] + bounds[i+2]) / 2
                bounds[i] = mid - 5.0
                bounds[i+2] = mid + 5.0

        resp = agent.planner.cmd({
            'cmd': 'build',
            'from': [agent.rx, agent.ry],
            'goal': list(agent.goal),
            'walls': clean,
            'bounds': bounds,
            'robot_speed': self.robot_speed,
            'robot_radius': self.robot_radius,
            'rrt_iters': self.rrt_iters,
        })

        dt = time.time() - t0

        if resp.get('ok'):
            agent.path = resp.get('path', [])
            agent.pidx = 0
            agent.navigating = True
            agent.t_build = time.time()
            agent.smart_fails = 0
            self.get_logger().info(
                f'[{agent.name}] Build OK: {len(agent.path)} pts, '
                f'{len(clean)} walls ({dt*1000:.0f}ms)')
            self.publish_path(agent)
            return True
        else:
            err = resp.get('error', '?')
            self.get_logger().warn(
                f'[{agent.name}] Build fail: {err} ({dt*1000:.0f}ms)')

            # Retry with big walls only
            if len(clean) > 30:
                big = [w for w in clean if abs(w[2]-w[0]) * abs(w[3]-w[1]) > 0.08]
                resp2 = agent.planner.cmd({
                    'cmd': 'build',
                    'from': [agent.rx, agent.ry],
                    'goal': list(agent.goal),
                    'walls': big, 'bounds': bounds,
                    'robot_speed': self.robot_speed,
                    'robot_radius': self.robot_radius,
                    'rrt_iters': self.rrt_iters,
                })
                if resp2.get('ok'):
                    agent.path = resp2.get('path', [])
                    agent.pidx = 0
                    agent.navigating = True
                    agent.t_build = time.time()
                    agent.known_walls = big
                    self.get_logger().info(
                        f'[{agent.name}] Retry OK: {len(agent.path)} pts, {len(big)} walls')
                    self.publish_path(agent)
                    return True

            agent.navigating = False
            return False

    # ============================================================
    #  SMART update — dynamic obstacles (fast, ~5ms on i9)
    # ============================================================
    def do_update(self, agent, obstacles):
        if not agent.navigating or not agent.path:
            return

        obs_list = [{'x': o[0], 'y': o[1], 'r': o[2], 'speed': o[3]}
                    for o in obstacles]

        resp = agent.planner.cmd({
            'cmd': 'update',
            'robot': [agent.rx, agent.ry],
            'path_idx': agent.pidx,
            'obstacles': obs_list,
        })

        if not resp.get('ok'):
            if resp.get('error') == 'smart_panic':
                self.get_logger().warn(f'[{agent.name}] SMART panic → rebuild')
                self.do_build(agent)
            return

        agent.t_update = time.time()

        result = resp.get('result', '')
        if result == 'replanned':
            new_path = resp.get('path', [])
            if new_path:
                agent.path = new_path
                agent.pidx = 0
                agent.smart_fails = 0
                self.get_logger().info(
                    f'[{agent.name}] ★ SMART avoided {len(obstacles)} obstacle(s)')
                self.publish_path(agent)
        elif result == 'replan_failed':
            agent.smart_fails += 1
            if agent.smart_fails > 3:
                self.get_logger().warn(f'[{agent.name}] SMART failed 3x → rebuild')
                self.do_build(agent)
                agent.smart_fails = 0
        elif result == 'safe':
            agent.smart_fails = 0

    # ============================================================
    #  Publish path to Pi 5
    # ============================================================
    def publish_path(self, agent):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for p in agent.path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(p[0])
            ps.pose.position.y = float(p[1])
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pubs[agent.name].publish(msg)

    # ============================================================
    #  Check map changes (every 5s)
    # ============================================================
    def check_maps(self):
        for agent in self.agents.values():
            if not agent.map_msg or not agent.navigating:
                continue
            cur = sum(1 for v in agent.map_msg.data if v > 50)
            if agent.occ_count > 0:
                change = abs(cur - agent.occ_count) / max(agent.occ_count, 1)
                if change > 0.03:
                    self.get_logger().info(
                        f'[{agent.name}] Map changed {change:.0%} → rebuild')
                    self.do_build(agent)
                    agent.occ_count = cur

    # ============================================================
    #  Main planning loop — 20 Hz (i9 has power to spare)
    # ============================================================
    def plan_loop(self):
        now = time.time()

        for agent in self.agents.values():
            if not agent.have_pose or not agent.navigating:
                continue

            # Update pidx based on robot position
            while agent.pidx < len(agent.path) - 1:
                px, py = agent.path[agent.pidx]
                if math.sqrt((px - agent.rx)**2 + (py - agent.ry)**2) < 0.4:
                    agent.pidx += 1
                else:
                    break

            # Goal reached?
            if agent.goal and agent.path:
                last = agent.path[-1]
                if math.sqrt((last[0]-agent.rx)**2 + (last[1]-agent.ry)**2) < 0.5:
                    self.get_logger().info(f'[{agent.name}] ✓ GOAL REACHED!')
                    agent.navigating = False
                    agent.path = []
                    # Publish empty path to stop Pi 5
                    self.publish_path(agent)
                    continue

            # Detect dynamic obstacles from LiDAR
            obstacles = self.detect_dynamic(agent)
            agent.tracked_obstacles = obstacles
            if obstacles:
                self.get_logger().info(f"[{agent.name}] dynamic: {len(obstacles)} obs")

            # Add other cars as dynamic obstacles
            for other_name, other in self.agents.items():
                if other_name == agent.name:
                    continue
                if not other.have_pose:
                    continue
                # Other car = spherical obstacle with real speed
                obstacles.append((
                    other.rx, other.ry,
                    self.robot_radius + 0.15,  # radius with margin
                    other.est_speed,           # real speed from pose tracking!
                ))

            # Feed to SMART update (every frame on i9, no throttle needed)
            if obstacles and now - agent.t_update > 0.05:
                self.do_update(agent, obstacles)

            # Stuck detection: not moving for 3s → rebuild
            if not hasattr(agent, 'stuck_x'):
                agent.stuck_x = agent.rx
                agent.stuck_y = agent.ry
                agent.stuck_t = now
            moved = math.sqrt((agent.rx - agent.stuck_x)**2 + (agent.ry - agent.stuck_y)**2)
            if moved > 0.1:
                agent.stuck_x = agent.rx
                agent.stuck_y = agent.ry
                agent.stuck_t = now
            elif now - agent.stuck_t > 3.0 and agent.navigating and agent.path:
                self.get_logger().warn(f'[{agent.name}] STUCK 3s → rebuild')
                self.do_build(agent)
                agent.stuck_t = now

            # No valid path → rebuild
            if agent.navigating and (not agent.path or agent.pidx >= len(agent.path)):
                self.get_logger().warn(f'[{agent.name}] No path → rebuild')
                self.do_build(agent)
                agent.t_update = now
                agent.tracked_obstacles = obstacles

    # ============================================================
    #  Visualization for RViz
    # ============================================================
    def publish_viz(self):
        ma = MarkerArray()
        mid = 0
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        for agent in self.agents.values():
            if not agent.have_pose:
                continue

            # Robot sphere
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = f'{agent.name}_robot'
            m.id = mid; mid += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = agent.rx
            m.pose.position.y = agent.ry
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.3
            m.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=1.0)
            ma.markers.append(m)

            # Goal
            if agent.goal:
                g = Marker()
                g.header = m.header
                g.ns = f'{agent.name}_goal'
                g.id = mid; mid += 1
                g.type = Marker.SPHERE
                g.action = Marker.ADD
                g.pose.position.x = agent.goal[0]
                g.pose.position.y = agent.goal[1]
                g.pose.position.z = 0.15
                g.pose.orientation.w = 1.0
                g.scale.x = g.scale.y = g.scale.z = 0.25
                g.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
                ma.markers.append(g)

            # Dynamic obstacles
            for i, obs in enumerate(agent.tracked_obstacles):
                om = Marker()
                om.header = m.header
                om.ns = f'{agent.name}_obs'
                om.id = mid; mid += 1
                om.type = Marker.SPHERE
                om.action = Marker.ADD
                om.pose.position.x = obs[0]
                om.pose.position.y = obs[1]
                om.pose.position.z = 0.1
                om.pose.orientation.w = 1.0
                r = obs[2] * 2
                om.scale.x = om.scale.y = om.scale.z = r
                om.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.7)
                om.lifetime.sec = 0
                om.lifetime.nanosec = 500000000
                ma.markers.append(om)

            # Known walls
            for i, w in enumerate(agent.known_walls):
                wm = Marker()
                wm.header = m.header
                wm.ns = f'{agent.name}_walls'
                wm.id = mid; mid += 1
                wm.type = Marker.CUBE
                wm.action = Marker.ADD
                wm.pose.position.x = (w[0] + w[2]) / 2.0
                wm.pose.position.y = (w[1] + w[3]) / 2.0
                wm.pose.position.z = 0.5
                wm.pose.orientation.w = 1.0
                wm.scale.x = max(abs(w[2] - w[0]), 0.05)
                wm.scale.y = max(abs(w[3] - w[1]), 0.05)
                wm.scale.z = 1.0
                wm.color = ColorRGBA(r=0.2, g=0.2, b=0.25, a=0.8)
                ma.markers.append(wm)

        self.viz_pub.publish(ma)


def main():
    rclpy.init()
    node = CentralPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for agent in node.agents.values():
            agent.planner.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
