#!/usr/bin/env python3
"""SMART-3D Nav v4
- Cartographer /map for walls (only extracted on build, not every frame)
- LiDAR line-segment check for path blocked detection
- Rebuild when blocked
- Pure Pursuit path following
"""
import math, json, subprocess, threading, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import tf2_ros

class SmartNavNode(Node):
    def __init__(self):
        super().__init__('smart_nav_node')
        self.declare_parameter('world_size', 20.0)
        self.declare_parameter('robot_speed', 0.2)
        self.declare_parameter('rrt_iterations', 8000)
        self.ws = self.get_parameter('world_size').value
        self.spd = self.get_parameter('robot_speed').value
        self.iters = self.get_parameter('rrt_iterations').value
        self.rx, self.ry, self.ryaw = 0.0, 0.0, 0.0
        self.got_pose = False
        self.goal = None
        self.path = []
        self.pidx = 0
        self.navigating = False
        self.last_scan = None
        self.last_build = 0.0
        self.carto_map = None

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)

        sq = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self.on_scan, sq)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        mq = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, mq)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/smart/path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/smart/markers', 10)

        self.proc = None
        self.plock = threading.Lock()
        self._start_planner()
        self.create_timer(0.1, self.nav_loop)
        self.create_timer(0.5, self.pub_viz)
        self.get_logger().info('SMART Nav v4 ws={} spd={}'.format(self.ws, self.spd))

    def _start_planner(self):
        try:
            self.proc = subprocess.Popen(
                ['/root/SMART3D/target/release/examples/smart_planner_server'],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, text=True, bufsize=1)
            d = json.loads(self.proc.stdout.readline().strip())
            if d.get('status') == 'ready':
                self.get_logger().info('Planner OK')
        except Exception as e:
            self.get_logger().error(str(e))

    def _pcmd(self, cmd):
        if not self.proc: return {'status': 'no_planner'}
        try:
            with self.plock:
                self.proc.stdin.write(json.dumps(cmd) + chr(10))
                self.proc.stdin.flush()
                return json.loads(self.proc.stdout.readline().strip())
        except: return {'status': 'error'}

    def _get_pose(self):
        try:
            t = self.tf_buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.rx = t.transform.translation.x
            self.ry = t.transform.translation.y
            q = t.transform.rotation
            self.ryaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.got_pose = True
            return True
        except: return False

    def on_scan(self, msg):
        self.last_scan = msg
        self._get_pose()

    def on_map(self, msg):
        self.carto_map = msg  # just store, don't process

    def on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.navigating = True
        self.pidx = 0
        self.get_logger().info('Goal: ({:.2f},{:.2f})'.format(*self.goal))
        self._build_plan()

    def _extract_walls(self):
        """Extract walls from cartographer map. Downsample 4x for speed."""
        msg = self.carto_map
        if msg is None:
            return []
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        # Downsample 4x
        s = 4
        dh, dw = h // s, w // s
        occ = np.zeros((dh, dw), dtype=np.uint8)
        for dy in range(dh):
            for dx in range(dw):
                block = data[dy*s:(dy+1)*s, dx*s:(dx+1)*s]
                if np.any(block > 50):
                    occ[dy, dx] = 1
        dres = res * s
        # Row-run extraction
        walls = []
        for y in range(dh):
            x = 0
            while x < dw:
                if occ[y, x]:
                    x0 = x
                    while x < dw and occ[y, x]:
                        x += 1
                    walls.append([
                        x0 * dres + ox,
                        y * dres + oy,
                        x * dres + ox,
                        (y + 1) * dres + oy])
                else:
                    x += 1
        return walls

    def _build_plan(self):
        if not self.goal or not self.got_pose: return
        walls = self._extract_walls()
        r = self._pcmd({
            'cmd': 'build', 'from': [self.rx, self.ry],
            'goal': list(self.goal), 'walls': walls,
            'world_size': self.ws, 'speed': self.spd, 'iters': self.iters})
        if r.get('status') == 'ok':
            self.path = r.get('path', [])
            self.pidx = 0
            self.get_logger().info('Plan: {} pts, {} walls'.format(len(self.path), len(walls)))
        else:
            self.get_logger().warn('Plan fail: ' + r.get('status', '?'))
        self.last_build = time.time()

    def _check_blocked(self):
        """Check if LiDAR hits are near path line segments."""
        if not self.last_scan or not self.path: return False
        scan = self.last_scan
        hits = []
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < min(scan.range_max, 3.0) and not (math.isnan(r) or math.isinf(r)):
                hits.append((self.rx + r * math.cos(self.ryaw + angle),
                             self.ry + r * math.sin(self.ryaw + angle)))
            angle += scan.angle_increment
        if not hits: return False
        # Check line segments: robot -> path[pidx] -> ... -> path[pidx+5]
        end = min(self.pidx + 6, len(self.path))
        pts = [(self.rx, self.ry)] + [self.path[i] for i in range(self.pidx, end)]
        for i in range(len(pts)-1):
            ax, ay = pts[i]
            bx, by = pts[i+1]
            dx, dy = bx-ax, by-ay
            L2 = dx*dx + dy*dy
            for hx, hy in hits:
                if L2 < 0.01:
                    d2 = (hx-ax)**2 + (hy-ay)**2
                else:
                    t = max(0.0, min(1.0, ((hx-ax)*dx+(hy-ay)*dy)/L2))
                    px = ax + t*dx
                    py = ay + t*dy
                    d2 = (hx-px)**2 + (hy-py)**2
                if d2 < 0.35**2:
                    return True
        # Emergency: front 30cm
        angle = scan.angle_min
        for r in scan.ranges:
            if abs(angle) < 0.5 and scan.range_min < r < 0.3 and not math.isnan(r):
                return True
            angle += scan.angle_increment
        return False

    def nav_loop(self):
        if not self.navigating or not self.goal or not self.got_pose: return
        # Blocked? Stop + rebuild with latest map
        if self.path and self._check_blocked():
            self.cmd_pub.publish(Twist())
            if time.time() - self.last_build > 2.0:
                self.get_logger().warn('PATH BLOCKED! Rebuild')
                self._build_plan()
            return
        if not self.path: return
        # Skip passed points
        while self.pidx < len(self.path) - 1:
            dx = self.path[self.pidx][0] - self.rx
            dy = self.path[self.pidx][1] - self.ry
            if math.sqrt(dx*dx + dy*dy) < 1.0: self.pidx += 1
            else: break
        # Goal reached?
        gx, gy = self.path[-1]
        if math.sqrt((gx-self.rx)**2 + (gy-self.ry)**2) < 0.8:
            self.get_logger().info('GOAL REACHED!')
            self.navigating = False
            self.cmd_pub.publish(Twist()); return
        # Lookahead
        ti = min(self.pidx, len(self.path)-1)
        for i in range(self.pidx, len(self.path)):
            dx = self.path[i][0] - self.rx
            dy = self.path[i][1] - self.ry
            if math.sqrt(dx*dx+dy*dy) >= 1.0: ti = i; break
        else: ti = len(self.path)-1
        tx, ty = self.path[ti]
        ae = math.atan2(ty-self.ry, tx-self.rx) - self.ryaw
        while ae > math.pi: ae -= 2*math.pi
        while ae < -math.pi: ae += 2*math.pi
        if abs(ae) > 0.8:
            v, w = 0.0, 1.0 * (1 if ae > 0 else -1)
        else:
            v = self.spd * (1 - abs(ae)/1.5)
            v = max(0.05, min(v, self.spd))
            w = max(-1.0, min(1.0, 2*v*math.sin(ae)/1.0))
        t = Twist(); t.linear.x = v; t.angular.z = w
        self.cmd_pub.publish(t)
        # Publish path
        pm = Path(); pm.header.frame_id = 'map'
        pm.header.stamp = self.get_clock().now().to_msg()
        ps0 = PoseStamped(); ps0.header = pm.header
        ps0.pose.position.x = self.rx; ps0.pose.position.y = self.ry
        ps0.pose.orientation.w = 1.0; pm.poses.append(ps0)
        for pt in self.path[self.pidx:]:
            ps = PoseStamped(); ps.header = pm.header
            ps.pose.position.x = float(pt[0]); ps.pose.position.y = float(pt[1])
            ps.pose.orientation.w = 1.0; pm.poses.append(ps)
        self.path_pub.publish(pm)

    def pub_viz(self):
        ma = MarkerArray()
        s = self.get_clock().now().to_msg()
        if self.goal:
            m = Marker(); m.header.frame_id='map'; m.header.stamp=s
            m.ns='goal'; m.id=0; m.type=Marker.CYLINDER; m.action=Marker.ADD
            m.pose.position.x=self.goal[0]; m.pose.position.y=self.goal[1]
            m.pose.orientation.w=1.0; m.scale.x=m.scale.y=0.4; m.scale.z=0.2
            m.color=ColorRGBA(r=1.0,g=0.2,b=0.2,a=0.8); ma.markers.append(m)
        self.marker_pub.publish(ma)

    def destroy_node(self):
        try: self.cmd_pub.publish(Twist())
        except: pass
        if self.proc:
            try: self._pcmd({'cmd':'quit'}); self.proc.wait(2)
            except: self.proc.kill()
        super().destroy_node()

def main():
    rclpy.init()
    node = SmartNavNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.try_shutdown()

if __name__ == '__main__': main()
