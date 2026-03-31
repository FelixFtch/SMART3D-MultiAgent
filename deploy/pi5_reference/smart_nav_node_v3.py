#!/usr/bin/env python3
"""SMART-3D Nav v3 - Minimal: no grid, no clustering, just block-and-learn"""
import math, json, subprocess, threading, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Path
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
        self.walls = []
        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
        sq = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self.on_scan, sq)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/smart/path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/smart/markers', 10)
        self.proc = None
        self.plock = threading.Lock()
        self._start_planner()
        self.create_timer(0.1, self.nav_loop)
        self.create_timer(0.5, self.pub_viz)
        self.get_logger().info('SMART Nav v3 ws={} spd={}'.format(self.ws, self.spd))

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

    def on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.navigating = True
        self.pidx = 0
        self.get_logger().info('Goal: ({:.2f},{:.2f})'.format(*self.goal))
        self._build_plan()

    def _build_plan(self):
        if not self.goal or not self.got_pose: return
        r = self._pcmd({
            'cmd': 'build', 'from': [self.rx, self.ry],
            'goal': list(self.goal), 'walls': self.walls,
            'world_size': self.ws, 'speed': self.spd, 'iters': self.iters})
        if r.get('status') == 'ok':
            self.path = r.get('path', [])
            self.pidx = 0
            self.get_logger().info('Plan: {} pts, {} walls'.format(len(self.path), len(self.walls)))
        else:
            self.get_logger().warn('Plan fail: ' + r.get('status', '?'))
        self.last_build = time.time()

    def _check_blocked(self):
        if not self.last_scan or not self.path: return []
        scan = self.last_scan
        hits = []
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < min(scan.range_max, 3.0) and not (math.isnan(r) or math.isinf(r)):
                hits.append((self.rx + r * math.cos(self.ryaw + angle),
                             self.ry + r * math.sin(self.ryaw + angle)))
            angle += scan.angle_increment
        if not hits: return []
        blockers = []
        end = min(self.pidx + 6, len(self.path))
        for pi in range(self.pidx, end):
            px, py = self.path[pi]
            for hx, hy in hits:
                if (px-hx)**2 + (py-hy)**2 < 0.5**2:
                    blockers.append((hx, hy))
        angle = scan.angle_min
        for r in scan.ranges:
            if abs(angle) < 0.5 and scan.range_min < r < 0.3 and not math.isnan(r):
                blockers.append((self.rx + r * math.cos(self.ryaw + angle),
                                 self.ry + r * math.sin(self.ryaw + angle)))
            angle += scan.angle_increment
        return blockers

    def _add_walls(self, pts):
        added = 0
        for hx, hy in pts:
            dup = False
            for w in self.walls:
                cx, cy = (w[0]+w[2])/2, (w[1]+w[3])/2
                if (hx-cx)**2 + (hy-cy)**2 < 0.3**2:
                    dup = True; break
            if not dup:
                self.walls.append([hx-0.25, hy-0.25, hx+0.25, hy+0.25])
                added += 1
        return added

    def nav_loop(self):
        if not self.navigating or not self.goal or not self.got_pose: return
        if self.path:
            blockers = self._check_blocked()
            if blockers:
                self.cmd_pub.publish(Twist())
                if time.time() - self.last_build > 2.0:
                    n = self._add_walls(blockers)
                    self.get_logger().warn('BLOCKED +{} walls={}'.format(n, len(self.walls)))
                    self._build_plan()
                return
        if not self.path: return
        while self.pidx < len(self.path) - 1:
            dx = self.path[self.pidx][0] - self.rx
            dy = self.path[self.pidx][1] - self.ry
            if math.sqrt(dx*dx + dy*dy) < 1.0: self.pidx += 1
            else: break
        gx, gy = self.path[-1]
        if math.sqrt((gx-self.rx)**2 + (gy-self.ry)**2) < 0.8:
            self.get_logger().info('GOAL REACHED!')
            self.navigating = False
            self.cmd_pub.publish(Twist()); return
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
        m = Marker(); m.header.frame_id='map'; m.header.stamp=s
        m.ns='walls'; m.id=0; m.type=Marker.LINE_LIST; m.action=Marker.ADD
        m.scale.x=0.03; m.color=ColorRGBA(r=1.0,g=0.5,b=0.0,a=0.7)
        m.pose.orientation.w=1.0
        for w in self.walls:
            for p in [(w[0],w[1]),(w[2],w[1]),(w[2],w[1]),(w[2],w[3]),
                       (w[2],w[3]),(w[0],w[3]),(w[0],w[3]),(w[0],w[1])]:
                pt=Point(); pt.x=p[0]; pt.y=p[1]; pt.z=0.05; m.points.append(pt)
        ma.markers.append(m)
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
