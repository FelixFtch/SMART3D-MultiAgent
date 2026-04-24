#!/usr/bin/env python3
"""
SMART-3D Multi-Agent Shared Unknown v2 → RViz2 Bridge

Visualizes:
  - Robots: colored sphere + collision radius ring + direction arrow + name label
  - Sensor range: filled semi-transparent circle in agent color
  - Paths: colored lines per agent
  - People: orange spheres + yellow OHZ rings
  - Walls: discovered (dark) / undiscovered (faint)
  - Fog-of-war occupancy grid
  - Tree edges colored by agent (optional)
  - HUD with per-agent status

Usage:
  cargo run --release --example smart_multi_agent_shared_unknown_v2 -- \
    --scenario mega_airport --num-agents 4 --agent-mode shared_unknown_corners \
    -n 0 --playback-speed 1 --sensor-range 12 --rrt-iterations 50000 \
    --lsr-max 80 --include-tree \
    | python3 ~/smart_rviz_bridge/smart_multi_agent_shared_unknown_v2_bridge.py
"""

import os
import sys
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA


def rgba(r, g, b, a=1.0):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


COLOR_WALL_SEEN    = rgba(0.0, 0.0, 0.0, 0.92)
COLOR_WALL_UNSEEN  = rgba(0.25, 0.25, 0.25, 0.22)
COLOR_GROUND       = rgba(1.0, 1.0, 1.0, 0.0)
COLOR_BORDER       = rgba(0.0, 0.0, 0.0, 0.65)
COLOR_OBSTACLE     = rgba(0.95, 0.45, 0.1, 0.85)
COLOR_OHZ          = rgba(1.0, 0.85, 0.0, 0.10)
COLOR_TREE_MAIN    = rgba(1.0, 0.6, 0.0, 0.35)
COLOR_TREE_DISJOINT = rgba(1.0, 0.5, 0.0, 0.12)
COLOR_DEPLOY_ORIGIN = rgba(1.0, 1.0, 1.0, 0.9)
COLOR_DEPLOY_HINT = rgba(1.0, 0.95, 0.5, 0.85)

Z_GROUND = -0.01
Z_TREE   = 0.05
Z_SENSOR = 0.03
Z_PATH   = 0.20
Z_ROBOT  = 0.5
Z_GOAL   = 0.3
Z_TEXT   = 3.0
WALL_H   = 2.5


class MultiAgentBridge(Node):
    def __init__(self):
        super().__init__('smart_multi_agent_shared_unknown_v2_bridge')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.latched_qos = latched
        self.tf_pub = TransformBroadcaster(self)
        self.static_tf_pub = StaticTransformBroadcaster(self)
        self.static_pub = self.create_publisher(MarkerArray, '/smart/static', latched)
        self.marker_pub = self.create_publisher(MarkerArray, '/smart/markers', 10)
        self.grid_pub   = self.create_publisher(OccupancyGrid, '/smart/map', latched)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.map_campus_pub = self.create_publisher(OccupancyGrid, '/map_campus', latched)
        self.map_by_car_pubs = {}
        self.path_pubs = {}
        self.path_display_pubs = {}
        self.pose_pubs = {}
        self.scan_pubs = {}
        self.goal_subs = {}
        self.goal_override_path = os.environ.get(
            'SMART_SHARED_UNKNOWN_V2_GOAL_PATH',
            '/tmp/smart_shared_unknown_v2_goal_overrides.json',
        )
        self.goal_overrides = {}
        self.agent_yaws = {}
        self.metadata = None
        self.frame_count = 0
        self.walls_data = []
        self.grid_res = 0.5
        self.grid_w = 64
        self.grid_h = 64
        self.robot_radius = 0.5
        self.agent_colors = {}
        self.num_agents = 0
        self.current_stamp = None
        try:
            os.remove(self.goal_override_path)
        except FileNotFoundError:
            pass
        self.get_logger().info('SMART Multi-Agent Shared Unknown v2 RViz2 Bridge')
        self.get_logger().info(f'Goal override path: {self.goal_override_path}')

    def make_marker(self, ns, mid, mtype, x=0, y=0, z=0, sx=1, sy=1, sz=1, color=None):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.current_stamp or self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.pose.orientation.w = 1.0
        m.scale.x = float(sx)
        m.scale.y = float(sy)
        m.scale.z = float(sz)
        if color:
            m.color = color
        m.lifetime.sec = 0
        m.lifetime.nanosec = 0
        return m

    def _write_goal_overrides(self):
        tmp = f'{self.goal_override_path}.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(self.goal_overrides, f, indent=2, sort_keys=True)
        os.replace(tmp, self.goal_override_path)

    def _on_goal_pose(self, name, msg):
        gx = float(msg.pose.position.x)
        gy = float(msg.pose.position.y)
        self.goal_overrides[name] = [gx, gy]
        self._write_goal_overrides()
        self.get_logger().info(f'[{name}] goal override -> ({gx:.2f}, {gy:.2f})')

    def _clear_dynamic_markers(self):
        # RViz keeps marker ids alive forever unless explicitly deleted. Clear the
        # volatile live layer before adding the current frame so old robot poses
        # cannot look like sticky duplicate agents.
        clear = MarkerArray()
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.current_stamp or self.get_clock().now().to_msg()
        m.action = Marker.DELETEALL
        clear.markers.append(m)
        self.marker_pub.publish(clear)

    @staticmethod
    def _ray_rect_hit(ox, oy, dx, dy, x1, y1, x2, y2):
        best = None
        if abs(dx) > 1e-8:
            for x in (x1, x2):
                t = (x - ox) / dx
                if t > 0.001:
                    y = oy + t * dy
                    if y1 <= y <= y2:
                        best = t if best is None else min(best, t)
        if abs(dy) > 1e-8:
            for y in (y1, y2):
                t = (y - oy) / dy
                if t > 0.001:
                    x = ox + t * dx
                    if x1 <= x <= x2:
                        best = t if best is None else min(best, t)
        return best

    @staticmethod
    def _ray_circle_hit(ox, oy, dx, dy, cx, cy, radius):
        lx = ox - cx
        ly = oy - cy
        b = 2.0 * (dx * lx + dy * ly)
        c = lx * lx + ly * ly - radius * radius
        disc = b * b - 4.0 * c
        if disc < 0.0:
            return None
        root = math.sqrt(disc)
        t1 = (-b - root) / 2.0
        t2 = (-b + root) / 2.0
        candidates = [t for t in (t1, t2) if t > 0.001]
        return min(candidates) if candidates else None

    def _estimate_yaw(self, name, rx, ry, path_pts, path_idx):
        last = self.agent_yaws.get(name, 0.0)
        if path_pts:
            target_idx = min(max(path_idx, 0), len(path_pts) - 1)
            tx, ty = path_pts[target_idx]
            dx = float(tx) - float(rx)
            dy = float(ty) - float(ry)
            if math.hypot(dx, dy) > 0.05:
                last = math.atan2(dy, dx)
        self.agent_yaws[name] = last
        return last

    def _publish_tf(self, name, rx, ry, yaw):
        tf = TransformStamped()
        tf.header.frame_id = 'map'
        tf.header.stamp = self.current_stamp or self.get_clock().now().to_msg()
        tf.child_frame_id = f'{name}/base_link'
        tf.transform.translation.x = float(rx)
        tf.transform.translation.y = float(ry)
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = math.sin(yaw / 2.0)
        tf.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_pub.sendTransform(tf)

    def _publish_scan(self, name, rx, ry, yaw, sensor_range, obs_list):
        if name not in self.scan_pubs:
            return
        msg = LaserScan()
        msg.header.frame_id = f'{name}/laser_link'
        msg.header.stamp = self.current_stamp or self.get_clock().now().to_msg()
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        beams = 180
        msg.angle_increment = (msg.angle_max - msg.angle_min) / float(beams - 1)
        msg.range_min = 0.15
        msg.range_max = float(sensor_range)
        msg.scan_time = 0.1
        msg.time_increment = msg.scan_time / float(beams)
        ranges = []
        for i in range(beams):
            local_ang = msg.angle_min + i * msg.angle_increment
            world_ang = yaw + local_ang
            dx = math.cos(world_ang)
            dy = math.sin(world_ang)
            best = float(sensor_range)
            for wall in self.walls_data:
                hit = self._ray_rect_hit(rx, ry, dx, dy, wall[0], wall[1], wall[2], wall[3])
                if hit is not None and hit < best:
                    best = hit
            for obs in obs_list:
                hit = self._ray_circle_hit(rx, ry, dx, dy, obs['x'], obs['y'], obs['radius'])
                if hit is not None and hit < best:
                    best = hit
            ranges.append(max(msg.range_min, min(best, msg.range_max)))
        msg.ranges = ranges
        self.scan_pubs[name].publish(msg)

    def process_metadata(self, data):
        self.metadata = data
        self.walls_data = data.get('walls', [])
        self.grid_res = data.get('grid_res', 0.5)
        self.grid_w = data.get('grid_w', 64)
        self.grid_h = data.get('grid_h', 64)
        self.robot_radius = data.get('robot_radius', 0.5)
        ws = float(data.get('world_size', 32))
        agents_meta = data.get('agents', [])
        self.num_agents = len(agents_meta)
        mode = data.get('agent_mode', '?')
        scenario = data.get('scenario', '?')
        n_obs = data.get('num_obstacles', 0)
        deployment_origin = data.get('deployment_origin', [ws / 2.0, ws / 2.0])
        deployment_layout = data.get('deployment_layout', 'distributed')
        initial_tree_scope = data.get('initial_tree_scope', 'shared')
        initial_sensor_range = float(data.get('initial_sensor_range', data.get('sensor_range', 10.0)))

        for a in agents_meta:
            name = a['name']
            c = a.get('color', [1, 1, 1])
            self.agent_colors[name] = (c[0], c[1], c[2])
            self.map_by_car_pubs[name] = self.create_publisher(OccupancyGrid, f'/{name}/map', self.latched_qos)
            self.path_pubs[name] = self.create_publisher(Path, f'/smart/path_{name}', 10)
            self.path_display_pubs[name] = self.create_publisher(Path, f'/{name}/path_display', 10)
            self.pose_pubs[name] = self.create_publisher(PoseStamped, f'/{name}/pose', 10)
            self.scan_pubs[name] = self.create_publisher(LaserScan, f'/{name}/scan', 10)
            if name not in self.goal_subs:
                self.goal_subs[name] = self.create_subscription(
                    PoseStamped,
                    f'/{name}/goal_pose',
                    lambda msg, n=name: self._on_goal_pose(n, msg),
                    10,
                )

        static_tfs = []
        for a in agents_meta:
            name = a['name']
            tf = TransformStamped()
            tf.header.frame_id = f'{name}/base_link'
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.child_frame_id = f'{name}/laser_link'
            tf.transform.translation.z = 0.05
            tf.transform.rotation.w = 1.0
            static_tfs.append(tf)
        if static_tfs:
            self.static_tf_pub.sendTransform(static_tfs)

        self.get_logger().info(
            f"{scenario} | {ws}x{ws}m | {self.num_agents} agents ({mode}) | "
            f"{len(self.walls_data)} walls | {n_obs} people")

        # Clear old
        clear = MarkerArray()
        for ns in ['static','walls','agents','paths','goals','sensor','tree',
                    'status','obstacles','ohz','collision']:
            c2 = Marker()
            c2.header.frame_id = 'map'
            c2.header.stamp = self.get_clock().now().to_msg()
            c2.ns = ns
            c2.action = Marker.DELETEALL
            clear.markers.append(c2)
        self.static_pub.publish(clear)
        self.marker_pub.publish(clear)
        import time; time.sleep(0.2)

        self._clear_dynamic_markers()
        markers = MarkerArray()
        mid = 0

        # Ground
        markers.markers.append(self.make_marker('static', mid, Marker.CUBE,
            x=ws/2, y=ws/2, z=Z_GROUND, sx=ws, sy=ws, sz=0.01, color=COLOR_GROUND))
        mid += 1

        # Border
        border = self.make_marker('static', mid, Marker.LINE_STRIP, z=0, sx=0.06, color=COLOR_BORDER)
        for cx, cy in [(0,0),(ws,0),(ws,ws),(0,ws),(0,0)]:
            p = Point(); p.x, p.y, p.z = float(cx), float(cy), 0.0
            border.points.append(p)
        markers.markers.append(border)
        mid += 1

        # Deployment origin marker
        dox, doy = deployment_origin
        origin = self.make_marker('static', mid, Marker.CYLINDER,
            x=dox, y=doy, z=0.08, sx=0.5, sy=0.5, sz=0.16, color=COLOR_DEPLOY_ORIGIN)
        markers.markers.append(origin)
        mid += 1
        # Paper-capture mode intentionally suppresses the deployment-origin
        # text label so screenshots stay clean.

        # Per-agent start + goal
        for a in agents_meta:
            name = a['name']
            c = self.agent_colors[name]
            color = rgba(c[0], c[1], c[2])
            color_dim = rgba(c[0], c[1], c[2], 1.0)
            sx, sy = a['start']
            gx, gy = a['goal']

            markers.markers.append(self.make_marker('static', mid, Marker.CYLINDER,
                x=sx, y=sy, z=0.25, sx=0.7, sy=0.7, sz=0.5, color=color_dim))
            mid += 1
            st = self.make_marker('static', mid, Marker.TEXT_VIEW_FACING,
                x=sx, y=sy-1.2, z=1.8, sx=0.55, sy=0.55, sz=0.55, color=color)
            st.text = f"{name} START"
            markers.markers.append(st)
            mid += 1

            markers.markers.append(self.make_marker('static', mid, Marker.CYLINDER,
                x=gx, y=gy, z=Z_GOAL, sx=1.0, sy=1.0, sz=0.8, color=color_dim))
            mid += 1
            gt = self.make_marker('static', mid, Marker.TEXT_VIEW_FACING,
                x=gx, y=gy-1.2, z=1.8, sx=0.55, sy=0.55, sz=0.55, color=color)
            gt.text = f"{name} GOAL"
            markers.markers.append(gt)
            mid += 1

        # Paper-capture mode suppresses title/subtitle HUD text. Start/goal
        # labels are kept because they are part of the figure semantics.

        # Faint full-scene wall context. The OccupancyGrid below still carries
        # the actual explored map: unknown=gray, free=white, discovered
        # wall=black. These context walls are intentionally translucent.
        for i, w in enumerate(self.walls_data):
            x1, y1, x2, y2 = w[0], w[1], w[2], w[3]
            cx = (x1+x2)/2; cy = (y1+y2)/2
            ssx = max(abs(x2-x1), 0.05); ssy = max(abs(y2-y1), 0.05)
            markers.markers.append(self.make_marker('walls_context', i, Marker.CUBE,
                x=cx, y=cy, z=0.005, sx=ssx, sy=ssy, sz=0.01, color=COLOR_WALL_UNSEEN))

        self.static_pub.publish(markers)
        grid_str = data.get('grid', '')
        if grid_str:
            self.publish_grid(grid_str)

    def publish_grid(self, grid_str):
        og = OccupancyGrid()
        og.header.frame_id = 'map'
        og.header.stamp = self.get_clock().now().to_msg()
        og.info.resolution = float(self.grid_res)
        og.info.width = self.grid_w
        og.info.height = self.grid_h
        og.info.origin.position.x = 0.0
        og.info.origin.position.y = 0.0
        og.info.origin.position.z = -0.02
        og.info.origin.orientation.w = 1.0
        d = []
        for ch in grid_str:
            if ch == 'F': d.append(0)
            elif ch == 'W': d.append(100)
            else: d.append(-1)
        og.data = d
        self.grid_pub.publish(og)
        self.map_pub.publish(og)
        self.map_campus_pub.publish(og)
        for pub in self.map_by_car_pubs.values():
            pub.publish(og)

    def process_frame(self, data):
        if self.metadata is None:
            return
        self.current_stamp = self.get_clock().now().to_msg()
        grid_str = data.get('grid')
        if grid_str and isinstance(grid_str, str):
            self.publish_grid(grid_str)

        markers = MarkerArray()
        agents = data.get('agents', [])
        ws = float(self.metadata.get('world_size', 32))
        sensor_range = data.get('sensor_range', self.metadata.get('sensor_range', 10.0))
        rr = data.get('robot_radius', self.robot_radius)
        obs_list = data.get('obstacles', [])

        for ai, a in enumerate(agents):
            name = a['name']
            rx, ry = a['robot']
            path_pts = a.get('path', [])
            path_idx = a.get('path_index', 0)
            reached = a.get('reached_goal', False)
            rebuilds = a.get('rebuilds', 0)
            c = a.get('color', [1,1,1])
            color = rgba(c[0], c[1], c[2])
            color_faint = rgba(c[0], c[1], c[2], 0.65)
            base = ai * 100
            yaw = self._estimate_yaw(name, rx, ry, path_pts, path_idx)
            self._publish_tf(name, rx, ry, yaw)

            # ---- Sensor range: filled circle ----
            if not reached:
                markers.markers.append(self.make_marker('sensor', base, Marker.CYLINDER,
                    x=rx, y=ry, z=Z_SENSOR,
                    sx=sensor_range*2, sy=sensor_range*2, sz=0.005,
                    color=rgba(c[0], c[1], c[2], 0.07)))
                ring = self.make_marker('sensor', base+1, Marker.LINE_STRIP,
                    z=0.01, sx=0.05, color=rgba(c[0], c[1], c[2], 0.4))
                for k in range(49):
                    ang = 2*math.pi*k/48
                    p = Point()
                    p.x = float(rx + sensor_range*math.cos(ang))
                    p.y = float(ry + sensor_range*math.sin(ang))
                    p.z = 0.01
                    ring.points.append(p)
                markers.markers.append(ring)
            else:
                # Delete sensor for finished
                d1 = self.make_marker('sensor', base, Marker.CYLINDER); d1.action = Marker.DELETE
                d2 = self.make_marker('sensor', base+1, Marker.LINE_STRIP); d2.action = Marker.DELETE
                markers.markers.extend([d1, d2])

            # ---- Robot sphere ----
            robot_color = rgba(c[0]*0.35, c[1]*0.35, c[2]*0.35, 0.4) if reached else color
            markers.markers.append(self.make_marker('agents', base, Marker.SPHERE,
                x=rx, y=ry, z=rr, sx=rr*2, sy=rr*2, sz=rr*2, color=robot_color))

            # ---- Collision radius ring ----
            if not reached:
                cr = self.make_marker('collision', base, Marker.LINE_STRIP,
                    z=0.02, sx=0.04, color=rgba(c[0], c[1], c[2], 0.6))
                for k in range(25):
                    ang = 2*math.pi*k/24
                    p = Point()
                    p.x = float(rx + rr*math.cos(ang))
                    p.y = float(ry + rr*math.sin(ang))
                    p.z = 0.02
                    cr.points.append(p)
                markers.markers.append(cr)
            else:
                d3 = self.make_marker('collision', base, Marker.LINE_STRIP); d3.action = Marker.DELETE
                markers.markers.append(d3)

            # Suppress per-frame floating agent text in paper-capture mode.

            if name in self.pose_pubs:
                pose_msg = PoseStamped()
                pose_msg.header.frame_id = 'map'
                pose_msg.header.stamp = self.current_stamp
                pose_msg.pose.position.x = float(rx)
                pose_msg.pose.position.y = float(ry)
                pose_msg.pose.orientation.z = math.sin(yaw / 2.0)
                pose_msg.pose.orientation.w = math.cos(yaw / 2.0)
                self.pose_pubs[name].publish(pose_msg)
                self._publish_scan(name, rx, ry, yaw, sensor_range, obs_list)

            # ---- Arrow ----
            arrow_drawn = False
            if not reached and path_pts and path_idx < len(path_pts):
                target = path_pts[min(path_idx, len(path_pts)-1)]
                dx, dy = target[0]-rx, target[1]-ry
                dist = math.sqrt(dx*dx + dy*dy)
                if dist > 0.01:
                    arrow = self.make_marker('agents', base+2, Marker.ARROW,
                        z=rr, sx=0.10, sy=0.22, sz=0.22, color=color)
                    p1 = Point(); p1.x,p1.y,p1.z = float(rx),float(ry),float(rr)
                    p2 = Point(); p2.x,p2.y,p2.z = float(rx+dx/dist*1.5),float(ry+dy/dist*1.5),float(rr)
                    arrow.points = [p1, p2]
                    markers.markers.append(arrow)
                    arrow_drawn = True
            if not arrow_drawn:
                da = self.make_marker('agents', base+2, Marker.ARROW)
                da.action = Marker.DELETE
                markers.markers.append(da)

            # ---- Path ----
            if path_pts and not reached:
                pm = self.make_marker('paths', base, Marker.LINE_STRIP,
                    z=Z_PATH, sx=0.20, color=color)
                pm.colors = []
                for j, pt in enumerate(path_pts):
                    p = Point(); p.x,p.y,p.z = float(pt[0]),float(pt[1]),Z_PATH
                    pm.points.append(p)
                    pm.colors.append(color_faint if j < path_idx else color)
                markers.markers.append(pm)
                if name in self.path_pubs:
                    path_msg = Path()
                    path_msg.header.frame_id = 'map'
                    path_msg.header.stamp = self.current_stamp
                    for pt in path_pts:
                        ps = PoseStamped(); ps.header = path_msg.header
                        ps.pose.position.x = float(pt[0])
                        ps.pose.position.y = float(pt[1])
                        ps.pose.orientation.w = 1.0
                        path_msg.poses.append(ps)
                    self.path_pubs[name].publish(path_msg)
                    if name in self.path_display_pubs:
                        self.path_display_pubs[name].publish(path_msg)
            else:
                dm = self.make_marker('paths', base, Marker.LINE_STRIP); dm.action = Marker.DELETE
                markers.markers.append(dm)

        # ---- People ----
        for i, obs in enumerate(obs_list):
            ox, oy = obs['x'], obs['y']
            orad = obs['radius']
            ohz_rad = obs.get('ohz_radius', orad*1.5)
            markers.markers.append(self.make_marker('obstacles', i, Marker.SPHERE,
                x=ox, y=oy, z=orad, sx=orad*2, sy=orad*2, sz=orad*2, color=COLOR_OBSTACLE))
            markers.markers.append(self.make_marker('ohz', i, Marker.CYLINDER,
                x=ox, y=oy, z=0.01, sx=ohz_rad*2, sy=ohz_rad*2, sz=0.005, color=COLOR_OHZ))

        # ---- Tree ----
        tree_data = data.get('tree')
        if tree_data is not None and tree_data:
            tm = self.make_marker('tree', 0, Marker.LINE_LIST, z=Z_TREE, sx=0.02, color=COLOR_TREE_MAIN)
            tm.colors = []
            for edge in tree_data:
                px,py = edge[0],edge[1]; cx,cy = edge[2],edge[3]
                tid = int(edge[4]) if len(edge) > 4 else 0
                aidx = int(edge[5]) if len(edge) > 5 else 0
                p1 = Point(); p1.x,p1.y,p1.z = float(px),float(py),Z_TREE
                p2 = Point(); p2.x,p2.y,p2.z = float(cx),float(cy),Z_TREE
                tm.points.extend([p1, p2])
                if aidx < len(agents):
                    ac = agents[aidx].get('color', [1,0.6,0])
                    ec = rgba(ac[0], ac[1], ac[2], 0.12 if tid != 0 else 0.25)
                else:
                    ec = COLOR_TREE_DISJOINT if tid != 0 else COLOR_TREE_MAIN
                tm.colors.extend([ec, ec])
            markers.markers.append(tm)

        # Suppress the floating status HUD in paper-capture mode.

        self.marker_pub.publish(markers)
        self.frame_count += 1
        self.current_stamp = None

    def process_end(self, data):
        self.get_logger().info(
            f"Done: {data.get('scenario','?')} | {data.get('agents_reached',0)}/{data.get('agents_total',0)} reached | "
            f"{data.get('total_frames','?')} frames, {data.get('total_time',0):.1f}s")


def main():
    rclpy.init()
    node = MultiAgentBridge()
    try:
        for line in sys.stdin:
            line = line.strip()
            if not line: continue
            try: data = json.loads(line)
            except json.JSONDecodeError: continue
            t = data.get('type', 'frame')
            if t == 'metadata': node.process_metadata(data)
            elif t == 'frame': node.process_frame(data)
            elif t == 'end': node.process_end(data)
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt: pass
    finally:
        node.get_logger().info(f"Total frames: {node.frame_count}")
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
