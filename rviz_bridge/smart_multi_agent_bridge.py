#!/usr/bin/env python3
"""
SMART-3D Multi-Agent → RViz2 Bridge v2

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
  cargo run --release --example smart_multi_agent -- \
    --scenario mega_airport --num-agents 4 --agent-mode cross \
    -n 150 --playback-speed 3 --sensor-range 12 --rrt-iterations 50000 \
    --lsr-max 80 --include-tree \
    | python3 ~/smart_rviz_bridge/smart_multi_agent_bridge.py
"""

import sys
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import ColorRGBA


def rgba(r, g, b, a=1.0):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


COLOR_WALL_SEEN    = rgba(0.2, 0.2, 0.25, 0.7)
COLOR_WALL_UNSEEN  = rgba(0.4, 0.4, 0.45, 0.20)
COLOR_GROUND       = rgba(0.12, 0.12, 0.15, 0.6)
COLOR_BORDER       = rgba(0.3, 0.3, 0.3, 0.4)
COLOR_OBSTACLE     = rgba(0.95, 0.45, 0.1, 0.85)
COLOR_OHZ          = rgba(1.0, 0.85, 0.0, 0.10)
COLOR_TREE_MAIN    = rgba(1.0, 0.6, 0.0, 0.35)
COLOR_TREE_DISJOINT = rgba(1.0, 0.5, 0.0, 0.12)

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
        super().__init__('smart_multi_agent_bridge')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.static_pub = self.create_publisher(MarkerArray, '/smart/static', latched)
        self.marker_pub = self.create_publisher(MarkerArray, '/smart/markers', 10)
        self.grid_pub   = self.create_publisher(OccupancyGrid, '/smart/map', latched)
        self.path_pubs = {}
        self.metadata = None
        self.frame_count = 0
        self.walls_data = []
        self.grid_res = 0.5
        self.grid_w = 64
        self.grid_h = 64
        self.robot_radius = 0.5
        self.agent_colors = {}
        self.num_agents = 0
        self.get_logger().info('SMART Multi-Agent RViz2 Bridge v2')

    def make_marker(self, ns, mid, mtype, x=0, y=0, z=0, sx=1, sy=1, sz=1, color=None):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
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

        for a in agents_meta:
            name = a['name']
            c = a.get('color', [1, 1, 1])
            self.agent_colors[name] = (c[0], c[1], c[2])
            self.path_pubs[name] = self.create_publisher(Path, f'/smart/path_{name}', 10)

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

        # Per-agent start + goal
        for a in agents_meta:
            name = a['name']
            c = self.agent_colors[name]
            color = rgba(c[0], c[1], c[2])
            color_dim = rgba(c[0], c[1], c[2], 0.4)
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

        # Title
        title = self.make_marker('static', mid, Marker.TEXT_VIEW_FACING,
            x=ws/2, y=ws+3, z=Z_TEXT, sx=1.2, sy=1.2, sz=1.2, color=rgba(1,1,0.3))
        title.text = f"SMART-3D: {scenario} | {self.num_agents} agents ({mode}) | {n_obs} people"
        markers.markers.append(title)
        mid += 1

        # Walls
        for i, w in enumerate(self.walls_data):
            x1, y1, x2, y2 = w[0], w[1], w[2], w[3]
            cx = (x1+x2)/2; cy = (y1+y2)/2
            ssx = max(abs(x2-x1), 0.05); ssy = max(abs(y2-y1), 0.05)
            markers.markers.append(self.make_marker('walls', i, Marker.CUBE,
                x=cx, y=cy, z=WALL_H/2, sx=ssx, sy=ssy, sz=WALL_H, color=COLOR_WALL_UNSEEN))

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

    def process_frame(self, data):
        if self.metadata is None:
            return
        grid_str = data.get('grid')
        if grid_str and isinstance(grid_str, str):
            self.publish_grid(grid_str)

        markers = MarkerArray()
        agents = data.get('agents', [])
        ws = float(self.metadata.get('world_size', 32))
        sensor_range = data.get('sensor_range', self.metadata.get('sensor_range', 10.0))
        rr = data.get('robot_radius', self.robot_radius)

        for ai, a in enumerate(agents):
            name = a['name']
            rx, ry = a['robot']
            path_pts = a.get('path', [])
            path_idx = a.get('path_index', 0)
            reached = a.get('reached_goal', False)
            rebuilds = a.get('rebuilds', 0)
            c = a.get('color', [1,1,1])
            color = rgba(c[0], c[1], c[2])
            color_faint = rgba(c[0], c[1], c[2], 0.25)
            base = ai * 100

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

            # ---- Label ----
            label = self.make_marker('agents', base+1, Marker.TEXT_VIEW_FACING,
                x=rx, y=ry, z=rr*2+0.9, sx=0.5, sy=0.5, sz=0.5, color=robot_color)
            if reached:
                label.text = f"{name} DONE"
            else:
                gx, gy = a['goal']
                d = math.sqrt((rx-gx)**2 + (ry-gy)**2)
                label.text = f"{name} {d:.0f}m R{rebuilds}"
            markers.markers.append(label)

            # ---- Arrow ----
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

            # ---- Path ----
            if path_pts and not reached:
                pm = self.make_marker('paths', base, Marker.LINE_STRIP,
                    z=Z_PATH, sx=0.10, color=color)
                pm.colors = []
                for j, pt in enumerate(path_pts):
                    p = Point(); p.x,p.y,p.z = float(pt[0]),float(pt[1]),Z_PATH
                    pm.points.append(p)
                    pm.colors.append(color_faint if j < path_idx else color)
                markers.markers.append(pm)
                if name in self.path_pubs:
                    path_msg = Path()
                    path_msg.header.frame_id = 'map'
                    path_msg.header.stamp = self.get_clock().now().to_msg()
                    for pt in path_pts:
                        ps = PoseStamped(); ps.header = path_msg.header
                        ps.pose.position.x = float(pt[0])
                        ps.pose.position.y = float(pt[1])
                        ps.pose.orientation.w = 1.0
                        path_msg.poses.append(ps)
                    self.path_pubs[name].publish(path_msg)
            else:
                dm = self.make_marker('paths', base, Marker.LINE_STRIP); dm.action = Marker.DELETE
                markers.markers.append(dm)

        # ---- People ----
        obs_list = data.get('obstacles', [])
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

        # ---- HUD ----
        active = data.get('active_agents', 0)
        wk = data.get('walls_known', 0); wt = data.get('walls_total', 0)
        sim_t = data.get('time', 0)
        parts = []
        for a in agents:
            n = a['name']; r = a.get('rebuilds', 0)
            if a.get('reached_goal', False):
                parts.append(f"{n}:DONE")
            else:
                rx2,ry2 = a['robot']; gx2,gy2 = a['goal']
                d = math.sqrt((rx2-gx2)**2 + (ry2-gy2)**2)
                parts.append(f"{n}:{d:.0f}m R{r}")
        hud = self.make_marker('status', 0, Marker.TEXT_VIEW_FACING,
            x=ws/2, y=ws+1.5, z=Z_TEXT, sx=0.7, sy=0.7, sz=0.7, color=rgba(1,1,1))
        hud.text = (f"t={sim_t:.1f}s | Walls {wk}/{wt} | Active {active}/{len(agents)}"
                    f" | People {len(obs_list)}\n" + "  ".join(parts))
        markers.markers.append(hud)

        self.marker_pub.publish(markers)
        self.frame_count += 1

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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
