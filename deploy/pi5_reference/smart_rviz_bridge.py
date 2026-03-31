#!/usr/bin/env python3
"""
SMART-3D → RViz2 Bridge v3 — LiDAR Discovery Visualization

Shows:
  - Sensor range circle around robot (cyan ring)
  - Walls: dark when discovered, dim/transparent when not yet seen
  - Detected obstacles (within sensor): solid colored spheres + OHZ
  - Undetected obstacles: faint ghost spheres (visible to viewer, not to robot)
  - Path with ahead/behind coloring
  - Replanning flash effect
  - Scenario title + status HUD

Usage:
  cargo run --release --example smart_rviz_json -- --scenario building --include-tree \
    | python3 smart_rviz_bridge.py
"""

import sys
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import ColorRGBA
import math


def rgba(r, g, b, a=1.0):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


# ---- Color scheme ----
COLOR_ROBOT        = rgba(0.2, 0.6, 1.0)
COLOR_GOAL         = rgba(1.0, 0.2, 0.2)
COLOR_START        = rgba(0.2, 1.0, 0.3)
COLOR_OBSTACLE     = rgba(0.9, 0.3, 0.3, 0.85)  # detected obstacles: red-ish
COLOR_OBS_GHOST    = rgba(0.5, 0.5, 0.5, 0.15)   # undetected: faint ghost
COLOR_OHZ          = rgba(1.0, 0.8, 0.0, 0.12)
COLOR_LRZ          = rgba(0.2, 0.6, 1.0, 0.06)
COLOR_SENSOR       = rgba(0.0, 1.0, 1.0, 0.12)   # sensor range fill
COLOR_SENSOR_RING  = rgba(0.0, 1.0, 1.0, 0.5)    # sensor range edge
COLOR_PATH_AHEAD   = rgba(0.0, 1.0, 0.5)
COLOR_PATH_BEHIND  = rgba(0.3, 0.3, 0.3, 0.4)
COLOR_TREE_MAIN    = rgba(0.5, 0.5, 0.5, 0.15)
COLOR_TREE_DISJOINT = rgba(1.0, 0.5, 0.0, 0.2)
COLOR_REPLANNED    = rgba(1.0, 1.0, 0.0)
COLOR_WALL_SEEN    = rgba(0.2, 0.2, 0.25, 0.95)   # discovered wall
COLOR_WALL_UNSEEN  = rgba(0.4, 0.4, 0.45, 0.25)   # undiscovered wall (faint)
COLOR_GROUND       = rgba(0.15, 0.15, 0.18, 0.6)  # dark fog background
COLOR_BORDER       = rgba(0.3, 0.3, 0.3, 0.4)

# Z layers
Z_GROUND   = -0.01
Z_TREE     = 0.01
Z_OHZ      = 0.02
Z_SENSOR   = 0.025
Z_LRZ      = 0.03
Z_OBSTACLE = 0.05
Z_PATH     = 0.10
Z_ROBOT    = 0.15
Z_GOAL     = 0.15
Z_TEXT     = 0.50
WALL_H     = 1.2  # wall 3D height


class SmartRvizBridge(Node):
    def __init__(self):
        super().__init__('smart_rviz_bridge')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.static_pub = self.create_publisher(MarkerArray, '/smart/static', latched)
        self.marker_pub = self.create_publisher(MarkerArray, '/smart/markers', 10)
        self.path_pub   = self.create_publisher(Path, '/smart/path', 10)
        self.grid_pub   = self.create_publisher(OccupancyGrid, '/smart/map', latched)

        self.metadata = None
        self.frame_count = 0
        self.walls_data = []
        self.grid_res = 0.5
        self.grid_w = 64
        self.grid_h = 64

        self.get_logger().info('SMART RViz2 Bridge v3 (LiDAR + FogOfWar) started')

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
        return m

    def process_metadata(self, data):
        self.metadata = data
        self.walls_data = data.get('walls', [])
        self.grid_res = data.get('grid_res', 0.5)
        self.grid_w = data.get('grid_w', 64)
        self.grid_h = data.get('grid_h', 64)
        scenario = data.get('scenario', '?')

        self.get_logger().info(
            f"Scenario: {scenario} | World: {data['world_size']}m | "
            f"Walls: {len(self.walls_data)} | People: {data.get('num_obstacles', 0)} | "
            f"Sensor: {data.get('sensor_range', '?')}m | Grid: {self.grid_w}x{self.grid_h}"
        )

        # Clear old static markers
        clear = MarkerArray()
        for ns in ['static', 'walls']:
            c = Marker()
            c.header.frame_id = 'map'
            c.header.stamp = self.get_clock().now().to_msg()
            c.ns = ns
            c.action = Marker.DELETEALL
            clear.markers.append(c)
        self.static_pub.publish(clear)

        import time; time.sleep(0.2)

        # Build static markers
        markers = MarkerArray()
        ws = data['world_size']

        # Ground
        markers.markers.append(self.make_marker('static', 0, Marker.CUBE,
            x=ws/2, y=ws/2, z=Z_GROUND, sx=ws, sy=ws, sz=0.01, color=COLOR_GROUND))

        # Border
        border = self.make_marker('static', 1, Marker.LINE_STRIP,
            z=0.0, sx=0.06, color=COLOR_BORDER)
        for cx, cy in [(0,0),(ws,0),(ws,ws),(0,ws),(0,0)]:
            p = Point(); p.x, p.y, p.z = float(cx), float(cy), 0.0
            border.points.append(p)
        markers.markers.append(border)

        # Start
        stx, sty = data['start']
        markers.markers.append(self.make_marker('static', 2, Marker.CYLINDER,
            x=stx, y=sty, z=Z_GOAL/2, sx=1.0, sy=1.0, sz=Z_GOAL, color=COLOR_START))
        st = self.make_marker('static', 3, Marker.TEXT_VIEW_FACING,
            x=stx, y=sty - 1.2, z=Z_TEXT, sx=0.7, sy=0.7, sz=0.7, color=COLOR_START)
        st.text = "START"
        markers.markers.append(st)

        # Goal
        gx, gy = data['goal']
        markers.markers.append(self.make_marker('static', 4, Marker.CYLINDER,
            x=gx, y=gy, z=Z_GOAL/2, sx=1.2, sy=1.2, sz=Z_GOAL, color=COLOR_GOAL))
        gt = self.make_marker('static', 5, Marker.TEXT_VIEW_FACING,
            x=gx, y=gy - 1.2, z=Z_TEXT, sx=0.7, sy=0.7, sz=0.7, color=COLOR_GOAL)
        gt.text = "GOAL"
        markers.markers.append(gt)

        # Scenario title
        title = self.make_marker('static', 6, Marker.TEXT_VIEW_FACING,
            x=ws/2, y=ws + 2.5, z=Z_TEXT, sx=1.0, sy=1.0, sz=1.0, color=rgba(1, 1, 0.3))
        title.text = f"SMART-3D: {scenario}"
        markers.markers.append(title)

        # Walls — initially all dim (undiscovered)
        for i, w in enumerate(self.walls_data):
            x1, y1, x2, y2 = w[0], w[1], w[2], w[3]
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            sx = max(abs(x2 - x1), 0.05)
            sy = max(abs(y2 - y1), 0.05)
            wm = self.make_marker('walls', i, Marker.CUBE,
                x=cx, y=cy, z=WALL_H/2, sx=sx, sy=sy, sz=WALL_H, color=COLOR_WALL_UNSEEN)
            markers.markers.append(wm)

        self.static_pub.publish(markers)
        self.get_logger().info(f"Published static: {len(self.walls_data)} walls")

        # Publish initial scanned grid
        grid_str = data.get('grid', '')
        if grid_str:
            self.publish_grid(grid_str)

    def publish_grid(self, grid_str):
        """Publish OccupancyGrid from compact string (U=unknown, F=free, W=wall)"""
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

        data = []
        for ch in grid_str:
            if ch == 'F':
                data.append(0)    # free → white
            elif ch == 'W':
                data.append(100)  # wall → black
            else:
                data.append(-1)   # unknown → gray
        og.data = data
        self.grid_pub.publish(og)

    def process_frame(self, data):
        if self.metadata is None:
            return

        # Publish fog-of-war grid when available
        grid_str = data.get('grid')
        if grid_str and isinstance(grid_str, str):
            self.publish_grid(grid_str)

        markers = MarkerArray()
        rx, ry = data['robot']
        sensor_range = data.get('sensor_range', 10.0)

        # ---- Robot ----
        replanned = data.get('replanned', False)
        result_str = data.get('result', '')
        is_failed = 'Failed' in result_str
        if replanned:
            robot_color = COLOR_REPLANNED
        elif is_failed:
            robot_color = rgba(1.0, 0.5, 0.0)  # orange = replan failed this frame
        else:
            robot_color = COLOR_ROBOT
        markers.markers.append(self.make_marker('robot', 0, Marker.SPHERE,
            x=rx, y=ry, z=Z_ROBOT, sx=0.9, sy=0.9, sz=0.9, color=robot_color))

        # Direction arrow
        path_pts = data.get('path', [])
        path_idx = data.get('path_index', 0)
        if path_pts and path_idx < len(path_pts):
            target = path_pts[min(path_idx, len(path_pts) - 1)]
            dx, dy = target[0] - rx, target[1] - ry
            dist = math.sqrt(dx*dx + dy*dy)
            if dist > 0.01:
                arrow = self.make_marker('robot', 1, Marker.ARROW,
                    z=Z_ROBOT, sx=0.12, sy=0.25, sz=0.25, color=COLOR_ROBOT)
                p1 = Point(); p1.x, p1.y, p1.z = float(rx), float(ry), Z_ROBOT
                p2 = Point()
                p2.x = float(rx + dx/dist * 1.3)
                p2.y = float(ry + dy/dist * 1.3)
                p2.z = Z_ROBOT
                arrow.points = [p1, p2]
                markers.markers.append(arrow)

        # ---- Sensor range ring (cyan circle) ----
        ring = self.make_marker('sensor', 0, Marker.LINE_STRIP,
            z=Z_SENSOR, sx=0.06, color=COLOR_SENSOR_RING)
        n_pts = 48
        for i in range(n_pts + 1):
            a = 2 * math.pi * i / n_pts
            p = Point()
            p.x = float(rx + sensor_range * math.cos(a))
            p.y = float(ry + sensor_range * math.sin(a))
            p.z = Z_SENSOR
            ring.points.append(p)
        markers.markers.append(ring)

        # Sensor range fill (flat cylinder)
        markers.markers.append(self.make_marker('sensor', 1, Marker.CYLINDER,
            x=rx, y=ry, z=Z_SENSOR/2,
            sx=sensor_range*2, sy=sensor_range*2, sz=0.005, color=COLOR_SENSOR))

        # ---- LRZ ----
        lrz_r = data.get('lrz_radius', 4.0)
        markers.markers.append(self.make_marker('lrz', 0, Marker.CYLINDER,
            x=rx, y=ry, z=Z_LRZ/2, sx=lrz_r*2, sy=lrz_r*2, sz=0.005, color=COLOR_LRZ))

        # ---- Obstacles: detected (solid) + ghost (faint) ----
        all_obs = data.get('all_obstacles', [])
        for i, obs in enumerate(all_obs):
            ox, oy = obs['x'], obs['y']
            orad = obs['radius']
            visible = obs.get('visible', True)

            if visible:
                markers.markers.append(self.make_marker('obstacles', i, Marker.SPHERE,
                    x=ox, y=oy, z=Z_OBSTACLE,
                    sx=orad*2, sy=orad*2, sz=orad*2, color=COLOR_OBSTACLE))
            else:
                markers.markers.append(self.make_marker('obstacles', i, Marker.SPHERE,
                    x=ox, y=oy, z=Z_OBSTACLE,
                    sx=orad*2, sy=orad*2, sz=orad*2, color=COLOR_OBS_GHOST))

        # OHZ for detected obstacles only
        detected_obs = data.get('obstacles', [])
        for i, obs in enumerate(detected_obs):
            ox, oy = obs['x'], obs['y']
            ohz_rad = obs.get('ohz_radius', obs['radius'] * 1.5)
            markers.markers.append(self.make_marker('ohz', i, Marker.CYLINDER,
                x=ox, y=oy, z=Z_OHZ/2,
                sx=ohz_rad*2, sy=ohz_rad*2, sz=0.015, color=COLOR_OHZ))

        # ---- Walls: update discovered vs undiscovered ----
        walls_vis = data.get('walls_visible', [])
        if walls_vis and self.walls_data:
            wall_markers = MarkerArray()
            for i, w in enumerate(self.walls_data):
                if i < len(walls_vis):
                    x1, y1, x2, y2 = w[0], w[1], w[2], w[3]
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    sx = max(abs(x2 - x1), 0.05)
                    sy = max(abs(y2 - y1), 0.05)
                    color = COLOR_WALL_SEEN if walls_vis[i] else COLOR_WALL_UNSEEN
                    wm = self.make_marker('walls', i, Marker.CUBE,
                        x=cx, y=cy, z=WALL_H/2, sx=sx, sy=sy, sz=WALL_H, color=color)
                    wall_markers.markers.append(wm)
            self.static_pub.publish(wall_markers)

        # ---- Path ----
        stamp = self.get_clock().now().to_msg()
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = stamp

        if path_pts:
            pm = self.make_marker('path', 0, Marker.LINE_STRIP,
                z=Z_PATH, sx=0.10, color=COLOR_PATH_AHEAD)
            pm.colors = []
            for j, pt in enumerate(path_pts):
                p = Point(); p.x, p.y, p.z = float(pt[0]), float(pt[1]), Z_PATH
                pm.points.append(p)
                pm.colors.append(COLOR_PATH_BEHIND if j < path_idx else COLOR_PATH_AHEAD)
                ps = PoseStamped()
                ps.header = path_msg.header
                ps.pose.position.x, ps.pose.position.y = float(pt[0]), float(pt[1])
                ps.pose.orientation.w = 1.0
                path_msg.poses.append(ps)
            markers.markers.append(pm)
        self.path_pub.publish(path_msg)

        # ---- Tree ----
        tree_data = data.get('tree')
        if tree_data is not None and tree_data:
            tm = self.make_marker('tree', 0, Marker.LINE_LIST,
                z=Z_TREE, sx=0.02, color=COLOR_TREE_MAIN)
            tm.colors = []
            for edge in tree_data:
                px, py, cx, cy = edge[0], edge[1], edge[2], edge[3]
                tid = int(edge[4]) if len(edge) > 4 else 0
                p1 = Point(); p1.x, p1.y, p1.z = float(px), float(py), Z_TREE
                p2 = Point(); p2.x, p2.y, p2.z = float(cx), float(cy), Z_TREE
                tm.points.extend([p1, p2])
                c = COLOR_TREE_DISJOINT if tid != 0 else COLOR_TREE_MAIN
                tm.colors.extend([c, c])
            markers.markers.append(tm)

        # ---- Status HUD ----
        ws = self.metadata['world_size']
        nv = data.get('num_visible', 0)
        nt = data.get('num_total', 0)
        result_str = data.get('result', '')
        rebuilds = data.get('rebuilds', 0)
        if replanned:
            sc = COLOR_REPLANNED
        elif is_failed:
            sc = rgba(1.0, 0.5, 0.0)
        else:
            sc = rgba(1, 1, 1)

        hud = self.make_marker('status', 0, Marker.TEXT_VIEW_FACING,
            x=ws/2, y=ws + 1.2, z=Z_TEXT, sx=0.55, sy=0.55, sz=0.55, color=sc)
        frame_n = data.get('frame', 0)
        sim_t = data.get('time', 0)
        hud.text = f"t={sim_t:.1f}s | Detected: {nv}/{nt}"
        wk = data.get('walls_known', 0)
        wt = data.get('walls_total', 0)
        if wt > 0:
            hud.text += f" | Walls: {wk}/{wt}"
        if replanned:
            hud.text += " ★REPLAN★"
        if rebuilds > 0:
            hud.text += f" | Rebuilds: {rebuilds}"
        markers.markers.append(hud)

        self.marker_pub.publish(markers)
        self.frame_count += 1

    def process_end(self, data):
        sc = data.get('scenario', '?')
        self.get_logger().info(
            f"Scenario '{sc}' done: {data.get('total_frames', '?')} frames, "
            f"{data.get('total_time', 0):.1f}s")


def main():
    rclpy.init()
    node = SmartRvizBridge()
    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                data = json.loads(line)
            except json.JSONDecodeError as e:
                node.get_logger().warn(f"Bad JSON: {e}")
                continue

            msg_type = data.get('type', 'frame')
            if msg_type == 'metadata':
                node.process_metadata(data)
            elif msg_type == 'frame':
                node.process_frame(data)
            elif msg_type == 'end':
                node.process_end(data)
            elif msg_type == 'pause':
                node.get_logger().info(f"Pause → next: {data.get('next_scenario', '?')}")
            elif msg_type == 'scenario_error':
                node.get_logger().error(f"Error: {data.get('error', '?')}")

            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"Total frames processed: {node.frame_count}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
