#!/usr/bin/env python3
"""
SMART-3D Multi-Agent -> RViz2 bridge

Consumes JSON from smart_multi_agent_3d and renders:
  - discovered / undiscovered 3D walls
  - start / goal markers
  - one sphere + arrow + label per agent
  - one 3D path per agent
  - optional tree edges in 3D
  - dynamic obstacles as orange spheres + OHZ
"""

import json
import math
import sys

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def rgba(r, g, b, a=1.0):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


COLOR_WALL_SEEN = rgba(0.0, 0.0, 0.0, 0.20)
COLOR_WALL_UNSEEN = rgba(0.0, 0.0, 0.0, 0.05)
COLOR_GROUND = rgba(1.0, 1.0, 1.0, 1.0)
COLOR_BORDER = rgba(0.0, 0.0, 0.0, 0.55)
COLOR_OBSTACLE = rgba(1.0, 0.45, 0.10, 0.85)
COLOR_OHZ = rgba(1.0, 0.85, 0.0, 0.10)
COLOR_TREE_DIM = rgba(1.0, 0.55, 0.0, 0.12)
COLOR_TEXT = rgba(1.0, 1.0, 1.0, 0.95)
COLOR_TITLE = rgba(1.0, 1.0, 0.5, 1.0)


class SmartMultiAgent3DBridge(Node):
    def __init__(self):
        super().__init__("smart_multi_agent_3d_bridge")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.static_pub = self.create_publisher(MarkerArray, "/smart/static", latched)
        self.marker_pub = self.create_publisher(MarkerArray, "/smart/markers", 10)
        self.path_pubs = {}
        self.metadata = None
        self.walls = []
        self.frame_count = 0
        self.get_logger().info("SMART 3D multi-agent bridge ready")

    def make_marker(self, ns, mid, mtype, color=None):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        if color is not None:
            m.color = color
        return m

    def safe_publish(self, pub, msg):
        try:
            pub.publish(msg)
            return True
        except RCLError:
            raise KeyboardInterrupt

    def process_metadata(self, data):
        self.metadata = data
        self.walls = data.get("walls", [])
        world = data.get("world_size", [32.0, 32.0, 22.0])
        self.world = [float(v) for v in world]

        for a in data.get("agents", []):
            self.path_pubs[a["name"]] = self.create_publisher(Path, f"/smart/path_{a['name']}", 10)

        clear = MarkerArray()
        wipe = Marker()
        wipe.header.frame_id = "map"
        wipe.header.stamp = self.get_clock().now().to_msg()
        wipe.action = Marker.DELETEALL
        clear.markers.append(wipe)
        self.safe_publish(self.static_pub, clear)
        self.safe_publish(self.marker_pub, clear)

        markers = MarkerArray()
        wx, wy, wz = self.world

        ground = self.make_marker("static", 0, Marker.CUBE, COLOR_GROUND)
        ground.pose.position.x = wx / 2.0
        ground.pose.position.y = wy / 2.0
        ground.pose.position.z = -0.01
        ground.scale.x = wx
        ground.scale.y = wy
        ground.scale.z = 0.01
        markers.markers.append(ground)

        border = self.make_marker("static", 1, Marker.LINE_STRIP, COLOR_BORDER)
        border.scale.x = 0.05
        for cx, cy in [(0, 0), (wx, 0), (wx, wy), (0, wy), (0, 0)]:
            p = Point()
            p.x, p.y, p.z = float(cx), float(cy), 0.0
            border.points.append(p)
        markers.markers.append(border)

        mid = 10
        for a in data.get("agents", []):
            c = a.get("color", [1, 1, 1])
            color = rgba(c[0], c[1], c[2], 0.45)
            start = [float(v) for v in a["start"]]
            goal = [float(v) for v in a["goal"]]

            ms = self.make_marker("starts", mid, Marker.SPHERE, color)
            ms.pose.position.x, ms.pose.position.y, ms.pose.position.z = start
            ms.scale.x = ms.scale.y = ms.scale.z = 0.8
            markers.markers.append(ms)
            mid += 1

            mg = self.make_marker("goals", mid, Marker.CUBE, color)
            mg.pose.position.x, mg.pose.position.y, mg.pose.position.z = goal
            mg.scale.x = mg.scale.y = mg.scale.z = 0.9
            markers.markers.append(mg)
            mid += 1

        # Paper-capture mode suppresses floating title and labels. Start/goal
        # geometry is still shown by the colored sphere/cube markers.

        self.safe_publish(self.static_pub, markers)

    def process_frame(self, data):
        if self.metadata is None:
            return

        markers = MarkerArray()
        walls_visible = data.get("walls_visible", [])

        # Walls
        for i, wall in enumerate(self.walls):
            x1, y1, z1, x2, y2, z2 = [float(v) for v in wall]
            visible = walls_visible[i] if i < len(walls_visible) else True
            color = COLOR_WALL_SEEN if visible else COLOR_WALL_UNSEEN
            m = self.make_marker("walls", i, Marker.CUBE, color)
            m.pose.position.x = (x1 + x2) / 2.0
            m.pose.position.y = (y1 + y2) / 2.0
            m.pose.position.z = (z1 + z2) / 2.0
            m.scale.x = max(abs(x2 - x1), 0.05)
            m.scale.y = max(abs(y2 - y1), 0.05)
            m.scale.z = max(abs(z2 - z1), 0.05)
            markers.markers.append(m)

        for ai, a in enumerate(data.get("agents", [])):
            c = a.get("color", [1, 1, 1])
            color = rgba(c[0], c[1], c[2], 1.0)
            faint = rgba(c[0], c[1], c[2], 0.25)
            rx, ry, rz = [float(v) for v in a["robot"]]
            rr = data.get("robot_radius", 0.5)
            base = ai * 100

            sensor = self.make_marker("sensor", base, Marker.SPHERE, rgba(c[0], c[1], c[2], 0.05))
            sensor.pose.position.x, sensor.pose.position.y, sensor.pose.position.z = rx, ry, rz
            sensor.scale.x = sensor.scale.y = sensor.scale.z = data.get("sensor_range", 10.0) * 2.0
            markers.markers.append(sensor)

            robot = self.make_marker("agents", base + 1, Marker.SPHERE, color if not a.get("reached_goal") else faint)
            robot.pose.position.x, robot.pose.position.y, robot.pose.position.z = rx, ry, rz
            robot.scale.x = robot.scale.y = robot.scale.z = rr * 2.0
            markers.markers.append(robot)

            # Suppress per-frame floating labels in paper-capture mode.

            path_pts = a.get("path", [])
            if path_pts:
                path_marker = self.make_marker("paths", base + 3, Marker.LINE_STRIP, color)
                path_marker.scale.x = 0.08
                path_marker.colors = []
                for j, pt in enumerate(path_pts):
                    pt = [float(v) for v in pt]
                    p = Point()
                    p.x, p.y, p.z = float(pt[0]), float(pt[1]), float(pt[2])
                    path_marker.points.append(p)
                    path_marker.colors.append(color if j >= a.get("path_index", 0) else faint)
                markers.markers.append(path_marker)

                path_msg = Path()
                path_msg.header.frame_id = "map"
                path_msg.header.stamp = self.get_clock().now().to_msg()
                for pt in path_pts:
                    pt = [float(v) for v in pt]
                    ps = PoseStamped()
                    ps.header = path_msg.header
                    ps.pose.position.x = float(pt[0])
                    ps.pose.position.y = float(pt[1])
                    ps.pose.position.z = float(pt[2])
                    ps.pose.orientation.w = 1.0
                    path_msg.poses.append(ps)
                pub = self.path_pubs.get(a["name"])
                if pub is not None:
                    self.safe_publish(pub, path_msg)

                if len(path_pts) > 1:
                    nxt = [float(v) for v in path_pts[min(max(a.get("path_index", 0), 1), len(path_pts) - 1)]]
                    arrow = self.make_marker("arrows", base + 4, Marker.ARROW, color)
                    arrow.scale.x = 0.08
                    arrow.scale.y = 0.18
                    arrow.scale.z = 0.18
                    p1 = Point()
                    p1.x, p1.y, p1.z = rx, ry, rz
                    p2 = Point()
                    p2.x, p2.y, p2.z = nxt[0], nxt[1], nxt[2]
                    arrow.points = [p1, p2]
                    markers.markers.append(arrow)

        for oi, obs in enumerate(data.get("obstacles", [])):
            ox, oy, oz = float(obs["x"]), float(obs["y"]), float(obs["z"])
            orad = float(obs["radius"])
            ohz = float(obs.get("ohz_radius", orad * 1.5))
            m = self.make_marker("obstacles", oi, Marker.SPHERE, COLOR_OBSTACLE)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = ox, oy, oz
            m.scale.x = m.scale.y = m.scale.z = orad * 2.0
            markers.markers.append(m)

            h = self.make_marker("ohz", oi, Marker.SPHERE, COLOR_OHZ)
            h.pose.position.x, h.pose.position.y, h.pose.position.z = ox, oy, oz
            h.scale.x = h.scale.y = h.scale.z = ohz * 2.0
            markers.markers.append(h)

        tree = data.get("tree")
        if tree:
            tm = self.make_marker("tree", 0, Marker.LINE_LIST, COLOR_TREE_DIM)
            tm.scale.x = 0.02
            tm.colors = []
            agents = data.get("agents", [])
            for edge in tree:
                px, py, pz, cx, cy, cz = [float(v) for v in edge[:6]]
                aidx = int(edge[7]) if len(edge) > 7 else 0
                ec = COLOR_TREE_DIM
                if aidx < len(agents):
                    c = agents[aidx].get("color", [1, 0.55, 0])
                    ec = rgba(c[0], c[1], c[2], 0.18)
                p1 = Point()
                p1.x, p1.y, p1.z = float(px), float(py), float(pz)
                p2 = Point()
                p2.x, p2.y, p2.z = float(cx), float(cy), float(cz)
                tm.points.extend([p1, p2])
                tm.colors.extend([ec, ec])
            markers.markers.append(tm)

        # Suppress the floating status HUD in paper-capture mode.

        self.safe_publish(self.marker_pub, markers)
        self.frame_count += 1

    def process_end(self, data):
        self.get_logger().info(
            f"Done {data.get('scenario','?')}: {data.get('agents_reached',0)}/{data.get('agents_total',0)} reached"
        )


def main():
    rclpy.init()
    node = SmartMultiAgent3DBridge()
    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                continue
            t = data.get("type", "frame")
            if t == "metadata":
                node.process_metadata(data)
            elif t == "frame":
                node.process_frame(data)
            elif t == "end":
                node.process_end(data)
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.get_logger().info(f"Frames rendered: {node.frame_count}")
        except RCLError:
            pass
        try:
            node.destroy_node()
        except RCLError:
            pass
        try:
            rclpy.shutdown()
        except RCLError:
            pass


if __name__ == "__main__":
    main()
