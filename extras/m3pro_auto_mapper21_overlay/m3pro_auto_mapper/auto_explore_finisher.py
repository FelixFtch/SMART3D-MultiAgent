#!/usr/bin/env python3
# coding: utf-8
"""
AutoExploreFinisher v1.6 (fused+interior-frontier)
- 订阅 /map 与 /map_merged，默认优先融合图进行判定与保存
- 新增 finish_frontier_mode:
    * 'strict'  : 全图无前沿（兼容旧逻辑）
    * 'interior': 只在按 enclose_tol_m 闭合后的“室内区域”找前沿，忽略外圈未知（默认）
    * 'ignore'  : 完全不看前沿
- 覆盖率/面积/围合 + 稳定性迟滞 + 覆盖率停滞兜底
- /auto_explore/force_finish 手动一键保存
- 保存：pbstream（Cartographer） + PGM/YAML（融合图优先，可选另存原图 *_raw）
- 保存后连续发布零速 + 通知 mapper /auto_explore/stop_now + 可选进程退出
"""

import os, time, json, math
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.time import Time as RosTime

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

import tf2_ros

try:
    from cartographer_ros_msgs.srv import FinishTrajectory, WriteState  # type: ignore
except Exception:
    FinishTrajectory = None
    WriteState = None


@dataclass
class FinishCriteria:
    coverage_goal: float
    area_goal_m2: float
    enclose_goal: float
    enclose_tol_m: float
    stall_sec: float
    min_delta: float
    stable_sec: float


class AutoExploreFinisher(Node):
    def __init__(self):
        super().__init__('auto_explore_finisher')

        # --- Topics / frames ---
        self.base_frame     = self.declare_parameter('base_frame', 'base_link').get_parameter_value().string_value
        self.map_topic_raw  = self.declare_parameter('map_topic', '/map').get_parameter_value().string_value
        self.map_topic_fused= self.declare_parameter('fused_map_topic', '/map_merged').get_parameter_value().string_value
        self.prefer_fused   = self.declare_parameter('prefer_fused_map', True).get_parameter_value().bool_value
        self.also_save_raw  = self.declare_parameter('also_save_raw_map', False).get_parameter_value().bool_value

        self.scan_topic     = self.declare_parameter('scan_topic', '/scan').get_parameter_value().string_value
        self.depth_topic    = self.declare_parameter('depth_topic', '/camera/depth/image_raw').get_parameter_value().string_value

        # --- Save options ---
        self.save_pbstream  = self.declare_parameter('finish_save_pbstream', True).get_parameter_value().bool_value
        self.pbstream_path  = self.declare_parameter('pbstream_path', '/home/jetson/yahboom_map.pbstream').get_parameter_value().string_value
        self.save_pgm       = self.declare_parameter('finish_save_pgm', True).get_parameter_value().bool_value
        self.pgm_prefix     = self.declare_parameter('pgm_map_path', '/home/jetson/yahboom_map').get_parameter_value().string_value
        self.exit_after_save= self.declare_parameter('exit_after_save', False).get_parameter_value().bool_value

        # --- Finish criteria ---
        self.criteria = FinishCriteria(
            coverage_goal = self.declare_parameter('coverage_goal', 0.80).get_parameter_value().double_value,
            area_goal_m2  = self.declare_parameter('area_goal_m2', 20.0).get_parameter_value().double_value,
            enclose_goal  = self.declare_parameter('enclose_goal', 0.50).get_parameter_value().double_value,
            enclose_tol_m = self.declare_parameter('enclose_tol_m', 0.35).get_parameter_value().double_value,
            stall_sec     = self.declare_parameter('finish_stall_sec', 20.0).get_parameter_value().double_value,
            min_delta     = self.declare_parameter('finish_min_delta', 0.003).get_parameter_value().double_value,
            stable_sec    = self.declare_parameter('finish_stable_sec', 2.0).get_parameter_value().double_value,
        )

        # --- Frontier gate mode ---
        self.frontier_mode = self.declare_parameter('finish_frontier_mode', 'interior').get_parameter_value().string_value
        if self.frontier_mode not in ('strict','interior','ignore'):
            self.frontier_mode = 'interior'

        # --- Depth emergency stop (kept for safety) & thresholds for PGM ---
        self.depth_stop_m  = self.declare_parameter('depth_stop_m', 0.33).get_parameter_value().double_value
        self.depth_roi_px  = int(self.declare_parameter('depth_roi_px', 12).get_parameter_value().integer_value)
        self.depth_scale_m_per_unit = self.declare_parameter('depth_scale_m_per_unit', 0.0).get_parameter_value().double_value

        self.free_thresh   = int(self.declare_parameter('free_thresh', 25).get_parameter_value().integer_value)
        self.occ_thresh    = int(self.declare_parameter('occ_thresh', 65).get_parameter_value().integer_value)

        # --- ROS I/O ---
        self.sub_map_raw   = self.create_subscription(OccupancyGrid, self.map_topic_raw,  self._on_map_raw,   qos_profile_sensor_data)
        self.sub_map_fused = self.create_subscription(OccupancyGrid, self.map_topic_fused,self._on_map_fused, qos_profile_sensor_data)
        self.sub_scan      = self.create_subscription(LaserScan,     self.scan_topic,     self._on_scan,      qos_profile_sensor_data)
        self.sub_depth     = self.create_subscription(Image,         self.depth_topic,    self._on_depth,     qos_profile_sensor_data)
        self.sub_force     = self.create_subscription(Bool,         '/auto_explore/force_finish', self._on_force, 1)

        self.pub_cmd       = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status    = self.create_publisher(String, '/auto_explore/finish_status', 1)

        self.tf_buffer     = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        self.tf_listener   = tf2_ros.TransformListener(self.tf_buffer, self)

        self.finish_cli    = self.create_client(FinishTrajectory, '/finish_trajectory') if FinishTrajectory else None
        self.write_cli     = self.create_client(WriteState,       '/write_state')       if WriteState       else None

        # --- runtime ---
        self._map_raw  : Optional[OccupancyGrid] = None
        self._map_fused: Optional[OccupancyGrid] = None
        self._saved = False
        self._depth_block = False
        self._lost_pose_since: Optional[float] = None

        self._last_cov = 0.0
        self._last_cov_t = time.time()
        self._pass_since: Optional[float] = None
        self._force_called = False

        self.timer = self.create_timer(1.0, self._tick)
        self.get_logger().info(f'AutoExploreFinisher v1.6 up. frontier_mode={self.frontier_mode}, prefer_fused_map={self.prefer_fused}')

    # ---------- Callbacks ----------
    def _on_map_raw(self, msg: OccupancyGrid):   self._map_raw = msg
    def _on_map_fused(self, msg: OccupancyGrid): self._map_fused = msg
    def _on_scan(self, msg: LaserScan):          pass

    def _on_depth(self, msg: Image):
        try:
            if self._saved: return
            scale = self.depth_scale_m_per_unit or (0.001 if ('16' in (msg.encoding or '')) else 1.0)
            w = msg.width; h = msg.height; roi = self.depth_roi_px
            x0 = max(0, w//2 - roi); x1 = min(w, w//2 + roi)
            if '32F' in (msg.encoding or '').upper():
                arr = np.frombuffer(msg.data, dtype=np.float32).reshape(h,w)[:, x0:x1]
            else:
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(h,w)[:, x0:x1].astype(np.float32)*scale
            valid = arr[(arr > 0.0) & np.isfinite(arr)]
            if valid.size == 0: return
            dmin = float(np.percentile(valid, 10))
            self._depth_block = dmin < self.depth_stop_m
            if self._depth_block:
                self.pub_cmd.publish(Twist())
        except Exception as e:
            self.get_logger().warn(f'depth callback error: {e}')

    def _on_force(self, msg: Bool):
        if msg.data and not self._saved:
            self._force_called = True
            self.get_logger().warn('[force_finish] received -> finishing now.')

    # ---------- Math helpers ----------
    @staticmethod
    def _disk(r:int):
        r = int(max(1, r)); yy,xx = np.ogrid[-r:r+1, -r:r+1]
        return (xx*xx + yy*yy) <= (r*r)

    @staticmethod
    def _dilate(m: np.ndarray, ker: np.ndarray) -> np.ndarray:
        H,W = m.shape; kr = ker.shape[0]//2
        out = np.zeros_like(m, bool)
        for y in range(H):
            y0,y1 = max(0,y-kr), min(H,y+kr+1)
            ky0, ky1 = kr-(y-y0), kr+(y1-y)
            for x in range(W):
                x0,x1 = max(0,x-kr), min(W,x+kr+1)
                kx0, kx1 = kr-(x-x0), kr+(x1-x)
                if np.any(m[y0:y1, x0:x1] & ker[ky0:ky1, kx0:kx1]):
                    out[y,x] = True
        return out

    @staticmethod
    def _erode(m: np.ndarray, ker: np.ndarray) -> np.ndarray:
        return ~AutoExploreFinisher._dilate(~m, ker)

    @staticmethod
    def _flood_from_edges(passable: np.ndarray) -> np.ndarray:
        """返回从边界可达的区域（True=外部可达）。passable=True 表示可通行。"""
        H,W = passable.shape
        vis = np.zeros_like(passable, bool)
        stack = [(0,0),(0,W-1),(H-1,0),(H-1,W-1)]
        while stack:
            y,x = stack.pop()
            if y<0 or y>=H or x<0 or x>=W: continue
            if vis[y,x] or not passable[y,x]: continue
            vis[y,x] = True
            stack.extend([(y-1,x),(y+1,x),(y,x-1),(y,x+1)])
        return vis  # True=外部

    # ---------- Frontier / progress ----------
    def _frontiers(self, grid: np.ndarray, res: float) -> np.ndarray:
        """根据模式返回前沿掩码（True=frontier）。"""
        unknown = (grid < 0)
        free    = (grid >= 0) & (grid <= self.free_thresh)
        if self.frontier_mode == 'ignore':
            return np.zeros_like(grid, dtype=bool)

        # 基础：free 邻接 unknown
        ker1 = self._disk(1)
        if self.frontier_mode == 'strict':
            return free & self._dilate(unknown, ker1)

        # interior：在“按 enclose_tol_m 闭合”的室内区域内判定
        R = max(1, int(round(self.criteria.enclose_tol_m / max(res, 1e-6))))
        closed_free = self._erode(self._dilate(free, self._disk(R)), self._disk(R))
        # 未闭合区域（未知/障碍/自由外）作为 passable，从边界泛洪得到“外部区域”
        # 这里让占据也不可通行，避免通过“黑墙”泄漏
        occ = (grid >= self.occ_thresh)
        outside = self._flood_from_edges(passable=(~closed_free) & (~occ))
        # 仅考虑“闭合内部”的 unknown
        interior_unknown = unknown & (~outside)
        return closed_free & self._dilate(interior_unknown, ker1)

    @staticmethod
    def _measure_progress(map_msg: OccupancyGrid, free_thresh:int, occ_thresh:int):
        data = np.array(map_msg.data, dtype=np.int16)
        w = map_msg.info.width; h = map_msg.info.height
        res = float(map_msg.info.resolution)
        grid = data.reshape(h, w)
        known = grid >= 0
        free  = (grid >= 0) & (grid <= free_thresh)

        coverage = known.sum() / (w * h + 1e-6)
        area_m2  = free.sum() * res * res

        # 简易围合率估计（对 free 闭运算 + 边界泛洪）
        pad = 1
        f = free.astype(np.uint8)
        fpad = np.pad(f, pad, mode='constant', constant_values=0)
        # 3x3 闭运算
        win = []
        for dy in (-1,0,1):
            for dx in (-1,0,1):
                win.append(fpad[pad+dy:pad+dy+h, pad+dx:pad+dx+w])
        dil = np.max(np.stack(win, axis=0), axis=0)
        dpad = np.pad(dil, pad, mode='constant', constant_values=0)
        win2=[]
        for dy in (-1,0,1):
            for dx in (-1,0,1):
                win2.append(dpad[pad+dy:pad+dy+h, pad+dx:pad+dx+w])
        closed = np.min(np.stack(win2, axis=0), axis=0).astype(bool)

        # leak: 从边界对 ~closed 泛洪
        visited = np.zeros_like(closed, dtype=bool)
        stack = [(0,0),(0,w-1),(h-1,0),(h-1,w-1)]
        leak = 0
        while stack:
            y,x = stack.pop()
            if y<0 or y>=h or x<0 or x>=w: continue
            if visited[y,x]: continue
            visited[y,x]=True
            if closed[y,x]: continue
            leak += 1
            stack.extend([(y-1,x),(y+1,x),(y,x-1),(y,x+1)])
        enclose = 1.0 - (leak / (h*w + 1e-6))
        return float(coverage), float(area_m2), float(np.clip(enclose, 0.0, 1.0)), grid, res

    # ---------- Main tick ----------
    def _tick(self):
        try:
            # TF guard
            try:
                _ = self.tf_buffer.lookup_transform('map', self.base_frame, RosTime())
                self._lost_pose_since = None
            except Exception:
                if self._lost_pose_since is None:
                    self._lost_pose_since = time.time()
                if time.time() - self._lost_pose_since > 2.0:
                    self._e_stop('Lost TF(map->base_link)')
                    return

            if self._depth_block:
                self.get_logger().warn('Depth stop engaged (< depth_stop_m).')
                self._depth_block = False

            if self._saved:
                return

            # 选图：融合优先
            chosen = self._map_fused if (self.prefer_fused and self._map_fused is not None) else self._map_raw
            if chosen is None:
                return

            cov, area, encl, grid, res = self._measure_progress(chosen, self.free_thresh, self.occ_thresh)
            fr_mask = self._frontiers(grid, res)
            no_frontiers = (np.count_nonzero(fr_mask) == 0)

            t = time.time()
            dt = max(1e-3, t - self._last_cov_t)
            dcov = cov - self._last_cov
            slope = dcov / dt

            status = {
                'coverage': round(cov,3),
                'area_m2': round(area,2),
                'enclose': round(encl,3),
                'dC_dt': round(slope, 4),
                'no_frontiers': bool(no_frontiers),
                'frontier_mode': self.frontier_mode,
                'map_used': ('fused' if (self.prefer_fused and self._map_fused is not None) else 'raw')
            }
            msg = String(); msg.data = json.dumps(status, ensure_ascii=False)
            self.pub_status.publish(msg)
            self.get_logger().info(f"[finish] cov={cov:.3f} area={area:.2f}m² encl={encl:.3f} dC/dt={slope:.4f} "
                                   f"noF={no_frontiers} mode={self.frontier_mode}")

            # === 触发判定 ===
            frontier_ok = (self.frontier_mode == 'ignore') or no_frontiers

            # 1) 主条件：覆盖/面积/围合 达标 + frontier_ok
            passed = False; reason = ''
            if frontier_ok:
                if cov >= self.criteria.coverage_goal:
                    passed, reason = True, f'coverage {cov:.2f} ≥ {self.criteria.coverage_goal:.2f}'
                elif area >= self.criteria.area_goal_m2:
                    passed, reason = True, f'area {area:.1f} ≥ {self.criteria.area_goal_m2:.1f} m²'
                elif encl >= self.criteria.enclose_goal and area >= 0.3*self.criteria.area_goal_m2:
                    passed, reason = True, f'enclose {encl:.2f} & area {area:.1f} m²'
            else:
                # 若不满足 frontier_ok，但 enclosure 很高，允许“稳定迟滞后放行”
                if (encl >= self.criteria.enclose_goal and cov >= 0.6*self.criteria.coverage_goal):
                    passed, reason = True, f'enclose-only (mode={self.frontier_mode})'

            # 2) 稳定性迟滞（避免瞬时抖动）
            if passed:
                if self._pass_since is None:
                    self._pass_since = t
                if (t - self._pass_since) >= self.criteria.stable_sec:
                    self.get_logger().info(f'Finish criteria met (stable): {reason}')
                    self._do_finish_and_save(chosen, self._map_raw)
                    return
            else:
                self._pass_since = None

            # 3) 覆盖停滞兜底（需要 frontier_ok 或者 enclosure 很高）
            if (t - self._last_cov_t) >= self.criteria.stall_sec:
                delta = cov - self._last_cov
                self._last_cov_t = t; self._last_cov = cov
                if (frontier_ok or encl >= self.criteria.enclose_goal) and cov >= 0.5*self.criteria.coverage_goal and delta <= self.criteria.min_delta:
                    self.get_logger().info('Finish stall guard fired.')
                    self._do_finish_and_save(chosen, self._map_raw)
                    return

            # 4) 手动强制
            if self._force_called:
                self._do_finish_and_save(chosen, self._map_raw)
                return

        except Exception as e:
            self.get_logger().error(f'tick error: {e}')

    # ---------- Save ----------
    def _do_finish_and_save(self, chosen_map: OccupancyGrid, raw_map: Optional[OccupancyGrid]):
        if self._saved: return
        self._saved = True
        self._e_stop('Finishing & saving')

        # 1) Cartographer pbstream
        if self.save_pbstream and self.write_cli:
            try:
                if self.finish_cli and self.finish_cli.wait_for_service(timeout_sec=2.0):
                    req = FinishTrajectory.Request(); req.trajectory_id = 0
                    fut = self.finish_cli.call_async(req)
                    rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
                    time.sleep(0.8)  # 给 Cartographer 收尾一点时间
                if self.write_cli.wait_for_service(timeout_sec=2.0):
                    os.makedirs(os.path.dirname(self.pbstream_path), exist_ok=True)
                    req = WriteState.Request()
                    req.filename = self.pbstream_path
                    req.include_unfinished_submaps = False
                    fut = self.write_cli.call_async(req)
                    rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
                    self.get_logger().info(f'pbstream saved to: {self.pbstream_path}')
            except Exception as e:
                self.get_logger().warn(f'pbstream save failed: {e}')

        # 2) 保存融合/原始 PGM
        if self.save_pgm:
            try:
                self._save_pgm_yaml(chosen_map, self.pgm_prefix)
                self.get_logger().info(f'FUSED map saved to: {self.pgm_prefix}.pgm / .yaml')
                if self.also_save_raw and raw_map is not None:
                    self._save_pgm_yaml(raw_map, self.pgm_prefix + '_raw')
                    self.get_logger().info(f'RAW map saved to: {self.pgm_prefix}_raw.pgm / .yaml')
            except Exception as e:
                self.get_logger().warn(f'save PGM/YAML failed: {e}')

        # 3) 刹车 & 通知 mapper
        for _ in range(40):  # 20Hz * 2s
            self.pub_cmd.publish(Twist())
            time.sleep(0.05)

        stop_pub = self.create_publisher(Bool, '/auto_explore/stop_now', 1)
        stop_pub.publish(Bool(data=True))
        time.sleep(0.1)

        self.get_logger().info('All saving steps done. Finisher exiting gracefully.')
        if self.exit_after_save:
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def _save_pgm_yaml(self, map_msg: OccupancyGrid, prefix: str):
        w = map_msg.info.width; h = map_msg.info.height
        res = map_msg.info.resolution
        origin = map_msg.info.origin
        data = np.array(map_msg.data, dtype=np.int16).reshape(h, w)
        img = np.full((h, w), 205, dtype=np.uint8)  # unknown=205
        img[data >= self.occ_thresh] = 0            # occupied=0
        img[(data >= 0) & (data <= self.free_thresh)] = 254  # free=254

        pgm_path = f'{prefix}.pgm'; yaml_path = f'{prefix}.yaml'
        with open(pgm_path, 'wb') as f:
            header = f'P5\n# CREATOR: auto_explore_finisher\n{w} {h}\n255\n'
            f.write(header.encode('ascii'))
            f.write(np.flipud(img).tobytes())
        yaml = (
            f'image: {os.path.basename(pgm_path)}\n'
            f'resolution: {res:.6f}\n'
            f'origin: [{origin.position.x:.6f}, {origin.position.y:.6f}, 0.0]\n'
            f'negate: 0\n'
            f'occupied_thresh: 0.65\n'
            f'free_thresh: 0.25\n'
        )
        with open(yaml_path, 'w', encoding='utf-8') as f:
            f.write(yaml)

    def _e_stop(self, reason: str):
        self.get_logger().warn(f'E-STOP: {reason}')
        self.pub_cmd.publish(Twist())


def main():
    rclpy.init()
    node = AutoExploreFinisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
