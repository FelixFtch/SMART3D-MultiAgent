#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AutoExploreMapper v2.10.0-depth-crossbar
- 深度“横档硬墙”：在相机前方测得的距离处，沿左右方向画一条矩形横档（与视线垂直），写入持久硬墙层（occ_hard）
  · 不会被雷达抹掉，仅当“朝向接近且深度非常安全”持续若干帧后由深度清除
- 近距避障：仍触发硬墙 + 短TTL封堵 + 立刻倒车；倒车/转向优先“朝目标”的开阔角（退化为更空一侧）
- 依旧保留：宽通道A*、路径阻塞重规划、角度闸+恢复期短视距、taboo、全局前沿聚类与信息增益、/map_merged 融合输出
"""

import math, time, heapq, json
from typing import Optional, Tuple, List
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan, Image
from visualization_msgs.msg import Marker
from std_msgs.msg import String, Bool

import tf2_ros
from tf_transformations import euler_from_quaternion as _efq

# ---- Optional deps ----
try:
    from sklearn.cluster import DBSCAN
    _HAS_SKLEARN = True
except Exception:
    DBSCAN = None
    _HAS_SKLEARN = False

try:
    import cv2
    _HAS_CV2 = True
except Exception:
    _HAS_CV2 = False

STATE_WAITING, STATE_INITIAL_ROT, STATE_EXPLORING, STATE_SLOW_SCAN, STATE_CORRIDOR, STATE_REVERSING, STATE_ESCAPE_TURN = range(7)

def yaw_from_quat(q) -> float:
    return _efq([q.x, q.y, q.z, q.w])[2]

def tnow() -> float:
    return time.time()

def distance_transform_free(free_mask: np.ndarray) -> np.ndarray:
    """对 free=True 的像素做距离变换（到障碍的距离，像素单位）。优先 OpenCV，退化 BFS。"""
    if _HAS_CV2:
        try:
            src = (free_mask.astype(np.uint8)) * 255
            return cv2.distanceTransform(src, cv2.DIST_L2, 3).astype(np.float32)
        except Exception:
            pass
    # BFS fallback
    H, W = free_mask.shape
    INF = 10**9
    dist = np.full((H, W), INF, np.int32)
    q = deque()
    occ = ~free_mask
    for r in range(H):
        for c in range(W):
            if occ[r, c]:
                dist[r, c] = 0
                q.append((r, c))
    def nbr4(r,c):
        if r>0:   yield r-1,c
        if r<H-1: yield r+1,c
        if c>0:   yield r,c-1
        if c<W-1: yield r,c+1
    while q:
        r,c = q.popleft()
        base = dist[r,c]
        for nr,nc in nbr4(r,c):
            if dist[nr,nc] > base + 1:
                dist[nr,nc] = base + 1
                q.append((nr,nc))
    dist[occ] = 0
    return dist.astype(np.float32)

class AutoExploreMapper(Node):
    def __init__(self):
        super().__init__('auto_explore_mapper')
        gp = self.declare_parameter

        # ---- Topics / frames ----
        self.base_frame   = gp('base_frame',  'base_link').value
        self.map_topic    = gp('map_topic',   '/map').value
        self.scan_topic   = gp('scan_topic',  '/scan').value
        self.depth_topic  = gp('depth_topic', '/camera/depth/image_raw').value

        # ---- Speeds & tracker ----
        self.linear_speed        = float(gp('linear_speed', 0.22).value)
        self.angular_speed       = float(gp('angular_speed',0.45).value)
        self.rotate_speed        = float(gp('rotate_speed', 0.30).value)
        self.lookahead_m         = float(gp('lookahead_m',  0.35).value)
        self.long_lookahead_m    = float(gp('long_lookahead_m', 1.20).value)
        self.replan_interval_sec = float(gp('replan_interval_sec', 1.5).value)
        self.commit_time_sec     = float(gp('commit_time_sec', 10.0).value)

        # ---- Map thresholds ----
        self.free_thresh  = int  (gp('free_thresh', 25).value)
        self.occ_thresh   = int  (gp('occ_thresh',  65).value)

        # ---- Progress ----
        self.min_progress_m     = float(gp('min_progress_m', 0.08).value)
        self.progress_check_sec = float(gp('progress_check_sec', 4.0).value)

        # ---- Geometry / inflate / gap close ----
        self.robot_radius_m   = float(gp('robot_radius_m', 0.11).value)
        self.inflate_margin_m = float(gp('inflate_margin_m', 0.03).value)
        self.gap_fill_enable  = bool (gp('gap_fill_enable', True).value)
        self.gap_fill_max_m   = float(gp('gap_fill_max_m',  0.15).value)

        # ---- Corridor fallback ----
        self.corridor_v           = float(gp('corridor_v', 0.22).value)
        self.corridor_k_ang       = float(gp('corridor_k_ang', 0.9).value)
        self.corridor_min_forward = float(gp('corridor_min_forward', 0.25).value)

        # ---- Decisions / stability ----
        self.branch_bias         = str  (gp('branch_bias', 'right').value)  # left/right/none
        self.branch_bias_weight  = float(gp('branch_bias_weight', 0.45).value)
        self.forward_penalty_k   = float(gp('forward_penalty_k',   1.2).value)
        self.taboo_radius_m      = float(gp('taboo_radius_m', 0.80).value)
        self.taboo_ttl_sec       = float(gp('taboo_ttl_sec', 25.0).value)

        # ---- Depth → HARD WALL (persist until cleared by depth-at-same-yaw) ----
        self.depth_stop_m        = float(gp('depth_stop_m', 0.30).value)
        self.depth_scale         = float(gp('depth_scale_m_per_unit', 0.0).value)
        self.depth_min_valid_m   = float(gp('depth_min_valid_m', 0.10).value)
        self.depth_max_valid_m   = float(gp('depth_max_valid_m', 6.00).value)
        self.depth_roi_px        = int  (gp('depth_roi_px', 12).value)

        # ---- 原“前向矩形硬墙”的几何参数（保留可选） ----
        self.paint_rect_width_m  = float(gp('paint_rect_width_m', 0.28).value)
        self.paint_rect_len_max  = float(gp('paint_rect_len_max', 0.80).value)
        self.hard_connect_enable = bool (gp('hard_connect_enable', True).value)
        self.hard_connect_m      = float(gp('hard_connect_m', 0.25).value)
        self.hard_thin_enable    = bool (gp('hard_thin_enable', True).value)
        self.hard_thin_radius_m  = float(gp('hard_thin_radius_m', 0.18).value)
        self.hard_disk_scale     = float(gp('hard_disk_scale', 0.40).value)

        # ---- 持久硬墙清除策略 ----
        self.depth_clear_m       = float(gp('depth_clear_m', 0.85).value)
        self.depth_clear_frames  = int  (gp('depth_clear_frames', 3).value)
        self.depth_debug_log     = bool (gp('depth_debug_log', True).value)

        # ---- 新：横档硬墙（与视线垂直）参数 ----
        self.depth_hard_enable        = bool (gp('depth_hard_enable', True).value)
        self.depth_hard_trigger_m     = float(gp('depth_hard_trigger_m', 0.75).value)  # 小于该距离开始画硬墙
        self.depth_hard_min_len       = float(gp('depth_hard_min_len', 0.18).value)
        self.depth_clear_yaw_tol_rad  = float(gp('depth_clear_yaw_tol_rad', 0.35).value)  # ~20°
        self.depth_clear_hard_margin_m= float(gp('depth_clear_hard_margin_m', 0.12).value)
        self.depth_hard_patch_limit   = int  (gp('depth_hard_patch_limit', 300).value)

        # ---- 关键：横档尺寸与定位 ----
        self.depth_bar_enable         = bool (gp('depth_bar_enable', True).value)
        self.depth_bar_len_m          = float(gp('depth_bar_len_m', 0.60).value)  # 左右跨越长度（横向）
        self.depth_bar_thick_m        = float(gp('depth_bar_thick_m', 0.12).value) # 沿前后方向厚度
        self.depth_bar_center_offset_m= float(gp('depth_bar_center_offset_m', -0.03).value) # 把横档略微放在“测得距离”之前

        # 新：避障转向优先对准目标
        self.depth_escape_open_m      = float(gp('depth_escape_open_m', 0.55).value)

        # ---- Virtual (TTL) layer ----
        self.virtual_block_ttl   = float(gp('virtual_block_ttl_sec', 2.0).value)
        self.virtual_block_r_m   = float(gp('virtual_block_radius_m', 0.30).value)

        # ---- Planning-only closing ----
        self.plan_close_enable   = bool (gp('plan_close_enable', True).value)
        self.plan_close_gap_m    = float(gp('plan_close_gap_m', 0.06).value)

        # ---- A* cost / clearance ----
        self.width_cost_k        = float(gp('width_cost_k', 0.6).value)
        self.min_clearance_m     = float(gp('min_clearance_m', 0.12).value)
        self.frontier_min_dt_m   = float(gp('frontier_min_dt_m', 0.14).value)

        # ---- Finisher 交由独立节点 ----
        self.self_finish_enable  = bool (gp('self_finish_enable', False).value)

        # ---- Path validation ----
        self.replan_on_block_enable = bool(gp('replan_on_block_enable', True).value)
        self.block_check_ahead_m    = float(gp('block_check_ahead_m', 0.80).value)
        self.block_detect_min_frames= int  (gp('block_detect_min_frames', 1).value)
        self._path_block_frames     = 0

        # ---- Escape ----
        self.reverse_time_sec    = float(gp('reverse_time_sec', 0.6).value)
        self.reverse_time_scale  = float(gp('reverse_time_scale', 0.5).value)
        self.reverse_speed       = float(gp('reverse_speed', self.linear_speed).value)
        self.reverse_turn_speed  = float(gp('reverse_turn_speed', 0.8*self.angular_speed).value)
        self.escape_turn_max_rad = float(gp('escape_turn_max_rad', math.radians(95)).value)
        self.escape_turn_min_rad = float(gp('escape_turn_min_rad', math.radians(25)).value)
        self._escape_target_yaw  = None
        self.escape_goal_override_rad = float(gp('escape_goal_override_rad', 0.7).value)
        self._escape_side_sign = 0  # -1=右, +1=左

        # ---- Map merged ----
        self.publish_map_merged  = bool (gp('publish_map_merged', True).value)

        # ---- Global planning ----
        self.use_global_tour     = bool (gp('use_global_tour', True).value)
        self.cluster_method      = str  (gp('cluster_method', 'dbscan' if _HAS_SKLEARN else 'cc').value)
        self.dbscan_eps_m        = float(gp('dbscan_eps_m', 0.60).value)
        self.dbscan_min_samples  = int  (gp('dbscan_min_samples', 12).value)
        self.cluster_downsample_stride = int(gp('cluster_downsample_stride', 2).value)
        self.max_cluster_count   = int  (gp('max_cluster_count', 12).value)
        self.global_graph_downsample_factor = int(gp('global_graph_downsample_factor', 2).value)
        self.global_plan_time_budget_sec   = float(gp('global_plan_time_budget_sec', 0.7).value)
        self.global_replan_min_interval_sec= float(gp('global_replan_min_interval_sec', 6.0).value)
        self.global_replan_known_delta_ratio= float(gp('global_replan_known_delta_ratio', 0.05).value)
        self.global_tour_strategy = str  (gp('global_tour_strategy', 'ratio').value)
        self.info_gain_enable     = bool (gp('info_gain_enable', True).value)
        self.info_gain_radius_m   = float(gp('info_gain_radius_m', 1.6).value)
        self.info_gain_beta       = float(gp('info_gain_beta', 1.0).value)
        self.arrive_thresh_m      = float(gp('arrive_thresh_m', 0.40).value)
        self.use_cluster_markers  = bool (gp('use_cluster_markers', False).value)

        # ---- Escape/track 强化 ----
        self.angle_gate_rad            = float(gp('angle_gate_rad', 0.30).value)
        self.recovery_small_lookahead_m = float(gp('recovery_small_lookahead_m', 0.20).value)
        self.recovery_guard_sec         = float(gp('recovery_guard_sec', 2.0).value)
        self.recovery_disable_corridor  = bool (gp('recovery_disable_corridor', True).value)
        self.post_recovery_block_r_m    = float(gp('post_recovery_block_r_m', 0.35).value)
        self.post_recovery_block_ttl    = float(gp('post_recovery_block_ttl', 6.0).value)

        # ---- 前沿过滤与无增益 taboo ----
        self.min_unknown_area_m2        = float(gp('min_unknown_area_m2', 1.0).value)
        self.goal_known_delta_min_ratio = float(gp('goal_known_delta_min_ratio', 0.001).value)

        # ---- ROS I/O ----
        qos_map = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.sub_map   = self.create_subscription(OccupancyGrid, self.map_topic,  self.on_map,   qos_map)
        self.sub_scan  = self.create_subscription(LaserScan,     self.scan_topic, self.on_scan,  10)
        self.sub_depth = self.create_subscription(Image,         self.depth_topic,self.on_depth, 10)
        self.pub_cmd   = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_map_merged = self.create_publisher(OccupancyGrid, '/map_merged', 1) if self.publish_map_merged else None

        # ---- Live viz ----
        self.enable_viz = True
        if self.enable_viz:
            self.pub_path   = self.create_publisher(Path,   '/planned_path', 10)
            self.pub_goal   = self.create_publisher(Marker, '/plan_goal',    10)
            self.pub_target = self.create_publisher(Marker, '/track_target', 10)
            self.pub_state  = self.create_publisher(String, '/auto_explore/state', 1)
        else:
            self.pub_state  = self.create_publisher(String, '/auto_explore/state', 1)

        # 保存后停机
        self.sub_stop = self.create_subscription(Bool, '/auto_explore/stop_now', self._on_stop_now, 1)

        self.tf_buffer  = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        self.tf_listener= tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- State / caches ----
        self.state = STATE_WAITING
        self.state_t0 = tnow()

        self.map: Optional[np.ndarray] = None
        self.res = None; self.ox = self.oy = None
        self.kernel_inflate = self.kernel_gap = None
        self.kernel_inflate_r = self.kernel_gap_r = -1

        self.free_mask = self.occ_mask = self.unknown_mask = None
        self.occ_extra_expire: Optional[np.ndarray] = None  # TTL
        self.occ_hard: Optional[np.ndarray] = None          # 持久硬墙（雷达不会覆盖）

        self.last_map_info = None
        self.last_map_header = None

        self.path_cells: List[Tuple[int,int]] = []
        self.goal_cell: Optional[Tuple[int,int]] = None

        self.init_rot_total = 2*math.pi
        self.init_rot_done  = 0.0

        self.last_prog_xy: Optional[Tuple[float,float]] = None
        self.last_prog_t  = tnow()
        self.no_progress_strikes = 0

        self.last_scan: Optional[LaserScan] = None

        self.commit_until = 0.0
        self.taboo_list: List[Tuple[float,float,float]] = []

        self._stall_start_ts = None
        self._stall_thresh_s = 1.0

        self._escape_turn_left = 0.0

        # Global planner runtime
        self.global_plan_tour: List[Tuple[int, int]] = []
        self._last_global_plan_ts = 0.0
        self._known_count_last    = 0
        self._global_plan_seq     = 0

        # NEW runtime
        self.recovery_until = 0.0
        self._stuck_pose: Optional[Tuple[float,float,float]] = None
        self._known_at_goal_start: Optional[int] = None

        # 硬墙 patch 管理（按中心/朝向/尺寸/形状记录；仅深度清空）
        self.hard_patches: List[dict] = []

        # 缓存最近一帧深度“列中位”与视场
        self._last_depth_row_med: Optional[np.ndarray] = None
        self._last_depth_hfov: float = math.radians(70.0)
        self._last_depth_ts: float = 0.0

        self.timer = self.create_timer(0.20, self.on_timer)
        self.get_logger().info("AutoExplore v2.10.0-depth-crossbar | depth HARD wall (crossbar) + aim-to-goal escape + /map_merged")

    # ---------------- Utils ----------------
    def _pub_state(self, ev: str, extra: dict=None):
        d = {'t': round(tnow(),2), 'state': self._state_name(self.state), 'ev': ev}
        if extra: d.update(extra)
        msg = String(); msg.data = json.dumps(d, ensure_ascii=False)
        self.pub_state.publish(msg)
        if extra: self.get_logger().info(f"[{ev}] {extra}")
        else:     self.get_logger().info(f"[{ev}]")

    def _state_name(self, s:int)->str:
        return ['WAIT','INIT_ROT','EXPLORE','SLOW_SPIN','CORRIDOR','REVERSE','ESCAPE_TURN'][s]

    def pose_in_map(self) -> Optional[Tuple[float,float,float]]:
        try:
            tf = self.tf_buffer.lookup_transform('map', self.base_frame, rclpy.time.Time())
            x, y = tf.transform.translation.x, tf.transform.translation.y
            yaw  = yaw_from_quat(tf.transform.rotation)
            return (x, y, yaw)
        except Exception as e:
            self.get_logger().warn(f"[TF] 获取失败：{e}", throttle_duration_sec=2.0)
            return None

    def world_to_cell(self, x: float, y: float) -> Optional[Tuple[int,int]]:
        if self.map is None: return None
        cx = int((x - self.ox) / self.res); cy = int((y - self.oy) / self.res)
        if 0 <= cx < self.map.shape[1] and 0 <= cy < self.map.shape[0]: return (cx, cy)
        return None

    def cell_to_world(self, cx: int, cy: int) -> Tuple[float,float]:
        return (self.ox + (cx + 0.5)*self.res, self.oy + (cy + 0.5)*self.res)

    @staticmethod
    def _angdiff(a,b):
        d=a-b
        while d> math.pi: d-=2*math.pi
        while d<-math.pi: d+=2*math.pi
        return d

    def publish_stop(self): self.pub_cmd.publish(Twist())

    def _known_count(self) -> int:
        return int(np.count_nonzero(~self.unknown_mask)) if self.unknown_mask is not None else 0

    # ---------------- Live viz helpers ----------------
    def _publish_planned_path(self):
        if not self.enable_viz or self.pub_path is None or self.last_map_info is None: return
        p = Path()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'map'
        poses = []
        for (x,y) in self.path_cells:
            wx,wy = self.cell_to_world(x,y)
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x = wx; ps.pose.position.y = wy; ps.pose.position.z = 0.0
            poses.append(ps)
        p.poses = poses
        self.pub_path.publish(p)

    def _publish_marker(self, pub, wx:float, wy:float, color=(1.0,0.6,0.1), scale=0.16, mid=1):
        if not self.enable_viz or pub is None: return
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'viz'; m.id = mid; m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position.x = wx; m.pose.position.y = wy; m.pose.position.z = 0.05
        m.scale.x = scale; m.scale.y = scale; m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = color[0], color[1], color[2], 0.9
        pub.publish(m)

    # ---------------- Subs ----------------
    def on_scan(self, msg: LaserScan):
        self.last_scan = msg

    # ---- Hard patch helpers ----
    def _rect_origin_from_center(self, cx:float, cy:float, yaw:float, length:float)->Tuple[float,float]:
        return (cx - math.cos(yaw)*0.5*length, cy - math.sin(yaw)*0.5*length)

    def _paint_hard_rect_center(self, cx:float, cy:float, yaw:float, length:float, width:float):
        rx, ry = self._rect_origin_from_center(cx, cy, yaw, length)
        self._paint_hard_rect_ahead(rx, ry, yaw, length, width)

    def _clear_hard_rect_center(self, cx:float, cy:float, yaw:float, length:float, width:float):
        rx, ry = self._rect_origin_from_center(cx, cy, yaw, length)
        self._clear_hard_rect_ahead(rx, ry, yaw, length, width)

    def _upsert_hard_patch(self, cx:float, cy:float, yaw:float, length:float, width:float, shape:str='rect'):
        # 合并到邻近 patch，避免爆炸
        for p in self.hard_patches:
            if p.get('shape','rect') != shape:
                continue
            if abs(self._angdiff(p['yaw'], yaw)) < 0.20 and math.hypot(p['cx']-cx, p['cy']-cy) < 0.25:
                p['cx'], p['cy'] = cx, cy
                p['yaw'] = yaw
                p['length'] = max(p['length'], length)
                p['width']  = max(p['width'],  width)
                p['clear_ok'] = 0
                return
        self.hard_patches.append({'cx':cx,'cy':cy,'yaw':yaw,'length':length,'width':width,'clear_ok':0,'shape':shape})
        if len(self.hard_patches) > self.depth_hard_patch_limit:
            self.hard_patches.pop(0)

    def _try_clear_patches_by_depth(self, yaw_now:float, med_depth:float):
        """仅当朝向接近 patch.yaw 且深度非常安全时，连续 N 帧清理该 patch。"""
        if not self.hard_patches: return
        thr = max(self.depth_clear_m, self.depth_stop_m + self.depth_clear_hard_margin_m)
        keep = []
        changed = False
        for p in self.hard_patches:
            if abs(self._angdiff(yaw_now, p['yaw'])) <= self.depth_clear_yaw_tol_rad and med_depth >= thr:
                p['clear_ok'] = p.get('clear_ok', 0) + 1
                if p['clear_ok'] >= self.depth_clear_frames:
                    if p.get('shape','rect') == 'bar':
                        self._clear_hard_crossbar_center(p['cx'], p['cy'], p['yaw'], p['length']*1.1, p['width']*1.1)
                    else:
                        self._clear_hard_rect_center(p['cx'], p['cy'], p['yaw'], p['length']*1.1, p['width']*1.1)
                    changed = True
                    continue  # drop
            else:
                p['clear_ok'] = 0
            keep.append(p)
        if changed:
            self._publish_map_merged()
        self.hard_patches = keep

    # ---- 横档硬墙：在前方 dist_m 处，沿左右方向画条形 ----
    def _paint_hard_crossbar_center(self, cx:float, cy:float, yaw:float, bar_len:float, bar_thick:float):
        if self.map is None:
            return
        if self.occ_hard is None:
            self.occ_hard = np.zeros_like(self.occ_mask, dtype=bool)
        step = max(self.res, 0.05)
        half_l = bar_len * 0.5
        half_t = bar_thick * 0.5
        c, s = math.cos(yaw), math.sin(yaw)
        u = (c, s)      # forward
        v = (-s, c)     # left-orthogonal
        t = -half_t
        while t <= half_t:
            yoff = -half_l
            while yoff <= half_l:
                wx = cx + u[0]*t + v[0]*yoff
                wy = cy + u[1]*t + v[1]*yoff
                cell = self.world_to_cell(wx, wy)
                if cell is not None:
                    self.occ_hard[cell[1], cell[0]] = True
                yoff += step
            t += step
        if self.hard_connect_enable:
            kr = int(max(1, round(self.hard_connect_m / max(self.res, 1e-3))))
            ker = self._make_disk(kr)
            self.occ_hard = self._erode(self._dilate(self.occ_hard, ker), ker)
        if self.hard_thin_enable and self.hard_thin_radius_m > 1e-3:
            tr = int(max(1, round(self.hard_thin_radius_m / max(self.res, 1e-3))))
            tker = self._make_disk(tr)
            self.occ_hard = self._erode(self.occ_hard, tker)
        self.get_logger().info(f"[HardBar] 写入：L≈{bar_len:.2f}m T≈{bar_thick:.2f}m", throttle_duration_sec=1.0)

    def _clear_hard_crossbar_center(self, cx:float, cy:float, yaw:float, bar_len:float, bar_thick:float):
        if self.map is None or self.occ_hard is None:
            return
        step = max(self.res, 0.05)
        half_l = bar_len * 0.5
        half_t = bar_thick * 0.5
        c, s = math.cos(yaw), math.sin(yaw)
        u = (c, s); v = (-s, c)
        t = -half_t
        cleared = 0
        while t <= half_t:
            yoff = -half_l
            while yoff <= half_l:
                wx = cx + u[0]*t + v[0]*yoff
                wy = cy + u[1]*t + v[1]*yoff
                cell = self.world_to_cell(wx, wy)
                if cell is not None:
                    cx_, cy_ = cell
                    if self.occ_hard[cy_, cx_]:
                        self.occ_hard[cy_, cx_] = False
                        cleared += 1
                yoff += step
            t += step
        if cleared > 0:
            self.get_logger().info(f"[HardBar] 清除横档 ~{cleared} cells", throttle_duration_sec=1.0)

    # ---------------- Depth sub ----------------
    def on_depth(self, msg: Image):
        """鲁棒解码 + 横档硬墙：在前方“测得距离”处横向写入（持久），近距仍触发倒车；避障后优先朝目标转向。"""
        enc = (msg.encoding or '').lower()
        H, W = msg.height, msg.width
        try:
            if '32f' in enc:
                depth_m = np.frombuffer(msg.data, dtype=np.float32).reshape(H, W)
            else:
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(H, W).astype(np.float32)
                if self.depth_scale and self.depth_scale > 0.0:
                    depth_m = arr * self.depth_scale
                else:
                    mx = float(np.nanmax(arr)) if arr.size else 0.0
                    depth_m = (arr * 0.001) if 50.0 < mx < 8000.0 else arr
        except Exception as e:
            self.get_logger().warn(f"[Depth] 解码失败：{e}", throttle_duration_sec=2.0)
            return

        k = max(2, int(self.depth_roi_px // 2))
        cy, cx = H//2, W//2
        roi = depth_m[cy-k:cy+k, cx-k:cx+k]
        roi = roi[np.isfinite(roi) & (roi >= self.depth_min_valid_m) & (roi <= self.depth_max_valid_m)]
        if roi.size == 0:
            return

        # 缓存列中位阵列，供“朝目标选角”使用
        try:
            row = depth_m[cy, :]
            row = np.where(np.isfinite(row) & (row>0), row, 0.0)
            win = 9
            row_med = np.array([np.median(row[max(0,i-win):min(W,i+win+1)]) for i in range(W)], dtype=np.float32)
            self._last_depth_row_med = row_med
            self._last_depth_hfov   = math.radians(70.0)
            self._last_depth_ts     = tnow()
        except Exception:
            self._last_depth_row_med = None

        med = float(np.median(roi))

        # 用列中位的“最远列角”作为脱困备选角（若后续无法朝目标）
        depth_suggest_yaw = None
        try:
            if self._last_depth_row_med is not None:
                best_idx = int(np.argmax(self._last_depth_row_med))
                col_ang = ((best_idx - (W/2.0)) / (W/2.0)) * (self._last_depth_hfov/2.0)
                depth_suggest_yaw = float(np.clip(col_ang, -self.escape_turn_max_rad, self.escape_turn_max_rad))
        except Exception:
            pass

        pose = self.pose_in_map()
        if pose is None:
            return
        rx, ry, ryaw = pose

        # ---- A) 在“测得距离”处画横档硬墙（持久，不会被雷达覆盖）----
        if self.depth_hard_enable and self.map is not None and med < self.depth_hard_trigger_m:
            if self.depth_bar_enable:
                dist = max(0.10, med + self.depth_bar_center_offset_m)
                cxw = rx + math.cos(ryaw) * dist
                cyw = ry + math.sin(ryaw) * dist
                bar_len   = float(max(0.20, self.depth_bar_len_m))
                bar_thick = float(max(0.06, self.depth_bar_thick_m))
                self._paint_hard_crossbar_center(cxw, cyw, ryaw, bar_len, bar_thick)
                self._upsert_hard_patch(cxw, cyw, ryaw, bar_len, bar_thick, shape='bar')
                self._publish_map_merged()
                if self.depth_debug_log:
                    self.get_logger().info(f"[DepthHardBar] center=({cxw:.2f},{cyw:.2f}) yaw={math.degrees(ryaw):.1f}° depth={med:.2f}m LxT={bar_len:.2f}x{bar_thick:.2f}")
            else:
                # 仍可回退用老的“前向矩形”策略
                length = min(self.paint_rect_len_max, max(self.depth_hard_min_len, med))
                width  = max(0.18, self.paint_rect_width_m)
                cxw = rx + math.cos(ryaw) * (0.5*length)
                cyw = ry + math.sin(ryaw) * (0.5*length)
                self._paint_hard_rect_center(cxw, cyw, ryaw, length, width)
                self._upsert_hard_patch(cxw, cyw, ryaw, length, width, shape='rect')
                self._publish_map_merged()

        # ---- B) 近距硬障：横档/矩形 + TTL + 立即倒车；倒车前优先“朝目标选角” ----
        if med < self.depth_stop_m and self.map is not None:
            if self.depth_bar_enable:
                dist = max(0.10, med + self.depth_bar_center_offset_m)
                cxw = rx + math.cos(ryaw) * dist
                cyw = ry + math.sin(ryaw) * dist
                bar_len   = float(max(0.20, self.depth_bar_len_m))
                bar_thick = float(max(0.06, self.depth_bar_thick_m))
                self._paint_hard_crossbar_center(cxw, cyw, ryaw, bar_len, bar_thick)
                self._upsert_hard_patch(cxw, cyw, ryaw, bar_len, bar_thick, shape='bar')
                # 在横档附近打短 TTL 封堵
                self._stamp_virtual_disk(cxw, cyw, max(self.virtual_block_r_m, 0.25), tnow()+self.virtual_block_ttl)
                self._publish_map_merged()
            else:
                length = min(self.paint_rect_len_max, max(0.18, med))
                width  = max(0.18, self.paint_rect_width_m)
                cxw = rx + math.cos(ryaw) * (0.5*length)
                cyw = ry + math.sin(ryaw) * (0.5*length)
                self._paint_hard_rect_center(cxw, cyw, ryaw, length, width)
                self._upsert_hard_patch(cxw, cyw, ryaw, length, width, shape='rect')
                self._stamp_virtual_disk(cxw, cyw, max(self.virtual_block_r_m, 0.25), tnow()+self.virtual_block_ttl)
                self._publish_map_merged()

            # 为脱困计算“朝目标的安全开阔角”
            yaw_to_goal_safe = self._best_escape_yaw_towards_goal(pose)
            if yaw_to_goal_safe is not None:
                self._escape_target_yaw = yaw_to_goal_safe
            elif depth_suggest_yaw is not None:
                self._escape_target_yaw = depth_suggest_yaw
            else:
                self._escape_target_yaw = None  # 让 _enter_fast_reverse() 用激光兜底

            self.path_cells = []; self.commit_until = 0.0
            self._enter_fast_reverse('depth_near')
        else:
            # ---- C) 仅在“相同朝向且很安全”时，连续 N 帧清除曾经写入的硬墙 ----
            self._try_clear_patches_by_depth(ryaw, med)
            if med >= self.depth_clear_m:
                # 额外“前向清空”，保护冗余
                pass

    # ---------- 目标航向/深度开阔角 → 选最佳脱困角 ----------
    def _goal_yaw(self, pose) -> Optional[float]:
        if self.goal_cell is None or pose is None:
            return None
        gx, gy = self.goal_cell
        wx, wy = self.cell_to_world(gx, gy)
        return math.atan2(wy - pose[1], wx - pose[0])

    def _best_escape_yaw_towards_goal(self, pose) -> Optional[float]:
        if self._last_depth_row_med is None or (tnow() - self._last_depth_ts) > 1.2:
            return None
        yaw_goal = self._goal_yaw(pose)
        if yaw_goal is None:
            return None
        ryaw = pose[2]
        err_desired = self._angdiff(yaw_goal, ryaw)
        row_med = self._last_depth_row_med
        W = row_med.size
        hfov = float(self._last_depth_hfov)
        angs = ((np.arange(W) - (W/2.0)) / (W/2.0)) * (hfov/2.0)
        open_mask = (row_med >= float(self.depth_escape_open_m))
        angs_clipped_mask = (angs >= -self.escape_turn_max_rad) & (angs <= self.escape_turn_max_rad)
        open_mask &= angs_clipped_mask
        if not np.any(open_mask):
            return None
        idx_des = int(np.clip(int(round((err_desired / (hfov/2.0)) * (W/2.0) + W/2.0)), 0, W-1))
        if open_mask[idx_des]:
            choose = angs[idx_des]
        else:
            cand_idx = np.where(open_mask)[0]
            diffs = np.abs(angs[cand_idx] - err_desired)
            scores = -diffs + 0.05 * row_med[cand_idx]
            choose = float(angs[cand_idx[np.argmax(scores)]])
        return float(np.clip(choose, -self.escape_turn_max_rad, self.escape_turn_max_rad))

    # ---------------- Map sub ----------------
    def on_map(self, msg: OccupancyGrid):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if getattr(self, 'map_stamp_last', None) == stamp: return
        self.map_stamp_last = stamp

        h, w = msg.info.height, msg.info.width
        self.res = msg.info.resolution
        self.ox  = msg.info.origin.position.x
        self.oy  = msg.info.origin.position.y
        dat = np.array(msg.data, dtype=np.int16).reshape(h, w)
        self.map = dat

        self.last_map_info = msg.info
        self.last_map_header = msg.header

        self.unknown_mask = (dat < 0)
        free0 = (dat >= 0) & (dat <= self.free_thresh)
        occ0  = (dat >= self.occ_thresh)

        # gap close
        gap_r = int(math.ceil(self.gap_fill_max_m / self.res)) if self.gap_fill_enable else 0
        if gap_r != getattr(self, 'kernel_gap_r', -1):
            self.kernel_gap_r = gap_r
            self.kernel_gap = self._make_disk(gap_r) if gap_r>0 else None
        occ_closed = occ0
        if gap_r > 0:
            occ_closed = self._erode(self._dilate(occ_closed, self.kernel_gap), self.kernel_gap)

        # inflate
        inflate_r = int(math.ceil((self.robot_radius_m + self.inflate_margin_m) / self.res))
        if inflate_r != getattr(self, 'kernel_inflate_r', -1):
            self.kernel_inflate_r = inflate_r
            self.kernel_inflate = self._make_disk(inflate_r) if inflate_r>0 else None
        self.occ_mask  = self._dilate(occ_closed, self.kernel_inflate) if inflate_r>0 else occ_closed
        self.free_mask = free0

        # init layers
        if self.occ_extra_expire is None or self.occ_extra_expire.shape != self.occ_mask.shape:
            self.occ_extra_expire = np.zeros_like(self.occ_mask, dtype=np.float32)
        if self.occ_hard is None or self.occ_hard.shape != self.occ_mask.shape:
            self.occ_hard = np.zeros_like(self.occ_mask, dtype=bool)

        # /map_merged
        self._publish_map_merged(msg)

        # 启动状态迁移
        if self.state == STATE_WAITING and self.pose_in_map() is not None:
            self.state, self.state_t0, self.init_rot_done = STATE_INITIAL_ROT, tnow(), 0.0

    # ---------------- Morph helpers ----------------
    def _make_disk(self, r: int):
        if r <= 0: return None
        if _HAS_CV2:
            try:
                return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*r+1, 2*r+1))
            except Exception:
                pass
        yy, xx = np.ogrid[-r:r+1, -r:r+1]
        return (xx*xx + yy*yy) <= (r*r)

    def _dilate(self, mask, kernel):
        if kernel is None: return mask
        if _HAS_CV2:
            try:
                return cv2.dilate(mask.astype(np.uint8), kernel) > 0
            except Exception:
                pass
        H,W=mask.shape; kr=kernel.shape[0]//2
        out=np.zeros_like(mask, bool)
        for y in range(H):
            y0,y1=max(0,y-kr),min(H,y+kr+1)
            for x in range(W):
                x0,x1=max(0,x-kr),min(W,x+kr+1)
                if np.any(mask[y0:y1,x0:x1] & kernel[kr-(y-y0):kr+(y1-y), kr-(x-x0):kr+(x1-x)]):
                    out[y,x]=True
        return out

    def _erode(self, mask, kernel):
        if kernel is None: return mask
        if _HAS_CV2:
            try:
                return cv2.erode(mask.astype(np.uint8), kernel) > 0
            except Exception:
                pass
        return ~self._dilate(~mask, kernel)

    # ---------------- Effective occupancy ----------------
    def _effective_occ(self) -> np.ndarray:
        if self.occ_mask is None: return None
        virt = (self.occ_extra_expire > tnow()) if self.occ_extra_expire is not None else False
        hard = self.occ_hard if self.occ_hard is not None else False
        base = self.occ_mask
        occ  = base | virt | hard
        # 更保守的“闭口”：仅对短期/硬墙层闭口，且半径 ≤ 通行半径一半
        if self.plan_close_enable and self.plan_close_gap_m > 1e-3:
            safe_gap = min(self.plan_close_gap_m, 0.5 * self.min_clearance_m)
            if safe_gap > 1e-3:
                r = int(max(1, round(safe_gap / max(self.res,1e-3))))
                ker = self._make_disk(r)
                extra = virt | hard
                extra_closed = self._erode(self._dilate(extra, ker), ker)
                occ = base | extra_closed
        return occ

    def _publish_map_merged(self, src: OccupancyGrid=None):
        if not self.publish_map_merged or self.map is None or self.pub_map_merged is None: return
        occ = self.occ_mask | ((self.occ_extra_expire>tnow()) if self.occ_extra_expire is not None else False) | \
              (self.occ_hard if self.occ_hard is not None else False)
        merged = np.full_like(self.map, -1, dtype=np.int16)
        known_free = (~self.unknown_mask) & (~occ)
        merged[known_free] = 0
        merged[occ] = 100
        out = OccupancyGrid()
        if src is not None:
            out.header = src.header; out.info = src.info
        else:
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = 'map'; out.info = self.last_map_info
        out.data = merged.reshape(-1).astype(np.int8).tolist()
        self.pub_map_merged.publish(out)

    # ---------------- Walls ----------------
    def _stamp_virtual_disk(self, cxw:float, cyw:float, radius_m:float, expire_t:float):
        if self.map is None or self.occ_extra_expire is None: return
        r = int(math.ceil(radius_m / self.res))
        cc = self.world_to_cell(cxw, cyw)
        if cc is None: return
        cx,cy = cc
        H,W = self.occ_extra_expire.shape
        y0,y1 = max(0, cy-r), min(H, cy+r+1)
        x0,x1 = max(0, cx-r), min(W, cx+r+1)
        rr2 = r*r
        for y in range(y0,y1):
            for x in range(x0,x1):
                if (x-cx)*(x-cx)+(y-cy)*(y-cy) <= rr2:
                    self.occ_extra_expire[y,x] = max(self.occ_extra_expire[y,x], expire_t)

    def _paint_hard_rect_ahead(self, rx:float, ry:float, yaw:float, length_m:float, width_m:float):
        if self.map is None:
            return
        if self.occ_hard is None:
            self.occ_hard = np.zeros_like(self.occ_mask, dtype=bool)
        step = max(self.res, 0.05)
        half_w = width_m * 0.5
        c, s = math.cos(yaw), math.sin(yaw)
        u = (c, s)      # forward
        v = (-s, c)     # left-orthogonal
        t = 0.0
        while t <= length_m:
            yoff = -half_w
            while yoff <= half_w:
                wx = rx + u[0]*t + v[0]*yoff
                wy = ry + u[1]*t + v[1]*yoff
                cell = self.world_to_cell(wx, wy)
                if cell is not None:
                    cx, cy = cell
                    self.occ_hard[cy, cx] = True
                yoff += step
            t += step
        if self.hard_connect_enable:
            kr = int(max(1, round(self.hard_connect_m / max(self.res, 1e-3))))
            ker = self._make_disk(kr)
            self.occ_hard = self._erode(self._dilate(self.occ_hard, ker), ker)
        if self.hard_thin_enable and self.hard_thin_radius_m > 1e-3:
            tr = int(max(1, round(self.hard_thin_radius_m / max(self.res, 1e-3))))
            tker = self._make_disk(tr)
            self.occ_hard = self._erode(self.occ_hard, tker)
        self.get_logger().info(f"[HardRect] 写入：L≈{length_m:.2f}m W≈{width_m:.2f}m", throttle_duration_sec=1.0)

    def _clear_hard_rect_ahead(self, rx:float, ry:float, yaw:float, length_m:float, width_m:float):
        if self.map is None or self.occ_hard is None:
            return
        step = max(self.res, 0.05)
        half_w = width_m * 0.5
        c, s = math.cos(yaw), math.sin(yaw)
        u = (c, s); v = (-s, c)
        t = 0.0
        cleared = 0
        while t <= length_m:
            yoff = -half_w
            while yoff <= half_w:
                wx = rx + u[0]*t + v[0]*yoff
                wy = ry + u[1]*t + v[1]*yoff
                cell = self.world_to_cell(wx, wy)
                if cell is not None:
                    cx, cy = cell
                    if self.occ_hard[cy, cx]:
                        self.occ_hard[cy, cx] = False
                        cleared += 1
                yoff += step
            t += step
        if cleared > 0:
            self.get_logger().info(f"[HardRect] 清除硬墙 ~{cleared} cells", throttle_duration_sec=1.0)

    # ---------------- Timer / states ----------------
    def on_timer(self):
        if self.state == STATE_WAITING:      self.publish_stop(); return
        if self.state == STATE_INITIAL_ROT:  self._initial_rot(); return
        if self.state == STATE_EXPLORING:    self._exploring();   return
        if self.state == STATE_SLOW_SCAN:    self._slow_spin();   return
        if self.state == STATE_CORRIDOR:     self._corridor();    return
        if self.state == STATE_REVERSING:    self._reverse();     return
        if self.state == STATE_ESCAPE_TURN:  self._escape_turn(); return

    def _initial_rot(self):
        dt = 0.20
        self.init_rot_done += abs(self.rotate_speed)*dt
        tw = Twist(); tw.angular.z = self.rotate_speed; self.pub_cmd.publish(tw)
        if self.init_rot_done >= self.init_rot_total:
            self.publish_stop()
            self.state = STATE_EXPLORING; self.state_t0 = tnow(); self.last_prog_xy = None
            self._pub_state('STATE->EXPLORE')

    def _exploring(self):
        now = tnow()

        # ---------- 是否触发“全局规划” ----------
        def need_global_replan():
            if not self.use_global_tour: return False
            if not self.global_plan_tour: return True
            if (now - self._last_global_plan_ts) >= self.global_replan_min_interval_sec:
                known = self._known_count()
                base  = max(1, self._known_count_last)
                if (known - self._known_count_last) / base >= self.global_replan_known_delta_ratio:
                    return True
            return False

        # ---------- 执行一次全局规划 ----------
        if need_global_replan():
            tour = self._plan_global_tour()
            if tour:
                self.global_plan_tour = tour
                self._last_global_plan_ts = now
                self._known_count_last = self._known_count()
                self.path_cells = []; self.commit_until = 0.0
                self._pub_state('GlobalPlan.new', {'goals': len(tour)})
            else:
                self._pub_state('GlobalPlan.fail')

        # ---------- 拿一个目标并局部A* ----------
        if not self.path_cells:
            if self.use_global_tour and self.global_plan_tour:
                next_goal = self.global_plan_tour.pop(0)
                self.goal_cell = next_goal
                pose = self.pose_in_map()
                if pose is not None:
                    sc = self.world_to_cell(pose[0], pose[1])
                    occ = self._effective_occ()
                    if sc is not None and occ is not None:
                        path = self._astar_path(sc, self.goal_cell, occ)
                        if path and len(path) > 1:
                            self.path_cells = path
                            self.commit_until = now + self.commit_time_sec
                            self._known_at_goal_start = self._known_count()
                            self._publish_planned_path()
                            wx, wy = self.cell_to_world(self.goal_cell[0], self.goal_cell[1])
                            self._publish_marker(self.pub_goal, wx, wy, color=(1.0, 0.6, 0.1), scale=0.16, mid=1)
                        else:
                            self.get_logger().warn(f"[GlobalPlan] A* fail to goal {self.goal_cell}, skip.")
                            self.path_cells = []
            if not self.path_cells:
                if self._plan_main():
                    self.commit_until = tnow() + self.commit_time_sec
                    self._known_at_goal_start = self._known_count()
                    self._publish_planned_path()
                else:
                    self.state = STATE_CORRIDOR; self.state_t0 = tnow(); self.publish_stop(); return
                self.state_t0 = tnow()

        # ---------- 路径有效性验证 + 必要时重规划 ----------
        def validate_and_maybe_replan():
            if not self.replan_on_block_enable or not self.path_cells or self.map is None:
                return False
            occ = self._effective_occ()
            if occ is None:
                return False
            dtm = distance_transform_free(~occ)
            idx = self._path_blocked_idx(occ, dtm)
            if idx >= 0:
                self._path_block_frames += 1
                if self._path_block_frames >= self.block_detect_min_frames:
                    pose = self.pose_in_map()
                    if pose is not None and self.goal_cell is not None:
                        sc = self.world_to_cell(pose[0], pose[1])
                        if sc is not None:
                            newp = self._astar_path(sc, self.goal_cell, occ, dt_cached=dtm)
                            if newp and len(newp) > 2:
                                self.path_cells = newp
                                self.commit_until = tnow() + self.commit_time_sec
                                self._publish_planned_path()
                                self.get_logger().warn(f"[Replan] 路径被阻，保留目标重算 A*（len={len(newp)}）")
                                self._path_block_frames = 0
                                return True
                    self.path_cells = []; self.commit_until = 0.0
                    self.get_logger().warn("[Replan] 路径被阻，放弃目标，寻找新前沿/中心线")
                    return self._plan_main()
            else:
                self._path_block_frames = 0
            return False

        validate_and_maybe_replan()

        # ---------- 跟踪 ----------
        self._track_once()

        # ---------- 到达判定 ----------
        if self.goal_cell is not None:
            pose = self.pose_in_map()
            if pose is not None:
                gx, gy = self.goal_cell
                wx, wy = self.cell_to_world(gx, gy)
                dist = math.hypot(wx - pose[0], wy - pose[1])
                if dist <= self.arrive_thresh_m or len(self.path_cells) <= 2:
                    # 到达：若增益很小 -> taboo
                    if self._known_at_goal_start is not None:
                        known_now = self._known_count()
                        base = max(1, self._known_at_goal_start)
                        ratio = (known_now - self._known_at_goal_start) / base
                        if ratio < self.goal_known_delta_min_ratio:
                            self.taboo_list.append((wx, wy, tnow() + self.taboo_ttl_sec))
                            self.get_logger().info(f"[Arrive] low-gain -> taboo at ({wx:.2f},{wy:.2f})")
                    self._known_at_goal_start = None
                    self.path_cells = []

        # ---------- 跟踪后再次快速验证 ----------
        validate_and_maybe_replan()

        # ---------- 走廊/脱困保底 ----------
        front_min = self._front_min_range()
        self._update_stall_timer(front_min < self.corridor_min_forward)
        if self._stall_fired():
            self._stamp_forward_block_and_reverse()

    def _slow_spin(self):
        tw = Twist(); tw.angular.z = -abs(self.rotate_speed); self.pub_cmd.publish(tw)
        if (tnow() - self.state_t0) >= 3.0:
            self.publish_stop(); self.state = STATE_EXPLORING; self.state_t0 = tnow()

    def _corridor(self):
        if self.last_scan is None:
            self._slow_spin(); return
        msg = self.last_scan
        arr = np.array(msg.ranges, dtype=np.float32)
        arr[~np.isfinite(arr)] = msg.range_max
        N = arr.size
        ang = msg.angle_min + np.arange(N)*msg.angle_increment
        lim = math.radians(60)
        maskf = (ang > -lim) & (ang < lim)
        fwd = arr[maskf]
        if fwd.size < 5:
            self._slow_spin(); return
        left  = arr[(ang >  math.radians(10)) & (ang <  lim)]
        right = arr[(ang < -math.radians(10)) & (ang > -lim)]
        lmed = float(np.median(left))  if np.count_nonzero(left  > 0.05) else 0.0
        rmed = float(np.median(right)) if np.count_nonzero(right > 0.05) else 0.0
        front_min = float(np.percentile(fwd[fwd>0.05], 10)) if np.count_nonzero(fwd>0.05) else 0.0

        err = (rmed - lmed)
        w = float(np.clip(self.corridor_k_ang * err, -self.angular_speed, self.angular_speed))
        bias = 0.06
        if self.branch_bias == 'left':  w += bias
        elif self.branch_bias == 'right': w -= bias

        tw = Twist()
        tw.angular.z = float(np.clip(w, -self.angular_speed, self.angular_speed))
        tw.linear.x  = self.corridor_v if front_min >= self.corridor_min_forward else 0.0
        self.pub_cmd.publish(tw)
        self._update_stall_timer(front_min < self.corridor_min_forward)
        if self._stall_fired(): self._stamp_forward_block_and_reverse()
        if (tnow() - self.state_t0) > 3.0:
            self.state = STATE_EXPLORING; self.state_t0 = tnow()

    # ---- Escape ----
    def _enter_fast_reverse(self, reason:str):
        self.path_cells = []
        self.commit_until = 0.0
        self._stuck_pose = self.pose_in_map()
        self.recovery_until = tnow() + self.recovery_guard_sec

        self.state = STATE_REVERSING
        self.state_t0 = tnow()
        self._this_reverse_sec = max(0.3, self.reverse_time_sec * self.reverse_time_scale)

        side_sign = -1
        if self._escape_target_yaw is None:
            pose = self.pose_in_map()
            yaw_goal_safe = self._best_escape_yaw_towards_goal(pose)
            if yaw_goal_safe is not None:
                self._escape_target_yaw = yaw_goal_safe
                side_sign = +1 if self._escape_target_yaw > 0 else -1
            else:
                yaw = -math.radians(60); side_sign = -1
                try:
                    if self.last_scan is not None:
                        arr = np.array(self.last_scan.ranges, dtype=np.float32)
                        ang = self.last_scan.angle_min + np.arange(arr.size)*self.last_scan.angle_increment
                        left  = arr[(ang >  math.radians(10)) & (ang <  math.radians(60))]
                        right = arr[(ang < -math.radians(10)) & (ang > -math.radians(60))]
                        lmed = float(np.median(left[np.isfinite(left) & (left>0.05)]))  if left.size  else 0.0
                        rmed = float(np.median(right[np.isfinite(right) & (right>0.05)])) if right.size else 0.0
                        if rmed > lmed:
                            yaw =  math.radians(60); side_sign = +1
                        else:
                            yaw = -math.radians(60); side_sign = -1
                except Exception:
                    pass
                self._escape_target_yaw = yaw
        else:
            side_sign = +1 if self._escape_target_yaw > 0 else -1

        self._escape_target_yaw = float(np.clip(self._escape_target_yaw, -self.escape_turn_max_rad, self.escape_turn_max_rad))
        self._escape_side_sign  = side_sign
        self._escape_turn_left  = max(self.escape_turn_min_rad, abs(self._escape_target_yaw))
        self.get_logger().warn(f"[Reverse] FAST {self._this_reverse_sec:.2f}s, 目标角={math.degrees(self._escape_target_yaw):.1f}°（{reason}）")

    def _reverse(self):
        tw = Twist()
        tw.linear.x = -abs(self.reverse_speed)
        tw.angular.z = -abs(self.reverse_turn_speed) if (self._escape_target_yaw or 0.0) <= 0 else abs(self.reverse_turn_speed)
        self.pub_cmd.publish(tw)
        if (tnow() - self.state_t0) >= self._this_reverse_sec:
            pose = self.pose_in_map()
            if pose is not None:
                rx, ry, ryaw = pose
                goal_err = None
                if self.goal_cell is not None:
                    gx, gy = self.goal_cell
                    wx, wy = self.cell_to_world(gx, gy)
                    yaw_goal = math.atan2(wy - ry, wx - rx)
                    goal_err = self._angdiff(yaw_goal, ryaw)

                if goal_err is not None and abs(goal_err) <= self.escape_goal_override_rad:
                    self._escape_target_yaw = float(np.clip(goal_err, -self.escape_turn_max_rad, self.escape_turn_max_rad))
                else:
                    side = self._escape_side_sign if self._escape_side_sign != 0 else (1 if (self._escape_target_yaw or 0.0) > 0 else -1)
                    mag  = max(self.escape_turn_min_rad, abs(self._escape_target_yaw or math.radians(60)))
                    self._escape_target_yaw = float(np.clip(side*mag, -self.escape_turn_max_rad, self.escape_turn_max_rad))

                self._escape_turn_left  = max(self.escape_turn_min_rad, abs(self._escape_target_yaw or 0.0))
            self.publish_stop()
            self.state = STATE_ESCAPE_TURN
            self.state_t0 = tnow()

    def _escape_turn(self):
        if self._escape_turn_left <= 0.0:
            self.publish_stop(); self.path_cells=[]; self.commit_until=0.0
            try:
                if self._stuck_pose is not None:
                    sx, sy, _ = self._stuck_pose
                    self._stamp_virtual_disk(sx, sy,
                        max(0.20, self.post_recovery_block_r_m),
                        tnow() + max(3.0, self.post_recovery_block_ttl))
                    self.taboo_list.append((sx, sy, tnow() + self.taboo_ttl_sec))
            except Exception:
                pass
            self._stuck_pose = None
            self.recovery_until = tnow() + self.recovery_guard_sec
            self.state = STATE_EXPLORING; self.state_t0 = tnow()
            return

        tw = Twist(); tw.linear.x = 0.0
        w = math.copysign(abs(self.angular_speed), -1.0 if (self._escape_target_yaw or 0.0)<0 else 1.0)
        tw.angular.z = w; self.pub_cmd.publish(tw)
        self._escape_turn_left -= abs(w) * 0.20

    # ---------------- Planning ----------------
    def _frontier_mask(self, free, occ, unknown):
        pad=1
        unk_p = np.pad(unknown, pad, 'constant', constant_values=True)
        free_p= np.pad(free,   pad, 'constant', constant_values=False)
        occ_p = np.pad(occ,    pad, 'constant', constant_values=False)
        nb = self._dilate(unk_p, self._make_disk(1))
        return (free_p & nb & (~occ_p))[pad:-pad, pad:-pad]

    def _angle_to_forward_bias(self, rx, ry, ryaw, wx, wy) -> float:
        ang = math.atan2(wy-ry, wx-rx)
        da = abs(self._angdiff(ang, ryaw))
        return min(1.0, da / math.pi)

    def _choose_frontier_goal(self, fr_mask, occ, pose)->Optional[Tuple[int,int]]:
        rx, ry, ryaw = pose
        dt = distance_transform_free(~occ)
        ys, xs = np.where(fr_mask)
        if xs.size==0: return None
        pref = self._preferred_heading(ryaw)
        best=None; best_score=-1e9
        for y,x in zip(ys,xs):
            wx,wy = self.cell_to_world(x,y)
            dt_m = float(dt[y,x]*self.res)
            if dt_m < self.frontier_min_dt_m:
                continue
            d = math.hypot(wx-rx, wy-ry)
            ang = math.atan2(wy-ry, wx-rx)
            bias = max(0.0, math.cos(abs(self._angdiff(ang, pref))))
            back_penalty = self._angle_to_forward_bias(rx, ry, ryaw, wx, wy) * self.forward_penalty_k
            if any(te>tnow() and math.hypot(wx-tx, wy-ty) < self.taboo_radius_m for tx,ty,te in self.taboo_list):
                continue
            score = d + 1.5*dt_m + self.branch_bias_weight*bias - back_penalty
            if score > best_score:
                best_score=score; best=(x,y)
        return best

    def _plan_frontier(self, strict=True)->bool:
        if self.map is None: return False
        pose = self.pose_in_map()
        if pose is None: return False
        sc = self.world_to_cell(pose[0], pose[1])
        if sc is None: return False

        occ = self._effective_occ()
        if not strict:
            min_r = int(math.ceil(self.robot_radius_m / self.res))
            kern = self._make_disk(min_r)
            occ_relax = self._dilate(self.map >= self.occ_thresh, kern) if kern is not None else (self.map >= self.occ_thresh)
            occ = occ_relax | (occ & ~occ_relax)

        fr = self._frontier_mask(self.free_mask, occ, self.unknown_mask)
        if int(np.count_nonzero(fr)) == 0: return False

        goal = self._choose_frontier_goal(fr, occ, pose)
        if goal is None: return False
        path = self._astar_path(sc, goal, occ)
        if not path or len(path)<2: return False
        self.path_cells, self.goal_cell = path, goal
        self._publish_planned_path()
        gx,gy = goal; wx,wy = self.cell_to_world(gx,gy)
        self._publish_marker(self.pub_goal, wx, wy, color=(1.0,0.6,0.1), scale=0.16, mid=1)
        return True

    def _plan_centerline(self)->bool:
        if self.map is None: return False
        pose = self.pose_in_map()
        if pose is None: return False
        rx, ry, _ = pose
        sc = self.world_to_cell(rx, ry)
        if sc is None: return False
        occ = self._effective_occ()
        dt = distance_transform_free(~occ)

        H,W = dt.shape
        reach = np.zeros_like(dt, bool)
        q = deque([sc]); reach[sc[1],sc[0]]=True
        while q:
            x,y=q.popleft()
            for dx,dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx,ny=x+dx,y+dy
                if 0<=nx<W and 0<=ny<H and not reach[ny,nx] and not occ[ny,nx] and dt[ny,nx]*self.res >= self.min_clearance_m:
                    reach[ny,nx]=True; q.append((nx,ny))
        cand = (dt > 1.0) & reach
        ys,xs = np.where(cand)
        if xs.size==0: return False
        best=None; score=-1e9
        for y,x in zip(ys,xs):
            wx,wy = self.cell_to_world(x,y)
            if any(te>tnow() and math.hypot(wx-tx, wy-ty) < self.taboo_radius_m for tx,ty,te in self.taboo_list):
                continue
            s = math.hypot(wx-rx, wy-ry) + (dt[y,x]*self.res)
            if s>score: score=s; best=(x,y)
        if best is None: return False
        path = self._astar_path(sc, best, occ, dt_cached=dt)
        if not path or len(path)<2: return False
        self.path_cells, self.goal_cell = path, best
        self._publish_planned_path()
        wx,wy = self.cell_to_world(best[0],best[1])
        self._publish_marker(self.pub_goal, wx, wy, color=(1.0,0.6,0.1), scale=0.16, mid=1)
        return True

    def _plan_main(self)->bool:
        return self._plan_frontier(True) or self._plan_frontier(False) or self._plan_centerline()

    # ---------------- A* ----------------
    def _astar_path(self, start:Tuple[int,int], goal:Tuple[int,int], occ:np.ndarray, dt_cached:np.ndarray=None):
        H,W = occ.shape
        sx,sy = start; gx,gy = goal
        if not (0<=sx<W and 0<=sy<H and 0<=gx<W and 0<=gy<H): return None
        if occ[sy,sx] or occ[gy,gx]: return None
        dt = dt_cached if dt_cached is not None else distance_transform_free(~occ)
        min_cells = self.min_clearance_m / self.res
        def nbr4(x,y):
            if x>0:   yield x-1,y
            if x<W-1: yield x+1,y
            if y>0:   yield x,y-1
            if y<H-1: yield x,y+1
        openq=[]; heapq.heappush(openq,(0.0,0.0,(sx,sy)))
        came={(sx,sy):None}; gcost={(sx,sy):0.0}
        while openq:
            f,g,(x,y) = heapq.heappop(openq)
            if (x,y)==(gx,gy): break
            for nx,ny in nbr4(x,y):
                if occ[ny,nx]: continue
                if dt[ny,nx] < min_cells: continue
                width_m = float(dt[ny,nx]*self.res)
                step = 1.0 + self.width_cost_k / max(width_m, 1e-3)
                ng = g + step
                if ng + 1e-6 < gcost.get((nx,ny), 1e18):
                    gcost[(nx,ny)] = ng
                    h = math.hypot(gx-nx, gy-ny)
                    heapq.heappush(openq, (ng+h, ng, (nx,ny)))
                    came[(nx,ny)] = (x,y)
        if (gx,gy) not in came: return None
        path=[]; cur=(gx,gy)
        while cur is not None:
            path.append(cur); cur=came[cur]
        path.reverse()
        return path

    # ---------------- Path validation ----------------
    def _path_blocked_idx(self, occ:np.ndarray, dt:np.ndarray) -> int:
        if not self.path_cells or self.res is None: return -1
        look_cells = int(max(1, min(len(self.path_cells)-1, round(self.block_check_ahead_m / self.res))))
        for i in range(1, look_cells+1):
            x,y = self.path_cells[i]
            if x<0 or y<0 or y>=occ.shape[0] or x>=occ.shape[1]: return i
            if occ[y,x] or (dt[y,x]*self.res < self.min_clearance_m):
                return i
        return -1

    # ---------------- Tracking & helpers ----------------
    def _front_min_range(self)->float:
        if self.last_scan is None: return 0.0
        msg = self.last_scan
        arr = np.array(msg.ranges, dtype=np.float32)
        arr[~np.isfinite(arr)] = msg.range_max
        ang = msg.angle_min + np.arange(arr.size)*msg.angle_increment
        mask = (np.abs(ang) <= math.radians(15))
        f = arr[mask]; f = f[(f>0.05) & np.isfinite(f)]
        return float(np.percentile(f, 10)) if f.size else 0.0

    def _track_once(self):
        if not self.path_cells: self.publish_stop(); return
        pose = self.pose_in_map()
        if pose is None: self.publish_stop(); return
        rx,ry,ryaw = pose

        steps = max(1, int(((self.recovery_small_lookahead_m if tnow() < self.recovery_until else self.long_lookahead_m) / self.res)))
        target_idx = min(len(self.path_cells)-1, steps)
        tx,ty = self.path_cells[target_idx]
        wx,wy = self.cell_to_world(tx,ty)
        dx,dy = wx-rx, wy-ry
        dist = math.hypot(dx,dy); ang = math.atan2(dy,dx)
        err  = self._angdiff(ang, ryaw)

        tw = Twist()
        if abs(err) > self.angle_gate_rad:
            tw.angular.z = float(np.clip(1.8*err, -self.angular_speed, self.angular_speed))
            tw.linear.x  = 0.0
        else:
            tw.angular.z = float(np.clip(1.2*err, -self.angular_speed, self.angular_speed))
            tw.linear.x  = self.linear_speed * max(0.0, 1.0 - abs(err)/max(1e-6, self.angle_gate_rad))

        if not (self.recovery_disable_corridor and tnow() < self.recovery_until):
            if len(self.path_cells) > 6:
                x0,y0 = self.path_cells[0]; x1,y1 = self.path_cells[min(6, len(self.path_cells)-1)]
                w0x,w0y = self.cell_to_world(x0,y0); w1x,w1y = self.cell_to_world(x1,y1)
                line_err = abs(self._angdiff(math.atan2(w1y-w0y, w1x-w0x), ryaw))
                if line_err < 0.25:
                    tw.angular.z *= self.corridor_k_ang
                    tw.linear.x   = max(tw.linear.x, self.corridor_v)

        if tnow() < self.recovery_until and getattr(self, '_escape_side_sign', 0) != 0 and abs(err) > self.angle_gate_rad:
            tw.angular.z += 0.08 * self._escape_side_sign
            tw.angular.z  = float(np.clip(tw.angular.z, -self.angular_speed, self.angular_speed))

        self.pub_cmd.publish(tw)
        self._publish_marker(self.pub_target, wx, wy, color=(0.2,0.9,1.0), scale=0.12, mid=2)

        if self.last_prog_xy is None:
            self.last_prog_xy=(rx,ry); self.last_prog_t=tnow()
        elif (tnow() - self.last_prog_t) > self.progress_check_sec:
            d = math.hypot(rx-self.last_prog_xy[0], ry-self.last_prog_xy[1])
            if d < self.min_progress_m:
                self.no_progress_strikes += 1
                self.taboo_list.append((rx, ry, tnow()+self.taboo_ttl_sec))
                self.path_cells = []; self.commit_until=0.0
                if self.no_progress_strikes >= 2:
                    self._enter_fast_reverse('no_progress'); return
            else:
                self.no_progress_strikes = 0
            self.last_prog_xy=(rx,ry); self.last_prog_t=tnow()

        if dist < 0.3*self.long_lookahead_m and len(self.path_cells)>1:
            popn = min(target_idx, max(1, int(0.2*self.long_lookahead_m/self.res)))
            for _ in range(popn):
                if len(self.path_cells)>1: self.path_cells.pop(0)
            self._publish_planned_path()

    def _update_stall_timer(self, stalled_now:bool):
        if stalled_now:
            if self._stall_start_ts is None: self._stall_start_ts = tnow()
        else:
            self._stall_start_ts = None

    def _stall_fired(self)->bool:
        return self._stall_start_ts is not None and (tnow() - self._stall_start_ts) >= self._stall_thresh_s

    def _stamp_forward_block_and_reverse(self):
        pose = self.pose_in_map()
        if pose is None or self.map is None:
            self._enter_fast_reverse('front_stall_no_pose'); return
        rx,ry,ryaw = pose
        d = max(0.22, self.corridor_min_forward)
        cxw = rx + math.cos(ryaw) * (0.6*d)
        cyw = ry + math.sin(ryaw) * (0.6*d)
        self._stamp_virtual_disk(cxw, cyw, max(self.virtual_block_r_m, 0.25), tnow()+self.virtual_block_ttl)
        self.path_cells = []; self.commit_until=0.0
        self._enter_fast_reverse('front_stall')

    def _preferred_heading(self, yaw):
        side_bias = 0.30
        if self.branch_bias == 'left':  return yaw + side_bias
        if self.branch_bias == 'right': return yaw - side_bias
        return yaw

    def _has_any_frontier(self) -> bool:
        occ = self._effective_occ()
        if occ is None or self.free_mask is None or self.unknown_mask is None: return False
        fr = self._frontier_mask(self.free_mask, occ, self.unknown_mask)
        return bool(np.count_nonzero(fr) > 0)

    def _on_stop_now(self, msg: Bool):
        if not msg.data:
            return
        self.get_logger().warn('[StopNow] stop signal received, halting...')
        self.publish_stop()
        try:
            self.timer.cancel()
        except Exception:
            pass
        for _ in range(10):
            self.publish_stop()
            time.sleep(0.02)
        try:
            self.destroy_node()
        except Exception:
            pass

    # ---------------- Global planner ----------------
    def _plan_global_tour(self) -> Optional[List[Tuple[int, int]]]:
        if not self.use_global_tour or self.map is None:
            return None
        pose = self.pose_in_map()
        if pose is None:
            return None
        robot_cell = self.world_to_cell(pose[0], pose[1])
        if robot_cell is None:
            return None

        occ = self._effective_occ()
        if occ is None:
            return None
        fr_mask = self._frontier_mask(self.free_mask, occ, self.unknown_mask)
        pix_rc = np.array(np.where(fr_mask)).T
        N0 = len(pix_rc)
        if N0 < 10:
            return None

        st = max(1, int(self.cluster_downsample_stride))
        if st > 1:
            pix_rc = pix_rc[::st, :]
        centroids_xy = []
        if self.cluster_method == 'dbscan' and _HAS_SKLEARN:
            eps_cells = max(2, int(round(self.dbscan_eps_m / max(self.res,1e-6))))
            db = DBSCAN(eps=eps_cells, min_samples=self.dbscan_min_samples).fit(pix_rc)
            labels = db.labels_
            labs = [l for l in set(labels) if l != -1]
            for k in labs:
                pts = pix_rc[labels==k]
                if pts.shape[0] == 0: continue
                cy, cx = np.mean(pts, axis=0).astype(int)
                centroids_xy.append( (int(cx), int(cy)) )
        else:
            if _HAS_CV2:
                H,W = fr_mask.shape
                img = np.zeros((H,W), np.uint8)
                img[pix_rc[:,0], pix_rc[:,1]] = 255
                num, labels = cv2.connectedComponents((img>0).astype(np.uint8), connectivity=8)
                for k in range(1, num):
                    ys, xs = np.where(labels==k)
                    if ys.size == 0: continue
                    cy, cx = int(np.mean(ys)), int(np.mean(xs))
                    centroids_xy.append( (int(cx), int(cy)) )
            else:
                lab = -np.ones(fr_mask.shape, dtype=np.int32)
                comp = 0
                for (r,c) in pix_rc:
                    if lab[r,c] >= 0: continue
                    comp += 1
                    q = [(r,c)]; lab[r,c] = comp
                    ys=[r]; xs=[c]
                    while q:
                        rr,cc=q.pop()
                        for dr,dc in ((1,0),(0,1),(-1,0),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                            nr,nc=rr+dr,cc+dc
                            if 0<=nr<fr_mask.shape[0] and 0<=nc<fr_mask.shape[1] and fr_mask[nr,nc] and lab[nr,nc]<0:
                                lab[nr,nc]=comp; q.append((nr,nc)); ys.append(nr); xs.append(nc)
                    cy, cx = int(np.mean(ys)), int(np.mean(xs))
                    centroids_xy.append((int(cx), int(cy)))

        if len(centroids_xy) > self.max_cluster_count:
            rx,ry,_ = pose
            d_pairs = []
            for (cx,cy) in centroids_xy:
                wx, wy = self.cell_to_world(cx, cy)
                d_pairs.append((math.hypot(wx-rx, wy-ry), (cx,cy)))
            d_pairs.sort(key=lambda t: t[0])
            centroids_xy = [p for _,p in d_pairs[:self.max_cluster_count]]

        filtered = []
        R = max(1, int(round(self.info_gain_radius_m / max(self.res,1e-6))))
        for (cx, cy) in centroids_xy:
            y0,y1 = max(0, cy-R), min(self.unknown_mask.shape[0], cy+R+1)
            x0,x1 = max(0, cx-R), min(self.unknown_mask.shape[1], cx+R+1)
            u = self.unknown_mask[y0:y1, x0:x1]
            yy,xx = np.ogrid[y0:y1, x0:x1]
            m = ((xx-cx)*(xx-cx)+(yy-cy)*(yy-cy)) <= R*R
            unk_cnt = int(np.count_nonzero(u & m))
            unk_area_m2 = unk_cnt * (self.res ** 2)
            if unk_area_m2 >= self.min_unknown_area_m2:
                filtered.append((cx, cy))
        centroids_xy = filtered
        if not centroids_xy:
            return None

        nodes = [robot_cell] + centroids_xy
        num_nodes = len(nodes)
        occ_coarse, factor = self._coarse_occ(self._effective_occ(), self.global_graph_downsample_factor)
        dt_coarse = distance_transform_free(~occ_coarse)
        cost = np.full((num_nodes, num_nodes), np.inf, dtype=np.float32)
        def coarse_cell(c): return (c[0]//factor, c[1]//factor)
        for i in range(num_nodes):
            cost[i,i] = 0.0
        for i in range(num_nodes):
            si = coarse_cell(nodes[i])
            for j in range(i+1, num_nodes):
                gi = coarse_cell(nodes[j])
                L = self._astar_len_coarse(si, gi, occ_coarse, dt_coarse)
                if L is not None:
                    cost[i,j] = cost[j,i] = float(L)

        reachable = [0]; unvis = set(range(1, len(nodes)))
        while True:
            added=False
            for u in list(unvis):
                if any(np.isfinite(cost[u,v]) for v in reachable):
                    reachable.append(u); unvis.remove(u); added=True
            if not added: break
        if len(reachable) <= 1:
            return None
        nodes = [nodes[k] for k in reachable]
        cost  = cost[np.ix_(reachable, reachable)]

        gains = None
        if self.info_gain_enable:
            gains = np.zeros(len(nodes), dtype=np.float32); gains[0] = 0.0
            RR = max(1, int(round(self.info_gain_radius_m / max(self.res,1e-6))))
            for k in range(1, len(nodes)):
                cx, cy = nodes[k]
                gains[k] = self._unknown_count_disk(cx, cy, RR)

        order = [0]
        remain = list(range(1, len(nodes)))
        if self.global_tour_strategy == 'nn':
            while remain:
                last = order[-1]
                cc = cost[last, remain]
                k = int(np.argmin(cc))
                if not np.isfinite(cc[k]):
                    break
                order.append(remain.pop(k))
        else:
            while remain:
                last = order[-1]
                best_idx = -1
                best_score = -1e9
                for idx, u in enumerate(remain):
                    c = cost[last, u]
                    if not np.isfinite(c):
                        continue
                    g = float(gains[u] if gains is not None else 1.0)
                    score = (g * self.info_gain_beta) / (c + 1.0)
                    if score > best_score:
                        best_score = score; best_idx = idx
                if best_idx < 0: break
                order.append(remain.pop(best_idx))

        final = [nodes[i] for i in order[1:]]
        return final

    def _coarse_occ(self, occ: np.ndarray, ds: int) -> Tuple[np.ndarray, int]:
        ds = max(1, int(ds))
        if ds == 1:
            return occ.copy(), 1
        H, W = occ.shape
        h = H//ds; w = W//ds
        occ_pad = occ[:h*ds, :w*ds]
        occ_blocks = occ_pad.reshape(h, ds, w, ds)
        occ_coarse = np.any(np.any(occ_blocks, axis=3), axis=1)
        return occ_coarse, ds

    def _astar_len_coarse(self, s:Tuple[int,int], g:Tuple[int,int], occ:np.ndarray, dt:np.ndarray)->Optional[int]:
        H,W = occ.shape
        sx,sy=s; gx,gy=g
        if not (0<=sx<W and 0<=sy<H and 0<=gx<W and 0<=gy<H): return None
        if occ[sy,sx] or occ[gy,gx]: return None
        min_cells = max(1, int(round(self.min_clearance_m / max(self.res,1e-6))))
        def nbr4(x,y):
            if x>0:   yield x-1,y
            if x<W-1: yield x+1,y
            if y>0:   yield x,y-1
            if y<H-1: yield x,y+1
        openq=[]; heapq.heappush(openq,(0.0,0.0,(sx,sy)))
        came={(sx,sy):None}; gcost={(sx,sy):0.0}
        while openq:
            f,g,(x,y) = heapq.heappop(openq)
            if (x,y)==(gx,gy): break
            for nx,ny in nbr4(x,y):
                if occ[ny,nx]: continue
                if dt[ny,nx] < min_cells: continue
                ng = g + 1.0
                if ng + 1e-6 < gcost.get((nx,ny), 1e18):
                    gcost[(nx,ny)] = ng
                    h = abs(gx-nx) + abs(gy-ny)
                    heapq.heappush(openq, (ng+h, ng, (nx,ny)))
                    came[(nx,ny)] = (x,y)
        return int(gcost.get((gx,gy), np.inf))

    def _unknown_count_disk(self, cx:int, cy:int, R:int)->int:
        H,W = self.unknown_mask.shape
        y0,y1 = max(0, cy-R), min(H, cy+R+1)
        x0,x1 = max(0, cx-R), min(W, cx+R+1)
        u = self.unknown_mask[y0:y1, x0:x1]
        yy,xx = np.ogrid[y0:y1, x0:x1]
        m = ((xx-cx)*(xx-cx)+(yy-cy)*(yy-cy)) <= R*R
        return int(np.count_nonzero(u & m))

    # ---------------- Save 兜底(若启用) ----------------
    def finish_and_save(self):
        try:
            from cartographer_ros_msgs.srv import FinishTrajectory, WriteState
            cli_finish = self.create_client(FinishTrajectory, '/finish_trajectory')
            cli_write  = self.create_client(WriteState,       '/write_state')
            if cli_finish.wait_for_service(timeout_sec=5.0) and cli_write.wait_for_service(timeout_sec=5.0):
                fut1=cli_finish.call_async(FinishTrajectory.Request(trajectory_id=0))
                rclpy.spin_until_future_complete(self, fut1, timeout_sec=10.0)
                fut2=cli_write.call_async(WriteState.Request(filename='/home/jetson/yahboom_map.pbstream', include_unfinished_submaps=False))
                rclpy.spin_until_future_complete(self, fut2, timeout_sec=10.0)
        except Exception as e:
            self.get_logger().error(f"[Save] exception: {e}")

# ---- main ----
def main(args=None):
    rclpy.init(args=args)
    node = AutoExploreMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
            node.destroy_node()
            rclpy.shutdown()
