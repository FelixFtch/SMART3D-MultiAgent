# m3pro_auto_mapper （融合增强版）

**功能**：Frontier + BFS 自主探索建图 + 双重避障（LiDAR 前向阈值 & 深度中心 0mm）+ 原地左右“择向”自转 + 动态封堵 + 自动保存 pbstream。

## 安装（Jetson Orin NX, 非 Docker）
```bash
mkdir -p ~/M3Pro_ws/src
cp -r m3pro_auto_mapper ~/M3Pro_ws/src/
cd ~/M3Pro_ws
colcon build --packages-select m3pro_auto_mapper
echo "source ~/M3Pro_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 一键运行
```bash
ros2 launch m3pro_auto_mapper auto_mapping.launch.py \
  linear_speed:=0.12 angular_speed:=0.35 rotate_speed:=0.30 \  guard_front_dist:=0.30 guard_hold_sec:=2.0 laser_count_threshold:=10 \  min_progress_m:=0.05 progress_check_sec:=4.0 dynamic_block_radius_cells:=2 \  area_goal_m2:=20.0 coverage_goal:=0.70 \  pbstream_path:="/home/jetson/yahboom_map.pbstream"
```

## 分步运行
1) 相机：`ros2 launch orbbec_camera dabai_dcw2.launch.py`  
2) 低层融合：`ros2 launch M3Pro_navigation base_bringup.launch.py`  
3) Cartographer：`ros2 launch slam_mapping cartographer.launch.py`  
4) 探索节点：
```bash
ros2 run m3pro_auto_mapper auto_explore_mapper \  --ros-args -p linear_speed:=0.12 -p angular_speed:=0.35 -p rotate_speed:=0.30 \             -p guard_front_dist:=0.30 -p guard_hold_sec:=2.0 -p laser_count_threshold:=10 \             -p min_progress_m:=0.05 -p progress_check_sec:=4.0 -p dynamic_block_radius_cells:=2 \             -p area_goal_m2:=20.0 -p coverage_goal:=0.70 \             -p pbstream_path:="/home/jetson/yahboom_map.pbstream"
```

## 保存地图
- **pbstream**：探索完成自动 `/finish_trajectory` + `/write_state` 到 `/home/jetson/yahboom_map.pbstream`
- **PGM/YAML**：`ros2 launch slam_mapping save_map.launch.py map_path:=/home/jetson/yahboom_map`


以下才是经常使用的执行代码



colcon build



source install/setup.bash

ros2 launch m3pro_auto_mapper explore_stack_with_finisher.launch.py   use_rviz:=true   base_frame:=base_link   map_topic:=/map   scan_topic:=/scan   depth_topic:=/camera/depth/image_raw   publish_map_merged:=true   linear_speed:=0.30 angular_speed:=0.70 rotate_speed:=0.50   corridor_v:=0.22 reverse_turn_speed:=0.85   lookahead_m:=0.28 long_lookahead_m:=1.10 replan_interval_sec:=1.5 commit_time_sec:=30.0   free_thresh:=25 occ_thresh:=65 robot_radius_m:=0.11 inflate_margin_m:=0.015   gap_fill_enable:=true gap_fill_max_m:=0.30   min_clearance_m:=0.16 frontier_min_dt_m:=0.22   depth_roi_px:=12 depth_stop_m:=0.33 depth_scale_m_per_unit:=0.001   hard_connect_enable:=true hard_connect_m:=0.30 hard_disk_scale:=0.30   hard_thin_enable:=true hard_thin_radius_m:=0.12   depth_clear_m:=0.85 depth_clear_frames:=3   reverse_time_sec:=1.5   escape_turn_max_rad:=1.57 escape_turn_min_rad:=0.44   virtual_block_ttl_sec:=6.0 virtual_block_radius_m:=0.35   plan_close_enable:=true plan_close_gap_m:=0.06   corridor_k_ang:=0.9 corridor_min_forward:=0.40   width_cost_k:=0.6   replan_on_block_enable:=true block_check_ahead_m:=0.80 block_detect_min_frames:=1   use_global_tour:=true dbscan_eps_m:=0.35 dbscan_min_samples:=6 frontier_thin_stride:=1   info_gain_enable:=true info_gain_radius_m:=1.8 info_gain_beta:=1.0   taboo_radius_m:=1.0 taboo_ttl_sec:=40.0   angle_gate_rad:=0.25   recovery_small_lookahead_m:=0.20 recovery_guard_sec:=2.0 recovery_disable_corridor:=true   post_recovery_block_r_m:=0.35 post_recovery_block_ttl:=8.0   min_unknown_area_m2:=1.0 goal_known_delta_min_ratio:=0.001   depth_hard_enable:=true depth_hard_trigger_m:=0.75 depth_clear_yaw_tol_rad:=0.35 depth_escape_open_m:=0.55   self_finish_enable:=false   coverage_goal:=0.80 area_goal_m2:=20.0 enclose_goal:=0.50 enclose_tol_m:=0.35   finish_stall_sec:=20.0 finish_min_delta:=0.003 finish_stable_sec:=2.0   finish_save_pbstream:=true pbstream_path:=${HOME}/yahboom_map.pbstream   finish_save_pgm:=true pgm_map_path:=${HOME}/yahboom_map   exit_after_save:=true &&   ros2 run nav2_map_server map_saver_cli -f ${HOME}/yahboom_map     --ros-args -p save_map_timeout:=10000 -p free_thresh:=0.25 -p occupied_thresh:=0.65                -p map_subscribe_transient_local:=true





新版本针对20的

colcon build
source install/setup.bash

ros2 launch m3pro_auto_mapper explore_stack_with_finisher.launch.py \
  use_rviz:=true \
  base_frame:=base_link \
  map_topic:=/map \
  scan_topic:=/scan \
  depth_topic:=/camera/depth/image_raw \
  publish_map_merged:=true \
  linear_speed:=0.30 angular_speed:=0.70 rotate_speed:=0.50 \
  corridor_v:=0.22 reverse_turn_speed:=0.85 \
  lookahead_m:=0.28 long_lookahead_m:=1.10 replan_interval_sec:=1.5 commit_time_sec:=30.0 \
  free_thresh:=25 occ_thresh:=65 robot_radius_m:=0.11 inflate_margin_m:=0.015 \
  gap_fill_enable:=true gap_fill_max_m:=0.30 \
  min_clearance_m:=0.16 frontier_min_dt_m:=0.22 \
  depth_roi_px:=12 depth_stop_m:=0.33 depth_scale_m_per_unit:=0.001 \
  hard_connect_enable:=true hard_connect_m:=0.30 hard_disk_scale:=0.30 \
  hard_thin_enable:=true hard_thin_radius_m:=0.12 \
  depth_clear_m:=0.85 depth_clear_frames:=3 \
  reverse_time_sec:=1.5 \
  escape_turn_max_rad:=1.57 escape_turn_min_rad:=0.44 \
  virtual_block_ttl_sec:=6.0 virtual_block_radius_m:=0.35 \
  plan_close_enable:=true plan_close_gap_m:=0.06 \
  corridor_k_ang:=0.9 corridor_min_forward:=0.40 \
  width_cost_k:=0.6 \
  replan_on_block_enable:=true block_check_ahead_m:=0.80 block_detect_min_frames:=1 \
  use_global_tour:=true dbscan_eps_m:=0.35 dbscan_min_samples:=6 frontier_thin_stride:=1 \
  info_gain_enable:=true info_gain_radius_m:=1.8 info_gain_beta:=1.0 \
  taboo_radius_m:=1.0 taboo_ttl_sec:=40.0 \
  angle_gate_rad:=0.25 \
  recovery_small_lookahead_m:=0.20 recovery_guard_sec:=2.0 recovery_disable_corridor:=true \
  post_recovery_block_r_m:=0.35 post_recovery_block_ttl:=8.0 \
  min_unknown_area_m2:=1.0 goal_known_delta_min_ratio:=0.001 \
  depth_hard_enable:=true depth_hard_trigger_m:=0.75 depth_clear_yaw_tol_rad:=0.35 depth_escape_open_m:=0.55 \
  self_finish_enable:=false \
  coverage_goal:=0.80 area_goal_m2:=20.0 enclose_goal:=0.50 enclose_tol_m:=0.35 \
  finish_stall_sec:=20.0 finish_min_delta:=0.003 finish_stable_sec:=2.0 \
  finish_save_pbstream:=true pbstream_path:=${HOME}/yahboom_map.pbstream \
  finish_save_pgm:=true pgm_map_path:=${HOME}/yahboom_map \
  exit_after_save:=true
