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
