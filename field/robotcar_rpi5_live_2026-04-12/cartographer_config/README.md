# RobotCar Cartographer SLAM Setup

## 走廊不丢了：LiDAR + IMU 融合建图

### 文件清单
| 文件 | 用途 |
|------|------|
| `robotcar_2d.lua` | Cartographer 配置（已针对走廊优化） |
| `cartographer_launch.py` | ROS2 Launch 文件（自动 remap /IMU_data） |
| `carto_bringup.sh` | 一键启动脚本（在 HOST 上运行） |
| `install_cartographer.sh` | 安装脚本（在容器内运行） |
| `stp23l_range.py` | STP-23L 前方测距 + 紧急刹车节点 |
| `mk_stp23l_dev.sh` | STP-23L 设备符号链接脚本 |

### 安装步骤

```bash
# 1. 把文件复制到容器
docker cp cartographer_config ros2-humble:/root/

# 2. 进容器安装 cartographer
docker exec -it ros2-humble bash
bash /root/cartographer_config/install_cartographer.sh
exit

# 3. 把 bringup 脚本复制到 host
cp cartographer_config/carto_bringup.sh ~/
chmod +x ~/carto_bringup.sh

# 4. 一键启动
~/carto_bringup.sh

# 5. 打开 RViz 看地图
docker exec -it ros2-gui bash -lc '
  source /opt/ros/humble/setup.bash
  export ROS_DOMAIN_ID=42
  export LIBGL_ALWAYS_SOFTWARE=1
  export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
  export QT_X11_NO_MITSHM=1
  rviz2
'
```

### RViz 设置
- Fixed Frame: `map`
- Add → Map → Topic: `/map`
- Add → LaserScan → Topic: `/scan`
- Add → TF（看到 map → odom → base_link → laser_link）

### TF 树
```
map (cartographer)
 └── odom (cartographer)
      └── base_link (cartographer)
           └── laser_link (static_transform_publisher)
```

### 走廊优化要点
- `use_online_correlative_scan_matching = true` 宽搜索窗口
- IMU 提供旋转约束，防止沿走廊方向漂移
- `constraint_builder.min_score = 0.6` 避免走廊重复几何的误回环

### 注意事项
- 运行 cartographer 时 **不要** 同时运行 `imu_to_tf.py`（冲突）
- cartographer 自己管理 `map → odom → base_link` 的 TF
- 只需要一个 static TF: `base_link → laser_link`
