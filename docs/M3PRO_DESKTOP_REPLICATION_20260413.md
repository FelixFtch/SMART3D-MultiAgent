# M3Pro 台式机复刻说明

这份说明对应的目标不是“只把代码拉下来”，而是让一台 Ubuntu 台式机尽量复刻 M3Pro Jetson 当前 private GitHub 里保存的整套工作区、依赖和运行入口。

## 适用范围

最适合的目标机：

- Ubuntu 22.04
- 已安装 ROS 2 Humble
- 有 sudo
- 能访问你的 private GitHub

最典型就是你那台 Alienware Linux 台式机。

## 这次已经给你补好的东西

新增桌面恢复脚本：

- `deploy/linux_desktop/restore_full_stack_desktop.sh`

它会做这些事：

1. 从仓库里的快照恢复三套 workspace
   - `~/mircoROS_agent`
   - `~/yahboomcar_ws`
   - `~/M3Pro_ws`
2. 重新覆盖最新版 `m3pro_auto_mapper` overlay
3. 安装台式机构建和运行最常见的 apt 依赖
4. 补安装 Python 侧的大模型/Dify 依赖
5. 跑 `rosdep`
6. 安装 Orbbec udev 规则
7. 生成桌面专用环境脚本
   - `~/smart3d_desktop_env.sh`
8. 生成桌面专用 micro-ROS 启动脚本
   - `~/start_agent_desktop_clone.sh`
9. 生成“还剩哪些机器相关硬编码没处理掉”的检查报告
   - `~/smart3d_desktop_remaining_refs.txt`

## 怎么跑

先把 private repo 克隆到台式机，例如：

```bash
git clone git@github.com:FelixFtch/SMART3D-MultiAgent.git
cd SMART3D-MultiAgent
```

然后直接执行：

```bash
bash deploy/linux_desktop/restore_full_stack_desktop.sh
```

如果你已经自己装好了 apt 依赖，不想重复装：

```bash
SKIP_APT=1 bash deploy/linux_desktop/restore_full_stack_desktop.sh
```

如果你想先只恢复源码和 helper，不立刻 build：

```bash
SKIP_BUILD=1 bash deploy/linux_desktop/restore_full_stack_desktop.sh
```

## 跑完之后先做什么

### 1. 载入环境

```bash
source ~/smart3d_desktop_env.sh
```

### 2. 确认包已经进来了

```bash
ros2 pkg list | rg 'm3pro_auto_mapper|slam_mapping|M3Pro_navigation|orbbec_camera|multi_brains|largemodel'
```

### 3. 看看还剩哪些机器相关路径

```bash
sed -n '1,200p' ~/smart3d_desktop_remaining_refs.txt
```

## 在台式机上最接近“完整复刻”的运行顺序

### micro-ROS

如果你已经知道实际串口设备，例如 `/dev/ttyUSB0`：

```bash
MICROROS_SERIAL_DEV=/dev/ttyUSB0 sh ~/start_agent_desktop_clone.sh
```

如果你坚持沿用原来的 `/dev/myserial` 习惯，就在桌面机也做同样的 udev alias。

### 深度相机

```bash
source ~/smart3d_desktop_env.sh
ros2 launch orbbec_camera dabai_dcw2.launch.py
```

### 自动探索主线

```bash
source ~/smart3d_desktop_env.sh
ros2 launch m3pro_auto_mapper explore_stack_with_finisher.launch.py use_rviz:=true
```

## 现在这台台式机上能复刻到什么程度

### 可以复刻得很完整的部分

- 三套 ROS workspace 代码树
- `m3pro_auto_mapper` 最新 overlay
- `slam_mapping`
- `M3Pro_navigation`
- `M3Pro_core`
- `updatecostmap`
- `largemodel`
- `multi_brains`
- `interfaces`
- `orbbec_camera`
- `micro_ros_agent`
- 大部分 ROS / Python 依赖

### 仍然取决于机器外设和现场条件的部分

1. micro-ROS 串口设备
   - 原机默认是 `/dev/myserial`
2. 麦克风别名
   - 有配置依赖 `/dev/mic`
3. 雷达话题
   - 当前主线默认吃 `/scan`
4. Orbbec 摄像头是否真的接在这台机器上
5. Dify / OpenAI / DashScope 等 API 是否可用
6. 本地 AI 模型文件
   - 当前配置默认找 `~/MODELS/...`
7. Jetson 专属算力路线
   - 例如没被纳入仓库的 `yahboom_yolov8`
   - TensorRT / CUDA 相关资产

## 一个很重要的现实判断

Alienware 这种 x86_64 Ubuntu 台式机，非常适合做：

- 完整源码备份机
- 编译机
- 调参机
- launch/TF/topic 验证机
- Dify / 多智能体 / RAG 接入验证机
- 没接车时的“半实物”环境

但如果你要做到和 Jetson 一模一样地带车跑，还是必须把下面这几项对齐：

- 串口
- 相机
- 雷达
- `/scan`、`/cmd_vel`、`/odom`、IMU
- `ROS_DOMAIN_ID=30`
- `~/MODELS`
- API key / secret

## 这次我顺手做掉的一个可移植改进

`m3pro_auto_mapper` overlay 里原来有几处默认路径写死为：

- `/home/jetson/yahboom_map`
- `/home/jetson/yahboom_map.pbstream`

我已经把 overlay 主线改成默认跟随当前用户的 home 目录了，所以未来恢复到别的机器时，地图默认输出路径会更自然。

## 最值得的下一步

下一步最值的是把桌面机和 Jetson 都统一成同一套“设备映射约定”：

- micro-ROS 串口 alias
- 麦克风 alias
- 激光雷达 topic/frame
- Orbbec 相机 launch

这样你以后换机器时，就不只是“代码能恢复”，而是真能接上设备直接跑。
