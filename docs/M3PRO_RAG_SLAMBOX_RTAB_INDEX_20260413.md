# M3Pro RAG / `slam_toolbox` / RTAB 盘点

## 结论先说

1. 你说的 `slambox`，在这套树里对应的是 `slam_toolbox`。
2. 当前 M3Pro 自动探索主线不是 `slam_toolbox`，也不是 RTABMap，而是 `m3pro_auto_mapper` + `cartographer`。
3. `slam_toolbox` 在仓库里有完整的封装入口和参数文件，但底层节点来自外部 ROS 包，不是 vendored 源码。
4. RTABMap 在这台 Jetson 的系统里装得很全，但当前仓库里没有 `rtabmap_ros` 源码；本地只看到一个叫 `rtab_bringup.launch.py` 的桥接启动文件，而且它本身并没有真正启动 `rtabmap` 节点。
5. RAG 这一条不是本地 `faiss/chroma/milvus` 风格的向量库工程，而是 `largemodel` + `multi_brains` + `interfaces` 这套 Dify / 知识库 / RAG pipeline / Qwen 接入层。

## 当前主线到底跑谁

当前自动建图主线明确接的是 `cartographer`：

- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/m3pro_auto_mapper/launch/explore_stack_with_finisher.launch.py`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/m3pro_auto_mapper/launch/explore_stack.launch.py`
- `extras/m3pro_auto_mapper21_overlay/launch/explore_stack_with_finisher.launch.py`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/m3pro_auto_mapper/README.md`

这些地方都把 `slam_mapping/launch/cartographer.launch.py` 当成主建图入口。

## 一、RAG / 大模型 / 知识库 这条线

### 仓库里已经有的核心代码

- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/largemodel/largemodel/model_service.py`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/largemodel/utils/large_model_interface.py`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/largemodel/utils/dify_client2.py`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/largemodel/config/large_model_interface.yaml`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/largemodel/config/map_mapping.yaml`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/multi_brains/multi_brains/model_service.py`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/multi_brains/multi_brains/action_service.py`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/multi_brains/multi_brains/utils/dify_client.py`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/multi_brains/config/multi_brains_setting.yaml`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/interfaces/srv/Qwen25.srv`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/interfaces/srv/LargeScaleModel.srv`

### 这条线已经能证明的能力

仓库里直接能看到这些接口能力：

- `KnowledgeBaseClient`
- `run_rag_pipeline(...)`
- Dify dataset / datasource / pipeline API 封装
- Qwen / LargeScaleModel 的 ROS service 定义
- `map_mapping.yaml` 这类“地图语义映射”配置

### 这条线现在的真实形态

这不是“本地向量数据库项目”那种结构，没在当前快照里看到下面这些本地核心件：

- `faiss`
- `chroma`
- `milvus`
- `llamaindex`
- `langchain` 主框架工程

所以更准确地说，它是：

`机器人 ROS2 行为层` + `Dify/知识库/RAG pipeline 接口层` + `Qwen/多模型调度`

如果你后面要把这条线搬到别的 Jetson，上 GitHub 最值钱的就是上面列出来的 `largemodel`、`multi_brains`、`interfaces` 整包，而不是去找一个并不存在的本地向量库目录。

## 二、`slam_toolbox` 这条线

### 已经盘出来的关键文件

- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/slam_mapping/launch/slam_toolbox.launch.py`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/slam_mapping/launch/online_async_launch.py`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/slam_mapping/config/mapper_params_online_async.yaml`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/slam_mapping/launch/app_Slam_Toolbox.xml`

### 这条线的结构

`slam_mapping/launch/slam_toolbox.launch.py` 会把下面几层串起来：

- `ira_laser_tools`
- `yahboom_laser_filter`
- `imu_filter_madgwick`
- `ekf_bringup`
- `slam_mapping/launch/online_async_launch.py`

其中真正跑图优化的是外部包：

- `package='slam_toolbox'`
- `executable='async_slam_toolbox_node'`

也就是说，这里仓库保存的是：

- 启动编排
- 参数文件
- App 入口

不是 `slam_toolbox` 本体源码。

### 这条线还依赖哪些本地源码

这次快照里已经一起收进来的相关支撑包有：

- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/M3Pro_core/ira_laser_tools`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/M3Pro_core/yahboom_laser_filter`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/M3Pro_core/imu_tools/imu_filter_madgwick`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/ekf_bringup`

相机这层来自另一个 workspace：

- `field/jetson_m3pro_original_2026-04-12/yahboomcar_ws_src/OrbbecSDK_ROS2/orbbec_camera`

### 迁移时别漏掉

如果新 Jetson 要恢复这条线，除了仓库源码，还要确保系统里有 `slam_toolbox` ROS 包本体。

## 三、RTAB / RTABMap 这条线

### 系统里确实装过

这台 Jetson 的包清单里明确记录了整组 RTABMap：

- `field/jetson_m3pro_original_2026-04-12/manifests/apt_manual_packages.txt`

里面能看到：

- `ros-humble-rtabmap`
- `ros-humble-rtabmap-launch`
- `ros-humble-rtabmap-ros`
- `ros-humble-rtabmap-slam`
- `ros-humble-rtabmap-odom`
- `ros-humble-rtabmap-viz`

以及其他整套 `rtabmap-*` 包。

### 但源码树里的实际情况

当前源码树里只盘到一个命名上相关的文件：

- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/M3Pro_navigation/launch/rtab_bringup.launch.py`

这个文件实际做的事情更像：

- 激光合并
- 激光滤波
- IMU 滤波
- EKF
- Orbbec 相机
- `updatecostmap`
- `camera_link` 静态 TF

它并没有直接启动 `rtabmap_ros`、`rtabmap`、`rgbd_odometry` 之类的 RTABMap 主节点。

### 当前判断

所以 RTABMap 在这份快照里的状态更像是：

- 系统环境曾经装全
- 可能做过相关实验
- 但当前 GitHub 主线快照里，并没有一套已经收口好的 RTABMap 主 launch / config / source tree

如果你后面要把 RTABMap 真正做成“可在别的 Jetson 直接落地”的分支，还需要再把：

- 真正的 `rtabmap_ros` 启动文件
- topic remap
- frame 约定
- camera / lidar / odom 的参数收口

单独整理出来。

## 四、现在这份仓库里，最值得继续保留的 portable 子树

如果目标是“别的 Jetson 也能复现”，这一批最关键：

- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/m3pro_auto_mapper`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/slam_mapping`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/M3Pro_navigation`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/largemodel`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/multi_brains`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/interfaces`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/ekf_bringup`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/updatecostmap`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/M3Pro_core`
- `field/jetson_m3pro_original_2026-04-12/yahboomcar_ws_src/OrbbecSDK_ROS2/orbbec_camera`

再配合：

- `deploy/jetson_m3pro/restore_full_stack.sh`
- `extras/m3pro_auto_mapper21_overlay`

就已经很接近一套可迁移底座了。

## 五、还没彻底收口的部分

1. RTABMap 现在更像“环境装过 + 留下桥接入口”，还不是完整主线。
2. `slam_toolbox` 本体依赖仍然是系统安装项，不在仓库里。
3. RAG 依赖的是外部 Dify / 知识库 / API 配置，仓库里保存的是调用层，不是整套本地知识库基础设施。

## 六、一个安全提醒

当前这批 `largemodel` / `multi_brains` 配置文件里带有 API key、接口地址和模型配置。

虽然你现在是 private repo，但从工程治理角度，后面还是建议把这些密钥迁到：

- `.env`
- 未入库配置文件
- 设备本地 secret

而不是长期直接放在源码配置里。
