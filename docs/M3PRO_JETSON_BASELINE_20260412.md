# M3Pro Jetson Baseline And Overlay Plan (2026-04-12)

This note records what should go back to private GitHub from the original
Jetson M3Pro machine, and what should be restored from the newer Mac-side
auto-mapper overlay.

## What To Pull Back From The Original Jetson

These are baseline machine components and belong in source control as the
closest recoverable "factory plus tuning" state:

- `field/jetson_m3pro_original_2026-04-12/Version.txt`
- `field/jetson_m3pro_original_2026-04-12/start_agent.sh`
- `field/jetson_m3pro_original_2026-04-12/mircoROS_agent_src/micro_ros_agent/`
- `field/jetson_m3pro_original_2026-04-12/mircoROS_agent_src/snap/`
- `field/jetson_m3pro_original_2026-04-12/yahboomcar_ws_src/OrbbecSDK_ROS2/orbbec_camera/`
- `field/jetson_m3pro_original_2026-04-12/yahboomcar_ws_src/OrbbecSDK_ROS2/orbbec_camera_msgs/`
- `field/jetson_m3pro_original_2026-04-12/yahboomcar_ws_src/OrbbecSDK_ROS2/orbbec_description/`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/M3Pro/`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/M3Pro_core/`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/M3Pro_navigation/`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/slam_mapping/`
- `field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/ekf_bringup/`

These pieces represent the original machine's low-level startup, camera stack,
robot description, EKF wiring, and Cartographer launch path.

## What Should Come From The Mac Overlay Instead

The newer exploration logic should not be taken from the original Jetson. It
should come from the Mac-side package snapshot:

- `extras/m3pro_auto_mapper21_overlay/`

Use that package as:

```bash
~/M3Pro_ws/src/m3pro_auto_mapper
```

The original Jetson audit showed that `m3pro_auto_mapper` was not currently
installed as a discoverable ROS package there, so the Mac snapshot is the
better source of truth for the mapper layer.

## Final Reconstructed Layout On Jetson

```text
~/mircoROS_agent/src/micro_ros_agent
~/yahboomcar_ws/src/OrbbecSDK_ROS2/orbbec_camera
~/yahboomcar_ws/src/OrbbecSDK_ROS2/orbbec_camera_msgs
~/yahboomcar_ws/src/OrbbecSDK_ROS2/orbbec_description
~/M3Pro_ws/src/M3Pro
~/M3Pro_ws/src/M3Pro_core
~/M3Pro_ws/src/M3Pro_navigation
~/M3Pro_ws/src/slam_mapping
~/M3Pro_ws/src/ekf_bringup
~/M3Pro_ws/src/m3pro_auto_mapper
```

## Build And Source Order

Build the three workspaces separately:

```bash
cd ~/mircoROS_agent
colcon build --symlink-install

cd ~/yahboomcar_ws
colcon build --symlink-install

cd ~/M3Pro_ws
colcon build --symlink-install
```

Then keep the runtime shell order aligned with the original machine:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/yahboomcar_ws/install/setup.bash
source /home/jetson/M3Pro_ws/install/setup.bash
export ROS_DOMAIN_ID=30
```

## Known Runtime Entry Points

### micro-ROS agent

The original `start_agent.sh` launches:

```bash
sh ~/start_agent.sh
```

which in turn starts:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/myserial -b 2000000
```

### Main mapping command

The newer mapping flow is the Mac-side auto-mapper launch:

```bash
cd ~/M3Pro_ws
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
  coverage_goal:=0.55 area_goal_m2:=12.0 enclose_goal:=0.30 enclose_tol_m:=0.35 \
  finish_frontier_mode:=interior \
  prefer_fused_map:=true fused_map_topic:=/map_merged also_save_raw_map:=true \
  finish_stable_sec:=2.0 finish_stall_sec:=9999.0 finish_min_delta:=0.01 \
  finish_save_pbstream:=true pbstream_path:=${HOME}/yahboom_map.pbstream \
  finish_save_pgm:=true pgm_map_path:=${HOME}/yahboom_map \
  exit_after_save:=true
```

## Bottom Line

The right split is:

- preserve the original Jetson base and sensor stack from the live machine,
- restore `m3pro_auto_mapper` from the Mac-side overlay,
- rebuild on Jetson with the original workspace/source order,
- keep this entire state in private GitHub so the broken Jetson is no longer a
  single point of failure.
