# M3Pro New Lidar Practical Checklist (2026-04-13)

This is the shortest practical guide for replacing the lidar on the M3Pro
Jetson stack without breaking Cartographer or the auto-mapper.

## One-Line Rule

The easiest path is:

1. make the new lidar driver publish a valid `sensor_msgs/LaserScan`
2. make TF resolve from the lidar frame to `base_link`
3. keep the final navigation scan topic as `/scan`

If you satisfy those three conditions, the rest of the stack usually needs
little or no change.

## Current Lidar Contract In This Repo

The current stack expects this flow:

```text
driver -> /scan0 and /scan1 -> /scan_multi -> /scan -> Cartographer + mapper
```

Relevant files:

- [base_bringup.launch.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/M3Pro_navigation/launch/base_bringup.launch.py)
- [merge_multi.launch.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/M3Pro_core/ira_laser_tools/launch/merge_multi.launch.py)
- [laserscan_merge.yaml](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/M3Pro_core/ira_laser_tools/config/laserscan_merge.yaml)
- [laser_filter_node.launch.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/M3Pro_core/yahboom_laser_filter/launch/laser_filter_node.launch.py)
- [laser_filter_processor.cpp](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/M3Pro_core/yahboom_laser_filter/src/laser_filter_processor.cpp)
- [cartographer.launch.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/slam_mapping/launch/cartographer.launch.py)
- [lds_2d.lua](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/slam_mapping/config/lds_2d.lua)
- [explore_stack_with_finisher.launch.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/extras/m3pro_auto_mapper21_overlay/launch/explore_stack_with_finisher.launch.py)

## Important Reality Check

The repo preserves the M3Pro lidar consumer package
`yahboom_M3Pro_laser`, but it is not the actual low-level hardware driver for a
new lidar model.

That package contains nodes like:

- `laser_Avoidance`
- `laser_Warning`
- `laser_Tracker`

and they already subscribe to `/scan`.

So when swapping to a new lidar, the real new work is usually:

- add the vendor ROS2 driver package,
- make it publish the expected topic(s),
- make its frame(s) match TF.

## Two Working Integration Paths

### Path A: Single Lidar, Simplest

Use this if the new lidar publishes one clean scan.

Target behavior:

- vendor driver publishes `/scan`
- frame is something like `laser_frame`
- TF resolves `laser_frame -> base_link`

What to change:

1. Keep downstream consumers unchanged.
2. Do not rely on the merge/filter chain if you do not need it.
3. Launch the vendor driver before Cartographer and mapper.

What can stay untouched:

- `slam_mapping/launch/cartographer.launch.py`
- `extras/m3pro_auto_mapper21_overlay/launch/explore_stack_with_finisher.launch.py`
- mapper `scan_topic:=/scan`

### Path B: Dual Lidar, Closest To Existing M3Pro Structure

Use this if you still want front/back or left/right dual lidar layout.

Target behavior:

- driver publishes `/scan0` and `/scan1`
- TF resolves `laser0_frame -> base_link`
- TF resolves `laser1_frame -> base_link`
- `ira_laser_tools` merges them into `/scan_multi`
- `yahboom_laser_filter` republishes filtered `/scan`

What to change:

1. Update `laserscan_topics` in [laserscan_merge.yaml](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/M3Pro_core/ira_laser_tools/config/laserscan_merge.yaml)
2. Update `range_min` and `range_max` there if the new lidar differs
3. Make sure the new lidar frames match the URDF or add matching static TF
4. Keep the final output topic `/scan`

## The Actual File-Level Checklist

### 1. New driver launch

The current `base_bringup.launch.py` only launches:

- IMU filter
- laser merge
- laser filter
- EKF

It does not launch a vendor lidar driver.

So for a new lidar, you need either:

1. a separate driver launch run before `base_bringup`, or
2. a local variant of `base_bringup.launch.py` that includes the vendor driver

## 2. Topic names

These are the important current topic assumptions:

- merge input: `/scan0 /scan1`
- merge output: `/scan_multi`
- filtered output: `/scan`
- mapper input default: `/scan`

Files:

- [laserscan_merge.yaml](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/M3Pro_core/ira_laser_tools/config/laserscan_merge.yaml)
- [laser_filter_processor.cpp](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/M3Pro_core/yahboom_laser_filter/src/laser_filter_processor.cpp)
- [explore_stack_with_finisher.launch.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/extras/m3pro_auto_mapper21_overlay/launch/explore_stack_with_finisher.launch.py)

Best practice:

- keep your final topic as `/scan`
- only change `scan_topic:=...` in the mapper launch if you absolutely must

### 3. Frame names and TF

The preserved M3Pro URDF already includes:

- `base_footprint`
- `base_link`
- `laser0_frame`
- `laser1_frame`

File:

- [M3Pro.urdf](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/M3Pro/urdf/M3Pro.urdf)

If your new lidar uses a different frame name, choose one of these:

1. rename the driver output frame to match existing URDF
2. add a static TF from the new frame to `base_link`
3. update the URDF and downstream configs together

The fastest route is usually option 2.

### 4. Cartographer config

Current Cartographer settings:

- `tracking_frame = "base_footprint"`
- `published_frame = "base_footprint"`
- `odom_frame = "odom"`
- `num_laser_scans = 1`
- `TRAJECTORY_BUILDER_2D.min_range = 0.1`
- `TRAJECTORY_BUILDER_2D.max_range = 10.0`

File:

- [lds_2d.lua](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/slam_mapping/config/lds_2d.lua)

Change this file if:

- the new lidar range is very different
- the lidar is much noisier or shorter-range
- you change frame naming away from `base_footprint`

The most likely edits are:

- `min_range`
- `max_range`
- `tracking_frame`
- `published_frame`

### 5. Laser filtering

The current filter removes returns under `0.18 m` inside the configured angle
window and republishes to `/scan`.

Files:

- [laser_filter_node.launch.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/M3Pro_core/yahboom_laser_filter/launch/laser_filter_node.launch.py)
- [laser_filter_processor.cpp](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/field/jetson_m3pro_original_2026-04-12/M3Pro_ws_src/M3Pro_core/yahboom_laser_filter/src/laser_filter_processor.cpp)

Change this if:

- your new lidar sees the robot body differently
- self-hits are wider or narrower
- the mounting position changed

### 6. Mapper launch

The mapper launch currently defaults to:

- `base_frame:=base_link`
- `scan_topic:=/scan`
- `depth_topic:=/camera/depth/image_raw`
- `robot_radius_m:=0.11`

File:

- [explore_stack_with_finisher.launch.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/extras/m3pro_auto_mapper21_overlay/launch/explore_stack_with_finisher.launch.py)

For a lidar swap, the two important mapper values are usually:

1. `scan_topic`
2. `base_frame`

In most cases you should keep them at:

```bash
scan_topic:=/scan
base_frame:=base_link
```

## The Fastest Working Recipe

If you want the least amount of surgery:

1. install the new lidar vendor ROS2 driver
2. make it publish one scan topic
3. remap or republish that topic to `/scan`
4. add static TF from the new lidar frame to `base_link`
5. keep `lds_2d.lua` unchanged at first
6. run Cartographer
7. run mapper

Only after that works should you touch merge/filter or Cartographer tuning.

## Bring-Up Test Sequence

Use this order on the new Jetson:

### Step 1: driver only

Check:

```bash
ros2 topic list | grep scan
ros2 topic echo /scan --once
```

### Step 2: TF

Check:

```bash
ros2 run tf2_ros tf2_echo base_link laser_frame
```

or whatever your new frame is.

### Step 3: merge/filter if dual lidar

Check:

```bash
ros2 topic echo /scan_multi --once
ros2 topic echo /scan --once
```

### Step 4: Cartographer

Run:

```bash
ros2 launch slam_mapping cartographer.launch.py
```

Check:

```bash
ros2 topic echo /map --once
```

### Step 5: mapper

Run:

```bash
ros2 launch m3pro_auto_mapper explore_stack_with_finisher.launch.py
```

## What I Would Change First On A New Lidar

If you hand me a brand-new lidar tomorrow, I would modify in this order:

1. vendor driver launch
2. TF from lidar frame to `base_link`
3. final scan topic to `/scan`
4. `laserscan_merge.yaml` only if using dual lidar
5. `lds_2d.lua` range values only if scan quality demands it

## Bottom Line

Do not start by rewriting mapper logic.

Start by making the new lidar look like the old contract:

- valid `LaserScan`
- final topic `/scan`
- valid TF to `base_link`

Once that is true, the existing M3Pro navigation and auto-mapper stack can
usually be reused with minimal edits.
