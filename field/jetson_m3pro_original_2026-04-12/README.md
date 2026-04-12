# Jetson M3Pro Original Snapshot (2026-04-12)

This folder is the preserved source snapshot from the original M3Pro Jetson
machine at `jetson@10.66.137.171` on April 12, 2026.

It is meant to answer two questions:

1. What belonged to the original Jetson base system and should be backed up?
2. What should be restored from Mac-side mapper work instead of the original
   Jetson snapshot?

## Backed-Up Core From The Original Jetson

These came directly from the live Jetson and should stay as provenance:

- `Version.txt`
- `start_agent.sh`
- `M3Pro_ws_src/M3Pro/`
- `M3Pro_ws_src/M3Pro_core/`
- `M3Pro_ws_src/M3Pro_navigation/`
- `M3Pro_ws_src/slam_mapping/`
- `M3Pro_ws_src/ekf_bringup/`
- `yahboomcar_ws_src/OrbbecSDK_ROS2/orbbec_camera/`
- `yahboomcar_ws_src/OrbbecSDK_ROS2/orbbec_camera_msgs/`
- `yahboomcar_ws_src/OrbbecSDK_ROS2/orbbec_description/`
- `mircoROS_agent_src/micro_ros_agent/`
- `mircoROS_agent_src/snap/`

## Not Present On The Original Jetson As A Built Package

The original Jetson did not have `m3pro_auto_mapper` available from the
installed ROS package index when audited. That mapper layer should come from
the Mac-side overlay snapshot in:

- `extras/m3pro_auto_mapper21_overlay/`

That overlay is the package you later run with:

```bash
ros2 launch m3pro_auto_mapper explore_stack_with_finisher.launch.py ...
```

## Intended Jetson Workspace Layout

To reconstruct the working machine, keep these three workspaces distinct:

1. `~/mircoROS_agent`
   - provides `micro_ros_agent`
2. `~/yahboomcar_ws`
   - provides Orbbec camera packages
3. `~/M3Pro_ws`
   - provides base robot model, navigation, Cartographer wiring, EKF, and
     should also receive `m3pro_auto_mapper`

The target `~/M3Pro_ws/src` layout should include:

- `M3Pro`
- `M3Pro_core`
- `M3Pro_navigation`
- `slam_mapping`
- `ekf_bringup`
- `m3pro_auto_mapper` from `extras/m3pro_auto_mapper21_overlay`

## Source Order Confirmed On The Jetson

The original machine used this shell order:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/yahboomcar_ws/install/setup.bash
source /home/jetson/M3Pro_ws/install/setup.bash
export ROS_DOMAIN_ID=30
```

## Startup Hook Confirmed On The Jetson

`start_agent.sh` launches:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/myserial -b 2000000
```

so `~/mircoROS_agent` is part of the real machine baseline, not an optional
extra.

## Notes

- This snapshot preserves source trees, not full `build/`, `install/`, or
  `log/` workspaces.
- The Orbbec package snapshot excludes upstream `.git` history on purpose.
- `Version.txt` on the original machine reported:

```text
NVMe NO.36, Version: 1.0.1, Released 20251230
```
