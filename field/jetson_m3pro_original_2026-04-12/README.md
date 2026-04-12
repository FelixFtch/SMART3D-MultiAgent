# Jetson M3Pro Original Snapshot (2026-04-12)

This folder is the preserved source snapshot from the original M3Pro Jetson
machine at `jetson@10.66.137.171`, expanded on April 13, 2026 into a more
complete recovery pack.

It is meant to answer three questions:

1. What belonged to the original Jetson base system and should be backed up?
2. What should be restored from Mac-side mapper work instead of the original
   Jetson snapshot?
3. What should live in private GitHub so another Jetson can be rebuilt from it?

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

## Additional Source Trees Now Included

The snapshot now also includes the rest of the moderate-size M3Pro and
Yahboom source packages needed to rebuild a fuller Jetson stack:

- `M3Pro_ws_src/calibration/`
- `M3Pro_ws_src/interfaces/`
- `M3Pro_ws_src/largemodel/`
- `M3Pro_ws_src/largemodel_arm/`
- `M3Pro_ws_src/M3Pro_KCF/`
- `M3Pro_ws_src/m3pro_auto_mapper/`
- `M3Pro_ws_src/multi_brains/`
- `M3Pro_ws_src/patrol/`
- `M3Pro_ws_src/ros_robot_app/`
- `M3Pro_ws_src/testtime/`
- `M3Pro_ws_src/text_chat/`
- `M3Pro_ws_src/updatecostmap/`
- `M3Pro_ws_src/yahboom_mapping/`
- `M3Pro_ws_src/yahboom_multi/`
- `yahboomcar_ws_src/arm_driver/`
- `yahboomcar_ws_src/arm_interface/`
- `yahboomcar_ws_src/arm_kin/`
- `yahboomcar_ws_src/arm_msgs/`
- `yahboomcar_ws_src/M3Pro_config/`
- `yahboomcar_ws_src/M3Pro_demo/`
- `yahboomcar_ws_src/M3Pro_MoveIt_demo/`
- `yahboomcar_ws_src/MoveIt_demo/`
- `yahboomcar_ws_src/test_config/`
- `yahboomcar_ws_src/test_moveit_config/`
- `yahboomcar_ws_src/yahboomcar_ctrl/`
- `yahboomcar_ws_src/yahboomcar_mediapipe/`
- `yahboomcar_ws_src/yahboomcar_msgs/`
- `yahboomcar_ws_src/yahboom_M3Pro_DepthCam/`
- `yahboomcar_ws_src/yahboom_M3Pro_description/`
- `yahboomcar_ws_src/yahboom_M3Pro_laser/`
- `home_support/joy_control/`
- `home_support/oled_yahboom/`
- `home_support/set.sh`
- `manifests/`

## Large Package Deliberately Not Inlined

`yahboom_yolov8` is intentionally not copied into this GitHub snapshot as a
normal source tree because the live package contains large vendored wheels,
weights, engines, and embedded `.git` packs that exceed normal GitHub-friendly
size expectations.

Instead, the repo preserves:

- `manifests/yahboom_yolov8_large_files.txt`
- `manifests/files_over_90MB.txt`

so the omitted large artifacts stay auditable.

## Not Present On The Original Jetson As A Built Package

The original Jetson did not have `m3pro_auto_mapper` available from the
installed ROS package index when first audited. The restored mapper layer
should come from the Mac-side overlay snapshot in:

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
   - provides base robot model, navigation, Cartographer wiring, EKF,
     app-layer ROS packages, and `m3pro_auto_mapper`

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

## Manifests

The `manifests/` folder captures environment-level context from the live
Jetson:

- `colcon_list_M3Pro_ws.txt`
- `colcon_list_yahboomcar_ws.txt`
- `pip3_freeze.txt`
- `apt_manual_packages.txt`
- `python3_version.txt`
- `pip3_version.txt`
- `lsusb.txt`
- `bashrc_ros_lines.txt`

Use these files when rebuilding on another Jetson or when swapping the Pi car
for the M3Pro Jetson stack.

## Notes

- This snapshot preserves source trees, not full `build/`, `install/`, or
  `log/` workspaces.
- The Orbbec package snapshot excludes upstream `.git` history on purpose.
- `yahboomcar_mediapipe` is included even though it contains one large
  landmark model file under 100 MB.
- `Version.txt` on the original machine reported:

```text
NVMe NO.36, Version: 1.0.1, Released 20251230
```
