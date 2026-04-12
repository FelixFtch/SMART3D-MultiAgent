# M3Pro Jetson Rebuild And New-Lidar Port Notes (2026-04-13)

This note answers the practical question:

How much of the M3Pro Jetson stack is now preserved in private GitHub, and
what still needs adaptation when rebuilding on another Jetson or replacing the
Pi-based car with this Jetson plus a new lidar?

## What Is Now In Private GitHub

The repo now preserves a near-complete source snapshot for the M3Pro Jetson in:

- `field/jetson_m3pro_original_2026-04-12/`

That snapshot now contains:

- the original low-level startup hook (`start_agent.sh`)
- `mircoROS_agent` source
- the full `M3Pro_ws/src` mapping, navigation, app-layer, and helper packages
  except generated build/install/log trees
- the `yahboomcar_ws/src` control, depth, lidar, description, MoveIt, arm, and
  mediapipe packages except `yahboom_yolov8`
- home-level support scripts such as `joy_control`, `oled_yahboom`, and `set.sh`
- environment manifests for Python, pip, apt manual packages, USB devices, and
  both workspace package lists

The restored mapper overlay is also preserved separately in:

- `extras/m3pro_auto_mapper21_overlay/`

## Why `yahboom_yolov8` Is Not In The Snapshot

The live package contains large binary artifacts such as:

- vendored Torch wheel files
- ONNX and TRT engine files
- model checkpoints
- embedded `.git` pack files

Those make it a bad fit for direct inclusion in the same GitHub repo. The
snapshot instead records its large files in:

- `field/jetson_m3pro_original_2026-04-12/manifests/yahboom_yolov8_large_files.txt`

If later you want that package versioned too, the clean path is either:

1. make a pruned source-only snapshot, or
2. put the heavyweight assets into Git LFS or a release artifact bucket

For mapping, navigation, lidar, and base control, `yahboom_yolov8` is not on
the critical path.

## The Minimum Stack Needed To Replace The Pi Car With This Jetson

If the goal is to move the car over from a Pi-centered stack to the M3Pro
Jetson stack, the minimum preserved packages are now available:

### Base Robot And Navigation

- `M3Pro`
- `M3Pro_core`
- `M3Pro_navigation`
- `slam_mapping`
- `ekf_bringup`
- `m3pro_auto_mapper`
- `yahboom_mapping`
- `updatecostmap`

### Base Control, Description, And Device-Side Packages

- `yahboomcar_ctrl`
- `yahboomcar_msgs`
- `M3Pro_config`
- `yahboom_M3Pro_description`
- `yahboom_M3Pro_laser`
- `yahboom_M3Pro_DepthCam`
- `OrbbecSDK_ROS2/orbbec_camera`
- `OrbbecSDK_ROS2/orbbec_camera_msgs`
- `OrbbecSDK_ROS2/orbbec_description`

### Startup And micro-ROS

- `start_agent.sh`
- `mircoROS_agent`

## What Still Has To Match On A Different Jetson

Even with the source preserved, these machine-specific items still need to be
checked on the new target:

1. Serial device naming
   - `start_agent.sh` currently points at `/dev/myserial`
2. ROS domain and shell source order
   - the preserved baseline uses `ROS_DOMAIN_ID=30`
3. Camera topic names
   - current mapper expects `/camera/depth/image_raw`
4. Lidar topic name and frame naming
   - current stack expects `/scan` and `base_link`
5. USB permissions and device visibility
   - check the captured `lsusb.txt` and package udev scripts
6. Any hardware-specific URDF offsets or laser extrinsics
7. Whether the new car exposes the same `cmd_vel`, odom, and IMU path

## If You Swap To A New Lidar

For a new lidar, the repo now preserves most of what you need, but these are
the main adaptation points:

1. Lidar driver package and launch file
   - likely within `yahboom_M3Pro_laser/` or a replacement driver package
2. `scan_topic`
   - the mapper currently assumes `/scan`
3. TF frame alignment
   - make sure the lidar frame is properly connected to `base_link`
4. Cartographer config
   - review `slam_mapping/config/lds_2d.lua`
   - update laser range/noise settings if the new lidar behaves differently
5. Laser merge/filter path
   - `ira_laser_tools`
   - `yahboom_laser_filter`

If the new lidar still publishes a sane `sensor_msgs/LaserScan` on `/scan` with
good TF, the rest of the stack should need much less change.

## Repo-Level Recovery Path

The repo now includes a rebuild helper:

- `deploy/jetson_m3pro/restore_full_stack.sh`

That script restores the preserved snapshot back into:

- `~/mircoROS_agent/src`
- `~/yahboomcar_ws/src`
- `~/M3Pro_ws/src`

and then reapplies the Mac-side `m3pro_auto_mapper` overlay.

## Bottom Line

Yes, the GitHub repo is now much closer to being a true recovery source for:

- another M3Pro Jetson,
- a Jetson migrated onto the small car,
- or a new-lidar rebuild,

with the remaining work mostly in machine-specific device mapping and topic/TF
matching rather than missing source code.
