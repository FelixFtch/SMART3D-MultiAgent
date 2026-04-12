# SMART3D Control Audit (2026-04-12)

This audit records the current control-tree reality after importing the live
`robotcar@raspberrypi` Pi 5 snapshot.

## Executive Summary

The repository currently contains three related but distinct layers:

1. `deploy/i9_planner.py` + `examples/smart_planner_server.rs`
   - Centralized i9 planning and multi-car coordination.
2. `deploy/pi5_client.py` + `deploy/cmd_vel_driver.py`
   - Thin Pi 5 execution client for the centralized architecture.
3. `field/robotcar_rpi5_live_2026-04-12/`
   - Live Pi 5 field tree with Cartographer, HiPNUC IMU/GNSS packages, serial
     tools, and deployment/bootstrap scripts.

The main control chain is real and mostly complete:

`IMU / LiDAR / wheel odom -> Cartographer -> /map,/tf,/pose -> i9_planner.py -> smart_planner_server.rs -> /path -> pi5_client.py -> /cmd_vel -> cmd_vel_driver.py -> DDSM115`

However, the repo is not a single perfectly unified tree yet. The live Pi 5
field snapshot contains upstream sensing/bring-up details that have not been
fully promoted into `deploy/`.

## Canonical Roles

### Centralized Planning Mainline

- `deploy/i9_planner.py`
- `examples/smart_planner_server.rs`
- `deploy/pi5_client.py`
- `deploy/cmd_vel_driver.py`

This is the i9-led architecture:

- Pi 5 publishes pose, scan, and map.
- i9 extracts walls and dynamic obstacles.
- i9 builds and repairs paths.
- Pi 5 follows the returned path and enforces local emergency stop.

### Pi 5 Live Field Tree

- `field/robotcar_rpi5_live_2026-04-12/cartographer_config/`
- `field/robotcar_rpi5_live_2026-04-12/deploy_smart_nav.sh`
- `field/robotcar_rpi5_live_2026-04-12/ros2_ws_src/hipnuc_imu/`
- `field/robotcar_rpi5_live_2026-04-12/ros2_ws_src/hipnuc_gnss/`
- `field/robotcar_rpi5_live_2026-04-12/ros2_ws_src/hipnuc_lib_package/`
- `field/robotcar_rpi5_live_2026-04-12/robot_base.py`
- `field/robotcar_rpi5_live_2026-04-12/checkusb.py`

This is the most complete Pi 5-side field snapshot now stored in the repo.

### Local Standalone Pi 5 Navigation Branch

- `deploy/pi5_reference/smart_nav_node.py`
- `deploy/pi5_reference/smart_nav_node_v3.py`
- `deploy/pi5_reference/smart_nav_node_v4.py`
- `field/robotcar_rpi5_live_2026-04-12/deploy_smart_nav.sh`

These files represent a local-on-Pi planning branch where the Rust planner is
run on the Pi itself instead of delegating planning to the i9 workstation.

## What Is Clearly Complete

### 1. Cartographer and Sensor Tuning Exists

`field/robotcar_rpi5_live_2026-04-12/cartographer_config/robotcar_2d.lua`
contains real corridor-oriented tuning:

- online correlative scan matching
- Ceres scan matcher weights
- IMU usage and gravity time constant
- motion filter thresholds
- submap resolution
- pose graph and loop-closure scoring

This is not placeholder config.

### 2. Pi 5 to i9 Topic Flow Exists

- Pi 5 thin client publishes `/{car}/pose`
- Pi 5 relays `/scan`
- Pi 5 Cartographer publishes `/map`
- i9 planner consumes those topics and publishes `/{car}/path`
- Pi 5 follows the path and outputs `/cmd_vel`
- motor driver converts `/cmd_vel` to RS485 RPM for DDSM115

### 3. DDSM115 Is in the Real Execution Chain

- `deploy/cmd_vel_driver.py` drives DDSM115 over RS485
- `deploy/pi5_reference/ddsm115_odom.py` reads wheel RPM and publishes `/odom`

So DDSM115 is not just an isolated utility; it participates in execution and
optionally odometry.

## What Is Still Split or Incomplete

### 1. Not All Pi 5 Detail Has Been Promoted Into `deploy/`

The live Pi 5 field snapshot is richer than the canonical `deploy/` tree.
Missing-from-mainline examples include:

- Cartographer config files and bring-up scripts
- HiPNUC ROS2 packages
- serial inspection helpers
- some bootstrap logic from `deploy_smart_nav.sh`

### 2. Centralized and Standalone Planning Branches Coexist

There are two valid but different planning paths:

- centralized i9 planning (`deploy/i9_planner.py` + `deploy/pi5_client.py`)
- local Pi 5 planning (`smart_nav_node*.py` and `deploy_smart_nav.sh`)

They should not be treated as one linear version chain.

### 3. Parameter Penetration Is Not Fully Finished

The centralized chain still has a few hardcoded or partially wired values:

- `robot_radius` is declared in `deploy/i9_planner.py` but not fully propagated
  as a per-agent runtime parameter into `examples/smart_planner_server.rs`
- some Pi 5 local scripts still use older `world_size/speed/iters` style planner
  calls instead of the newer bounds-oriented interface

### 4. Wheel-Odom Configuration Is Not Fully Consolidated

`deploy_smart_nav.sh` references a wheel-odom Cartographer variant
(`robotcar_2d_odom.lua`), but that exact file is not present in the imported
field snapshot. This suggests the odom-enabled branch existed operationally,
but is not fully preserved in the current repo tree.

### 5. Some ROS2 Side Packages Look Experimental

The imported HiPNUC packages are valuable, but not every file reads like a
fully cleaned mainline package. Treat them as live-source imports first, then
promote selectively.

## Duplicate Files

These pairs are currently exact duplicates:

- `deploy/pi5_client.py`
- `deploy/pi5_reference/pi5_client.py`

- `deploy/cmd_vel_driver.py`
- `deploy/pi5_reference/cmd_vel_driver.py`

They should eventually be collapsed to one canonical copy plus documentation.

## Recommended Next Steps

1. Keep `field/robotcar_rpi5_live_2026-04-12/` as the immutable live snapshot.
2. Promote selected Pi 5 files into canonical `deploy/` subfolders:
   - `cartographer_config/`
   - HiPNUC ROS2 packages
   - `checkusb.py`
   - `robot_base.py`
3. Mark `deploy/pi5_reference/` explicitly as archive/reference rather than
   pretending it is the only Pi 5 source of truth.
4. Finish parameter plumbing for:
   - `robot_radius`
   - odom-enabled Cartographer config
   - any per-car geometry values

## Bottom Line

The SMART3D control tree is not fake or empty. It already supports a real robot
pipeline with:

- tuned Pi 5 sensing and mapping,
- centralized i9 planning,
- Pi 5 local tracking and emergency stop,
- DDSM115 motor execution.

But the repository still contains a split between:

- the canonical centralized planning path,
- the richer live Pi 5 field snapshot,
- and an older local-on-Pi planning branch.
