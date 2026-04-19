# SMART3D Jetson Pi5 Reference Restore - 2026-04-19

## Connection

- Jetson:
  - host: `jetson@10.66.121.109`
  - password: `jetson`
  - runtime dir: `/home/jetson/SMART3D/jetson_runtime`
- i9:
  - host: `felix@10.66.112.64`
  - SSH key: `~/.ssh/rpi5_robot`
  - planner dir: `/home/felix/SMART3D`
  - RViz config: `/home/felix/smart_rviz_bridge/smart_real_robot.rviz`

## What Was Restored

The default Jetson campus wrapper now restores the original Pi5 split instead of the hybrid single-owner runtime:

- Passive chain:
  - static TF
  - LiDAR
  - IMU
  - `imu_qos_bridge.py`
  - `ddsm115_odom.py`
  - Cartographer with `robotcar_2d_odom.lua`
- Active chain:
  - original Pi-style `cmd_vel_driver.py`
  - `pi5_client.py`
- Campus support is started outside the pure passive script:
  - `campus_map_bridge.py`
  - `robot_state_publisher`
  - `pi5_client.py`
  - optional `pi_ros_watchdog.py`

This means the default behavior is back to:

- passive chain owns sensing + mapping
- active chain is started separately
- `ddsm115_odom.py` remains the `/odom` publisher
- `cmd_vel_driver_pi5_reference.py` does **not** publish `/odom`

## Default Start Commands

From Mac:

```bash
JETSON_RUNTIME_PROFILE=pi5_reference \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh start-passive
```

```bash
JETSON_RUNTIME_PROFILE=pi5_reference \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh validate
```

```bash
JETSON_RUNTIME_PROFILE=pi5_reference \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh start-active
```

```bash
JETSON_RUNTIME_PROFILE=pi5_reference \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh goal-test
```

```bash
JETSON_RUNTIME_PROFILE=pi5_reference \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh stop
```

## Files Added / Changed

### New reference scripts

- `connection-guideline/smart3d/jetson/start_pi5_reference_passive.sh`
  - pure Pi-style 6-step passive chain
- `connection-guideline/smart3d/jetson/start_pi5_reference_campus_support.sh`
  - starts campus bridge, robot_state_publisher, pi5_client, watchdog
- `connection-guideline/smart3d/jetson/start_pi5_reference_active.sh`
  - starts original Pi-style cmd_vel driver separately

### Runtime control

- `connection-guideline/smart3d/jetson/jetson_runtime_ctl.sh`
  - added:
    - `start-pi5-passive`
    - `start-pi5-active`
  - healthcheck now treats `/odom` as a first-class signal

### Campus wrapper

- `connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh`
  - default profile is now `JETSON_RUNTIME_PROFILE=pi5_reference`
  - `start-passive` now starts Pi5 reference passive chain by default
  - `start-active` now adds the active chain without restarting the passive chain
  - `validate` now requires:
    - `/scan`
    - `/odom`
    - `/map_campus`
    - `/car1/pose`
  - `front_range` is only a soft check in Pi5 reference mode

### Deployment split

- `connection-guideline/smart3d/jetson/deploy_runtime_to_jetson.sh`
  - deploys both driver variants:
    - `cmd_vel_driver_pi5_reference.py`
    - `cmd_vel_driver_single_owner.py`
  - keeps:
    - `ddsm115_odom.py`

### i9 planner note

- local source:
  - `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/i9_planner.py`
- remote deployed copy:
  - `/home/felix/SMART3D/i9_planner.py`

Current planner copy includes the earlier path fix that prepends the live car pose when the returned path starts away from the car.

## Current Verified Runtime State

Validated on 2026-04-19 after restore:

- `/scan`: present, ~12 Hz
- `/IMU_data`: present, ~100 Hz
- `/odom`: present, from `ddsm115_odom.py`, ~10 Hz
- `/map_campus`: present
- `/car1/pose`: present
- `goal-test`: passed
- i9 planner + RViz: started successfully

Observed Jetson process set after restore:

- `lslidar_driver_node`
- `hipnuc_imu talker`
- `ddsm115_odom.py`
- `cartographer_node`
- `cartographer_occupancy_grid_node`
- `campus_map_bridge.py`
- `robot_state_publisher`
- `pi5_client.py`
- `pi_ros_watchdog.py`
- `cmd_vel_driver_pi5_reference.py` after `start-active`

Observed `/dev/ttyUSB0` owners in active mode:

- `ddsm115_odom.py`
- `cmd_vel_driver_pi5_reference.py`

That dual owner is intentional in the restored Pi5 reference profile. It is not the single-owner experimental runtime.

## Important Non-Reference Files Still Present

These are still in the runtime, but they are **not** the default restored path:

- `start_full_native.sh`
- `cmd_vel_driver_single_owner.py`
- `scan_metadata_normalizer.py`
- `odom_supervisor.sh`
- `start_pi5_three_sensor_fusion.sh`
- `start_rf2o_odom.sh`
- `start_laser_scan_matcher_odom.sh`
- `start_kinematic_icp.sh`

Treat these as experimental / alternate runtime pieces, not the Pi5 reference truth.

## Known Remaining Issue

Even after restoring the Pi5 split, Cartographer on Jetson still shows LiDAR timing warnings such as:

- dropped earlier points
- `Ignored subdivision of a LaserScan message ... previous subdivision time ... is not before current subdivision time`

So:

- the wrapper/runtime topology is now back to Pi5 reference semantics
- but there is still a Jetson-side LiDAR timestamp/order issue worth watching if RViz walls still look offset or suddenly smear

If visual wall mismatch remains, the first place to inspect is now:

- LiDAR scan time / subdivision ordering
- not the old hybrid single-owner wrapper design

## Why The Chassis May Still Look "Not Moving"

Two common reasons in the restored chain:

1. `pi5_client.py` local emergency stop:
   - if front scan is too close, it publishes zero `/cmd_vel`
2. tiny goal already considered reached:
   - planner can immediately mark success if current pose is already within tolerance

Also note:

- if wheel power / RS485 physical wiring is not ready, restore of the software topology does not make the chassis move by itself

