# SMART3D Next AI Master Handoff (2026-04-19)

This is the master handoff for the current `Mac <-> i9 <-> Jetson robotcar`
SMART3D stack.

If a future AI only reads one file before touching anything, read this file
first.

## 0. Shutdown State at Handoff Time

As of this handoff:

- Jetson runtime was stopped cleanly.
- Jetson was powered off with `systemctl poweroff`.
- SSH to `jetson@10.66.121.109` timed out after shutdown.
- i9 planner and RViz were stopped by the normal campus stop wrapper.
- i9 watchdogs were left alone.

That means the robot was not left in an active navigation state.

## 1. Read Order

Read these in this order:

1. this file
2. `SMART3D_PITFALLS_AND_FAILURE_ATLAS_2026-04-19.md`
3. `SMART3D_CAMPUS_CONNECTION_GUIDE.md`
4. `SMART3D_JETSON_PI5_REFERENCE_RESTORE_2026-04-19.md`
5. `SMART3D_JETSON_CAMPUS_RUNTIME_2026-04-18.md`

If the next AI needs the exact currently approved planner/client display trio,
also inspect:

- `backups/perfect2_2026-04-19_053032/README.md`
- `backups/perfect2_2026-04-19_053032/i9_planner.py`
- `backups/perfect2_2026-04-19_053032/pi5_client.py`
- `backups/perfect2_2026-04-19_053032/smart_real_robot.rviz`

## 2. The Current Truth in One Paragraph

The current best Jetson robotcar baseline is not the original Pi5 formal
three-source fusion in its pure form. The best working runtime at the end of
this round was:

- Jetson passive chain:
  - LSLiDAR driver
  - HiPNUC IMU on `/IMU_data`
  - `scan_metadata_normalizer.py`
  - Cartographer with `robotcar_2d.lua`
  - `campus_map_bridge.py`
  - `robot_state_publisher`
  - `pi_ros_watchdog.py`
- Jetson active chain:
  - `cmd_vel_driver_pi5_reference.py`
  - `pi5_client.py`
- i9:
  - `i9_planner.py`
  - `rviz2` with `smart_real_robot.rviz`

Important nuance:

- the wrapper profile is still called `pi5_reference`
- but wheel odom was left disabled by default because the no-wheel baseline was
  more stable in practice
- Smart3D static wall handling is currently based on static repair
  (`update_static`), not blind full rebuild on every map change

## 3. Development History That Actually Matters

The short version of the whole saga:

1. Original Pi5 formal baseline:
   - `/scan + /IMU_data + /odom -> Cartographer`
   - central planning on i9
   - follower on the robot
2. Early Jetson port:
   - many chains were mixed together
   - hybrid single-owner experiments and Pi5 reference semantics got confused
3. LiDAR heading bug:
   - RViz looked off by roughly 45 degrees
   - root cause was not TF
   - root cause was `m10p_uart.yaml` using `compensation: true`
   - fixed by setting `compensation: false`
4. Catastrophic drift bug:
   - not normal slow IMU drift
   - looked like sudden relocalization to a place far outside the local map
   - strongest evidence pointed at scan timing / subdivision order instability
   - `scan_metadata_normalizer.py` became part of the current baseline
5. Wheel odom return attempt:
   - wheel odom can help local turning if it is good
   - but bad wheel odom and RS485 issues can poison the chain
   - the stable baseline backed away from fused wheel odom for now
6. Smart3D runtime recovery:
   - path origin issue was fixed so planned path does not start from a stale far
     point
   - static map updates now use static repair logic instead of always forcing a
     full rebuild
   - display path was split from control path
7. Final visual tweak round:
   - `perfect1`, `perfect_final`, `perfect_final_display`, and `perfect2`
     backups were created
   - `perfect2` is the approved display/control/planner snapshot to preserve

## 4. Machines and Connection Points

### Mac

This local machine stores the canonical control tree:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d`

### i9 Alienware

- host example: `felix@10.66.112.64`
- SSH key from Mac:
  - `/Users/felixfang/.ssh/rpi5_robot`
- remote SMART3D dir:
  - `/home/felix/SMART3D`
- remote RViz dir:
  - `/home/felix/smart_rviz_bridge`

Notes:

- i9 campus IP is dynamic
- do not hardcode `10.66.112.64` as eternal truth
- when off-campus, the i9 Cloudflare SSH path was proven useful in earlier
  rounds

### Jetson Orin NX 16GB

- host example: `jetson@10.66.121.109`
- password: `jetson`
- runtime dir:
  - `/home/jetson/SMART3D/jetson_runtime`

Notes:

- Jetson campus IP is dynamic
- do not assume `10.66.121.109` will stay valid forever

### GitHub

The robot handoff tree was also published to the private repo under:

- `FelixFtch/SMART3D-MultiAgent`
- path:
  - `jetsoncar/`

Direct URL:

- <https://github.com/FelixFtch/SMART3D-MultiAgent/tree/main/jetsoncar>

Do not confuse this with the mistakenly created standalone `jetsoncar` repo.

## 5. Canonical Local File Tree

The current control tree is:

```text
/Users/felixfang/Documents/Playground/connection-guideline/smart3d
├── SMART3D_CAMPUS_CONNECTION_GUIDE.md
├── SMART3D_CAMPUS_RUNTIME.md
├── SMART3D_JETSON_CAMPUS_RUNTIME_2026-04-18.md
├── SMART3D_JETSON_ORIN_NX16_MIGRATION_FROM_PI5.md
├── SMART3D_JETSON_PI5_REFERENCE_RESTORE_2026-04-19.md
├── SMART3D_NEXT_AI_MASTER_HANDOFF_2026-04-19.md
├── SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md
├── SMART3D_PITFALLS_AND_FAILURE_ATLAS_2026-04-19.md
├── backups/
├── i9/
├── jetson/
├── pi/
├── research/
├── rviz/
├── scripts/
└── vendor/
```

Most important subtrees:

```text
jetson/
  deploy_runtime_to_jetson.sh
  jetson_runtime_ctl.sh
  m10p_uart.yaml
  scan_metadata_normalizer.py
  start_full_native.sh
  start_pi5_reference_passive.sh
  start_pi5_reference_active.sh
  start_pi5_reference_campus_support.sh
  macbook_wasd_to_jetson.sh
  run_robotcar_wasdqe_test.sh
  lidar_supervisor.sh
  stationary_slam_debug.py
  run_stationary_slam_debug.sh

scripts/
  smart3d_jetson_campus.sh
  smart3d_campus.sh
  jetson_logitech_latest_view.sh

rviz/
  smart_real_robot.rviz

pi/
  campus_map_bridge.py
  pi_ros_watchdog.py

backups/
  imu_lidar_stable_2026-04-19_000927/
  jetson_perfect1_2026-04-19_045543/
  perfect_final_2026-04-19_052436/
  perfect_final_display_2026-04-19_052722/
  perfect2_2026-04-19_053032/
```

## 6. Remote Runtime Shape

### Jetson runtime

Expected deployed runtime location:

```text
/home/jetson/SMART3D/jetson_runtime
```

Important files there:

- `m10p_uart.yaml`
- `scan_metadata_normalizer.py`
- `start_full_native.sh`
- `start_pi5_reference_passive.sh`
- `start_pi5_reference_active.sh`
- `start_pi5_reference_campus_support.sh`
- `cmd_vel_driver_pi5_reference.py`
- `pi5_client.py`
- `campus_map_bridge.py`
- `pi_ros_watchdog.py`
- `robotcar.urdf`

### i9 runtime

Expected deployed files:

- `/home/felix/SMART3D/i9_planner.py`
- `/home/felix/smart_rviz_bridge/smart_real_robot.rviz`
- `/tmp/cyclone-campus-planner-jetson.xml`
- `/tmp/cyclone-campus-rviz-jetson.xml`
- `/tmp/cyclone-campus-validate-jetson.xml`
- `/tmp/cyclone-campus-watch-jetson.xml`

## 7. What the Current Startup Wrapper Really Does

Canonical wrapper:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh`

Important defaults at the time of handoff:

- `JETSON_RUNTIME_PROFILE=pi5_reference`
- `JETSON_PI5_REFERENCE_CARTO_CONFIG_BASENAME=robotcar_2d.lua`
- `PI5_EMERGENCY_STOP_DIST=0.05`
- `I9_MAP_CHANGE_INTERVAL_SEC=0.8`
- `I9_MAP_CHANGE_THRESHOLD=0.03`
- `I9_MAP_CHANGE_MIN_CELLS=30`
- `START_PI5_REFERENCE_WHEEL_ODOM=0`

That means:

- Cartographer uses IMU
- Cartographer does not use wheel odom by default
- the front emergency stop threshold is 5 cm
- planner map-change response is more aggressive than before

## 8. Canonical Start / Stop / Test Commands

### Start passive

```bash
JETSON_RUNTIME_PROFILE=pi5_reference \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh start-passive
```

### Validate

```bash
JETSON_RUNTIME_PROFILE=pi5_reference \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh validate
```

### Start active

```bash
JETSON_RUNTIME_PROFILE=pi5_reference \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh start-active
```

### Goal-test

```bash
JETSON_RUNTIME_PROFILE=pi5_reference \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh goal-test
```

### Full active

```bash
JETSON_RUNTIME_PROFILE=pi5_reference \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh full-active
```

### Status

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh status
```

### Stop

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh stop
```

## 9. What Counts as a Healthy Bring-Up

After passive start, expect:

- `/scan`
- `/IMU_data`
- `/map_campus`
- `/car1/pose`

Soft checks:

- `/odom` may be absent if wheel odom is intentionally off
- `/front_range` is not the primary proof

After active start, expect:

- `cmd_vel_driver_pi5_reference.py` alive on Jetson
- `pi5_client.py` alive on Jetson
- `i9_planner.py` alive on i9
- `rviz2` alive on i9

After `goal-test`, expect:

- i9 publishes `/goal_pose`
- Jetson receives non-empty `/car1/path`
- `pi5_client.py` logs goal / path activity
- planner does not keep publishing a path that starts far behind the car

## 10. Current Behavior of the Planner and Follower

### Planner side

The currently approved planner behavior includes:

- static wall-triggered repair using `update_static`
- fresh-wall filtering before dynamic classification
- empty-path clearing
- separate RViz display path topic:
  - `/car1/path_display`
- periodic display path refresh so walked segments visually disappear

This is why the approved RViz config must subscribe to:

- `/car1/path_display`

not the raw control path topic.

### Follower side

The currently approved follower behavior is the reverted, simpler version:

- it still uses anchor-jump limits
- it does not use the rejected "hold stale path and wait" logic

That stale/hold experiment made the robot wait for the path to disappear before
moving. It was rejected and reverted.

## 11. UDP / Camera Notes

Canonical camera helper:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/jetson_logitech_latest_view.sh`

Supported modes:

- `udp`
- `local`
- `stop`
- `status`
- `ros-camera`

What it does right:

- low-latency ffmpeg sender from Jetson
- low-latency ffplay viewer on i9
- intentionally tries to prefer newest frames over perfect reliability

What repeatedly went wrong:

- video and SLAM together can load Jetson enough to trigger:
  - `Dropped earlier points`
  - TF timing issues
  - map lag
  - larger localization instability during motion

Therefore:

- do not run UDP video during serious long-run SLAM diagnosis unless that exact
  combined load is the thing being tested
- for navigation debugging, turning camera off often gives cleaner evidence

## 12. Direct Keyboard Control

Canonical wrapper on Mac:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/macbook_wasd_to_jetson.sh`

Canonical launcher on Jetson:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/run_robotcar_wasdqe_test.sh`

The wrapper kills:

- `cmd_vel_driver.py`
- `ddsm115_odom.py`
- `stp23l_hard_brake_guard.py`

before handing `/dev/ttyUSB0` to the direct-control script.

Important truth:

- the keyboard test is for direct motor ownership
- it is not the same runtime as the normal SMART3D active chain

## 13. Repeated Errors That Must Not Be Forgotten

1. LiDAR direction looked off by roughly 45 degrees.
   - not TF
   - fixed by `m10p_uart.yaml`:
     - `compensation: false`

2. Catastrophic giant jumps.
   - not simple IMU drift
   - more like wrong SLAM relocalization / timing chain failure
   - often correlated with `Dropped earlier points`

3. Planner started before sensing was truly healthy.
   - this created ghost symptoms that looked like planner bugs

4. Local repository confusion.
   - the old private repo copy was disturbed during an earlier sync mistake
   - do not blindly trust `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent`
   - prefer:
     - this control tree
     - `backups/perfect2_2026-04-19_053032`
     - GitHub `SMART3D-MultiAgent/jetsoncar`

5. Green path residual confusion.
   - raw path topic and display path topic must stay separate

6. Wheel odom over-optimism.
   - wheel odom can help local turning
   - bad wheel odom can also damage the chain quickly

7. Camera load confusion.
   - video delay issues and SLAM timing failures can show up together
   - that does not mean UDP itself is the only cause

8. Static wall lag confusion.
   - a wall can be obvious in scan before it becomes a stable occupied wall in
     the known-wall set
   - Cartographer occupancy update lag is part of that story

## 14. Backups That Matter

### `imu_lidar_stable_2026-04-19_000927`

Use when:

- you want the cleanest no-wheel LiDAR+IMU stability baseline
- you need to re-prove stationary stability first

### `jetson_perfect1_2026-04-19_045543`

Use when:

- you want the first user-approved "this actually feels much better" version

### `perfect_final_2026-04-19_052436`

Use when:

- you need the near-final logic before display tweaks

### `perfect_final_display_2026-04-19_052722`

Use when:

- you need the display-specific improvement snapshot

### `perfect2_2026-04-19_053032`

Use when:

- you want the approved planner + follower + RViz display trio
- you need the easiest rollback for the current end-state

## 15. What the Next AI Should Do First

1. Read this file.
2. Read the failure atlas.
3. Decide whether the task is:
   - runtime recovery
   - wheel odom return
   - SLAM timing diagnosis
   - display-only tweak
4. If runtime recovery:
   - restore the no-wheel stable baseline first
   - only then add wheel odom if the task truly needs it
5. If wheel odom return:
   - verify common ground
   - verify RS485 owner model
   - verify low-speed motion before long autonomous runs

## 16. Paper / Benchmark Side Note

Separate from the robot runtime:

- `mega_university` 1000-trial benchmark runs were already completed
- key JSON outputs were previously reported on i9
- single-agent mega_university mean rebuild count was not recorded and remains
  `n/a`

Do not rerun robot bring-up just to answer the paper-table question.

## 17. Final Summary

At the end of this session:

- Jetson is off
- active navigation chain is not running
- the best current robot runtime is the no-wheel IMU+LiDAR-centered baseline
  wrapped in the current Jetson campus scripts
- `perfect2` is the approved planner/follower/display snapshot
- the next AI should treat wheel odom as an optional enhancement, not as the
  unquestioned baseline
