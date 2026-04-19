# SMART3D Jetson Campus Runtime Status

Last updated: 2026-04-18

This note is the current Jetson Orin NX 16GB status for the SMART3D campus
chain.

Use this together with:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_CAMPUS_CONNECTION_GUIDE.md`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh`

Do not mix this with old `smart3d-campus` or old Jetson/M3PRO trees.

## What exists now

The Jetson side now has a real staged campus orchestration entrypoint:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh`

It wraps:

- Jetson runtime deploy
- i9 planner / RViz / validate XML generation
- passive robot bring-up
- i9-side validation
- active `cmd_vel_driver.py` bring-up
- i9 planner + RViz launch
- `goal-test`
- `estop-test`
- backend compare
- BRIO camera check

The lower-level Jetson runtime controls remain here:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/jetson_runtime_ctl.sh`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/compare_slam_backends.sh`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/lidar_supervisor.sh`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_rf2o_odom.sh`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_laser_scan_matcher_odom.sh`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/measure_odom_candidate.py`

## Current best commands

Current campus IPs observed on 2026-04-18:

- i9: `10.66.112.64`
- Jetson: `10.66.121.109`

IPs are still dynamic. Re-detect every session.

When campus IP access is unavailable from the Mac, the i9 Cloudflare SSH path is
confirmed working:

```bash
ssh -i ~/.ssh/rpi5_robot \
  -o ProxyCommand='/opt/homebrew/bin/cloudflared access ssh --hostname %h' \
  felix@alienware-ssh.mofangpi.net
```

On 2026-04-18 from the current non-campus Mac network, this path still returned:

- host: `felix-Alienware-Aurora-R16`
- user: `felix`

Important limitation:

- no standalone Jetson Cloudflare hostname is documented
- but on 2026-04-18 the Mac-to-campus problem was bypassed successfully with:
  - `Mac -> Cloudflare SSH -> i9 -> SSH jump -> Jetson`
- `~/.ssh/rpi5_robot.pub` is now installed on the Jetson user account
- the Jetson control scripts now support `JETSON_SSH_PROXY_COMMAND`

Changed-network-safe jump proxy used successfully on 2026-04-18:

```bash
JETSON_SSH_PROXY_COMMAND='ssh -i ~/.ssh/rpi5_robot \
  -o StrictHostKeyChecking=no \
  -o UserKnownHostsFile=/tmp/codex_known_hosts_i9 \
  -o ProxyCommand="/opt/homebrew/bin/cloudflared access ssh --hostname alienware-ssh.mofangpi.net" \
  felix@alienware-ssh.mofangpi.net -W %h:%p'
```

Prepare + passive bring-up:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh start-passive
```

Validate from Jetson and i9:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh validate
```

Start active chain:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh start-active
```

Real planner proof:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh goal-test
```

Tiny motion pulse:

```bash
MOVE_TEST_SPEED=0.03 \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh move-test
```

Controlled hard-brake proof:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh estop-test
```

Camera proof:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh camera-check
```

One-shot core bring-up:

```bash
MOVE_TEST_SPEED=0.03 \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh full-active
```

Backend compare:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh benchmark-backends 45
```

Sensors-only bring-up for odom work:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh start-sensors
```

Start `rf2o` odom candidate:

```bash
JETSON_HOST='jetson@10.66.121.109' \
I9_IP='10.66.112.64' \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/jetson_runtime_ctl.sh start-rf2o
```

Start `laser_scan_matcher` odom candidate:

```bash
JETSON_HOST='jetson@10.66.121.109' \
I9_IP='10.66.112.64' \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/jetson_runtime_ctl.sh start-laser-scan-matcher
```

Measure candidate drift + CPU:

```bash
JETSON_HOST='jetson@10.66.121.109' \
I9_IP='10.66.112.64' \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/jetson_runtime_ctl.sh measure-odom rf2o 60
```

Stop:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh stop
```

Cloudflare-capable i9 orchestration:

```bash
I9_HOST='felix@alienware-ssh.mofangpi.net' \
I9_SSH_PROXY_COMMAND='/opt/homebrew/bin/cloudflared access ssh --hostname %h' \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh status
```

Changed-network-safe Jetson orchestration:

```bash
I9_HOST='felix@alienware-ssh.mofangpi.net' \
I9_SSH_PROXY_COMMAND='/opt/homebrew/bin/cloudflared access ssh --hostname %h' \
JETSON_HOST='jetson@10.66.121.109' \
JETSON_SSH_PROXY_COMMAND='ssh -i ~/.ssh/rpi5_robot \
  -o StrictHostKeyChecking=no \
  -o UserKnownHostsFile=/tmp/codex_known_hosts_i9 \
  -o ProxyCommand="/opt/homebrew/bin/cloudflared access ssh --hostname alienware-ssh.mofangpi.net" \
  felix@alienware-ssh.mofangpi.net -W %h:%p' \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh goal-test
```

## What was actually verified

### Core passive chain

After a real Jetson reboot on 2026-04-18, the passive cartographer chain came
up cleanly:

- `/scan` was live
- `/IMU_data` was live
- `/front_range` was live
- `/map` and `/map_campus` were live
- `/car1/pose` was live

Observed local probe from `start_full_native.sh`:

- `/scan msgs=61`
- `/IMU_data msgs=500`
- `/map msgs=9`
- `/car1/pose msgs=50`

Observed 6-second healthcheck after reboot:

- `/scan`: `72`
- `/IMU_data`: `601`
- `/front_range`: `60`
- `/map`: `14`
- `/map_campus`: `6`
- `/car1/pose`: `60`

### i9-side campus validation

From i9, the Jetson chain produced real samples for:

- `/scan`
- `/map_campus`
- `/car1/pose`
- `/front_range`

This means the DDS campus path was actually working, not just local Jetson ROS.

On the changed non-campus Mac network, i9 still reached the Jetson directly:

- `ping 10.66.121.109` succeeded from i9
- `nc -zv 10.66.121.109 22` succeeded from i9
- password SSH to `jetson@10.66.121.109` was confirmed through the i9 jump
- after the public key was installed, direct Mac control via the jump path also
  worked

### rf2o + EKF + IMU fusion

New fusion assets now exist:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/rf2o_odom_for_ekf_params.yaml`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/ekf_rf2o_imu.yaml`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_rf2o_ekf_imu.sh`

Real 60-second stationary measurement on 2026-04-18:

- output topic: `/odom`
- message count: `1801`
- avg rate: `30.02 Hz`
- final translation drift: `0.011554 m`
- final yaw drift: `-0.000004 rad`
- max translation from start: `0.015213 m`
- avg EKF CPU: `6.28%`
- max EKF CPU: `7.2%`
- avg EKF RSS: `44.6 MB`

Practical reading:

- `rf2o` stayed the scan-matching source
- `robot_localization` turned it into a much cleaner `30 Hz` `/odom`
- this is now the preferred odom path to carry forward before testing more
  exotic candidates

### slam_toolbox with rf2o + EKF

This was re-validated remotely on the changed-network pass, not just prepared
locally.

Observed facts:

- Jetson healthcheck showed `/map`, `/map_campus`, `/car1/pose`
- i9 saw `/map_campus`, `/car1/pose`, `/scan`, `/front_range`, `/odom`
- `goal-test` passed:
  - i9 planner received `/goal_pose`
  - planner logged `[build] OK`
  - Jetson captured a non-empty `/car1/path`
  - `pi5_client.py` logged `Goal forwarded`
  - `pi5_client.py` logged `Path received: 2 pts`

Observed stability nuance:

- `lslidar_driver_node` hit a new low-level assertion once:
  - `ddsrt_chh_resize: Assertion 'dist < HH_HOP_RANGE' failed`
- `lidar_supervisor.sh` respawned it and the chain recovered
- slam_toolbox still showed queue-full warnings, so it is working but not fully
  relaxed

### cartographer with rf2o + EKF

This was also re-validated remotely on 2026-04-18.

Observed facts:

- Jetson healthcheck showed `/map`, `/map_campus`, `/car1/pose`
- i9 planner `goal-test` passed with a non-empty `/car1/path`
- `pi5_client.py` logged `Goal forwarded` and `Path received: 2 pts`
- planner build logs were healthy

Observed stability nuance:

- cartographer continued to log `Dropped earlier points`
- the healthcheck process snapshot showed cartographer materially heavier than
  slam_toolbox in this short live sample

Operational reading:

- both SLAM backends now close the Smart3D planning loop with `rf2o + EKF`
- slam_toolbox looks lighter in the short live sample
- cartographer still works, but it remains the heavier path on this Jetson

### Active chain

`cmd_vel_driver.py` started successfully and owned `/dev/ttyUSB0`.

Observed log:

- `RS485 opened /dev/ttyUSB0`
- `cmd_vel driver started`

### Goal-test

The planner loop closed successfully.

Observed facts:

- i9 published `/goal_pose`
- Jetson captured a non-empty `/{car}/path` with 2 poses
- `pi5_client.py` logged `Goal forwarded`
- i9 planner logged `[build]`
- i9 planner logged `Build OK`

Important nuance:

- `pi5_client.py` later logged `Path received: 0 pts`
- this is consistent with current `i9_planner.py`, which publishes an empty
  path after immediate goal completion in order to stop the robot
- so the closure worked, but the final log line is not a reliable indicator of
  whether a non-empty path was published earlier

On a later repeat after the hard-brake recovery test, `pi5_client.py` also
logged:

- `Path received: 2 pts`
- `GOAL REACHED!`

So the post-estop recovery path was proven, not just the first active launch.

### Hard-brake / estop test

`estop-test` was added and verified on 2026-04-18.

What it does:

- temporarily stops the live `stp23l_range.py` publisher
- publishes fake `/front_range=0.04m` for 12 seconds
- waits for `/stp23l_hard_brake_active` to go `true`
- confirms the RS485 owner on `/dev/ttyUSB0` is no longer `cmd_vel_driver.py`
- then publishes fake `/front_range=0.20m` to clear and confirms recovery

Changed-network rerun result on 2026-04-18:

- the rerun succeeded over the `Mac -> Cloudflare -> i9 -> Jetson` jump path
- `/stp23l_hard_brake_active` became `true`
- guard log showed:
  - `Front range under trigger: 0.040m <= 0.050m`
  - `STP23L hard-brake triggered after sustained close-range wall contact`
- while active, `/dev/ttyUSB0` was no longer owned by `cmd_vel_driver.py`; it was
  held by `stp23l_hard_brake_guard.py`
- after publishing fake `0.20m`, `/stp23l_hard_brake_active` returned `false`
- guard log showed `STP23L hard-brake cleared after front range moved above clear distance`
- `stp23l_range.py` and `cmd_vel_driver.py` were both restored successfully, and
  `/front_range` returned to a live value near `3.57m`

This means the emergency-stop path was not only logically triggered; it really
did seize RS485 ownership and then recover cleanly.

### Kinematic-ICP status

The upstream `PRBonn/kinematic-icp` workspace was built successfully on the
Jetson on 2026-04-18:

- workspace: `/home/jetson/SMART3D/ws_kinematic_icp`
- package build result: `Finished <<< kinematic_icp`

New runtime helper added locally and deployed:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_kinematic_icp.sh`
- `jetson_runtime_ctl.sh start-kinematic-icp`

What was verified:

- Kinematic-ICP starts on this Jetson in `LaserScan` mode with:
  - `lidar_topic:=/scan`
  - `use_2d_lidar:=true`
  - `wheel_odom_frame:=odom`
  - `base_frame:=base_link`
  - `lidar_odom_frame:=odom_kinematic`
  - `lidar_odometry_topic:=/lidar_odometry_kinematic`
- it published a live `nav_msgs/msg/Odometry` stream on `/lidar_odometry_kinematic`
- it also published TF as `kinematic_icp_online_node`
- `tf2_echo odom_kinematic base_link` resolved successfully after startup

Important caveat discovered during this smoke:

- with the default launch behavior, Kinematic-ICP repeatedly hit:
  - `Lookup would require extrapolation into the future`
- the immediate cause was that the current prior `odom -> base_link` TF on this
  robot lags scan timestamps enough that a zero TF timeout is too strict
- running the node with `tf_timeout:=0.2` fixed initialization and allowed the
  odometry stream to appear

Operational reading:

- Kinematic-ICP is now buildable and launchable here
- but this was only an integration smoke, not a meaningful odometry benchmark
- the prior fed to it in this smoke was `rf2o + robot_localization`, which is a
  scan-based prior and therefore circular for judging Kinematic-ICP's real value

### Prior odom / TF truth for Kinematic-ICP

The main integration blocker is no longer package build. It is ownership and
trustworthiness of the prior odom/TF contract.

Current practical truth:

- `sensors-only` default: no `/odom`, no `odom -> base_link`
- `slam_toolbox` default campus path: static fake `odom -> base_link` unless
  explicitly changed
- `ddsm115_odom.py`: publishes `/odom` but does not publish TF
- `odom_tf_relay.py`: can relay `/odom` into TF
- `rf2o + EKF`: currently the cleanest live `odom -> base_link` contract

Important caveat:

- live TF inspection on 2026-04-18 still showed multiple `/tf` publishers,
  including `CLaserOdometry2DNode` and `ekf_filter_node`
- the `rf2o` params do show `publish_tf: false`, so at minimum there is still an
  endpoint-level ambiguity that should be cleaned up before treating the prior
  as fully trustworthy

Bottom line:

- Kinematic-ICP is ready as an experimental side branch
- it is **not** ready to replace the mainline odom path yet
- the next meaningful step is not another install pass; it is cleaning up the
  single-owner `odom -> base_link` story, ideally from trustworthy wheel odom
- verifies `cmd_vel_driver.py` is no longer the RS485 owner
- publishes fake `/front_range=0.20m` to clear the brake
- restores `stp23l_range.py`
- restarts `cmd_vel_driver.py`
- verifies `/front_range` and `/dev/ttyUSB0` ownership recovered

Observed facts:

- `/stp23l_hard_brake_active` became `true`
- guard log emitted `STP23L hard-brake triggered after sustained close-range wall contact`
- `cmd_vel_driver.py` was killed
- `/dev/ttyUSB0` moved to `stp23l_hard_brake_guard.py`
- after clear, `/stp23l_hard_brake_active` became `false`
- guard log emitted `STP23L hard-brake cleared after front range moved above clear distance`
- `cmd_vel_driver.py` was restarted and again opened `/dev/ttyUSB0`

### BRIO camera

BRIO was verified separately with `camera-check`.

Confirmed:

- device path exists at `/dev/v4l/by-id/usb-046d_Logitech_BRIO_6A53C801-video-index0`
- Jetson publishes `/camera_info`
- Jetson publishes `/image_raw`
- i9 can see `/camera_info`

Current caveat:

- no camera calibration file yet:
  `/home/jetson/.ros/camera_info/logitech_brio.yaml`

## SLAM backend compare

Compare report:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/reports/slam_compare_2026-04-18_033254/summary.md`

Raw data:

- `cartographer_healthcheck.json`
- `cartographer_metrics.json`
- `slam_toolbox_healthcheck.json`
- `slam_toolbox_metrics.json`

## 2D laser odom candidate A/B

Detailed result note:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/ODOM_CANDIDATE_AB_RESULTS_2026-04-18.md`

Current practical conclusion:

- `rf2o_laser_odometry` is the best current external scan odom candidate
- `ros2_laser_scan_matcher` stayed interesting as a low-drift reference, but it
  was much heavier and only ran around `6 Hz`
- `rf2o` worked with both `slam_toolbox` and cartographer in smoke tests
- `laser_scan_matcher` worked with cartographer, but its `slam_toolbox`
  combination did not produce a healthy map in the smoke window

## Prepared but not re-validated remotely

The following local Jetson runtime files were added after the odom A/B to start
the next integration step, but they were not re-validated on the real Jetson in
the latest changed-network pass because `10.66.121.109` was unreachable:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/rf2o_odom_for_ekf_params.yaml`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/ekf_rf2o_imu.yaml`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_rf2o_ekf_imu.sh`

Planned flow for that branch:

- `rf2o` publishes raw scan odom on `/odom_rf2o`
- `robot_localization` fuses `/odom_rf2o` + `/imu_reliable`
- EKF remaps `odometry/filtered -> /odom`
- EKF becomes the single owner of `odom -> base_link`

This is the right next real-hardware validation step once Jetson SSH is back.

Practical conclusion from the 45-second benchmark:

- `slam_toolbox` used much less CPU than cartographer
- `slam_toolbox` is not yet an equal drop-in replacement for the cartographer
  campus path
- cartographer remains the current mainline for the real Smart3D chain

Important caveats:

- benchmark used `START_ODOM=0`
- `slam_toolbox` depended on `scan_metadata_normalizer.py`
- `slam_toolbox` used static `odom -> base_link`
- this was not a long driving soak or ground-truth comparison

## Real remaining gaps

1. Wheel odometry is still not trusted.
2. `cmd_vel -> motor motion` was only smoke-tested at the driver layer, not
   physically verified from feedback odometry.
3. DDSM115 remains a single-bus ownership problem:
   - `cmd_vel_driver.py`
   - `ddsm115_odom.py`
   - `stp23l_hard_brake_guard.py`
   all want the same RS485 path.
4. BRIO is not part of the required `goal-test` gate and should stay a separate
   module unless there is a good reason to fold it into the core runtime.
5. `stp23l_link` is still not represented in `robotcar.urdf`.
6. LiDAR repeated restart behavior is still fragile enough that a cold reboot is
   sometimes the cleanest recovery.
7. After `stop`, BRIO `v4l2_camera` may still remain running if it was launched
   earlier for camera validation. This is harmless but it means `status` after
   stop is not expected to be a completely empty process list.

## Two fixes made in this round

1. `compare_slam_backends.sh` report generation no longer writes literal `\n`
   into `summary.md`.
2. `pi_ros_watchdog.py` no longer uses a self-killing LiDAR restart command that
   could terminate its own shell when trying to restart `lslidar_driver_node`.
3. `stp23l_hard_brake_guard.py` now closes `/dev/ttyUSB0` when the hard brake
   clears, so `cmd_vel_driver.py` can reclaim the RS485 bus without manual
   cleanup.
4. `smart3d_jetson_campus.sh` now has a first-class `estop-test` command for a
   repeatable hard-brake smoke test and recovery check.

## Failure reproduced in repeated startup

During a repeated cold start on 2026-04-18, LiDAR failed with:

- `malloc(): unaligned tcache chunk detected`

This caused:

- `/scan` publisher count to drop to `0`
- cartographer to wait on scan input
- passive validation to fail

The watchdog did detect the scan gap and attempted restarts, but that exposed a
second parent-shell self-kill bug in the restart path. That restart bug was then
fixed locally. A later full-active retry came up cleanly again and passed
`goal-test`, so the chain is currently usable, but the underlying LiDAR driver
still looks inherently fragile.

## Operational truth for the next session

- Use `smart3d_jetson_campus.sh` as the Jetson campus entrypoint.
- Keep cartographer as the default SLAM backend for the real chain.
- Treat BRIO as separately validated, not as a blocker for core goal-test.
- Do not claim wheel odom is fixed.
- If LiDAR gets into a weird half-alive state after many retries, reboot Jetson
  and start again from `start-passive`.

## Original Pi5 baseline vs current Jetson stopgap

The restore target is not the current Jetson `no-IMU` stopgap.

The original Pi5 formal SMART3D / Cartographer baseline was:

- LiDAR `/scan`
- IMU `/IMU_data`
- wheel odom `/odom`
- Cartographer using all three

The most important original Pi5 files are:

- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/start_full.sh`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/cartographer_launch.py`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d_odom.lua`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/ddsm115_odom.py`

What those files say:

- `start_full.sh` starts LiDAR, HiPNUC IMU, `imu_qos_bridge.py`,
  `ddsm115_odom.py`, then Cartographer.
- `cartographer_launch.py` remaps `scan -> /scan` and `imu -> /IMU_data`.
- `robotcar_2d_odom.lua` sets `use_odometry = true`.
- `robotcar_2d_odom.lua` sets `TRAJECTORY_BUILDER_2D.use_imu_data = true`.

So the original Pi5 formal chain was:

- `/scan`
- `/IMU_data`
- `/odom`
- into Cartographer together

Do not let a future session collapse that into “Pi5 was basically LiDAR-only”.

## Current Jetson stable stopgap

As of 2026-04-18, the most stable Jetson stopgap is narrower than the Pi5
formal baseline:

- Cartographer config:
  - `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d_noimu.lua`
- wheel odom:
  - off during the stability checks (`START_ODOM=0`)
- scan path:
  - Cartographer consumes `/scan_slam`
- IMU device:
  - still publishes `/IMU_data`
- IMU fusion:
  - disabled in Cartographer for the current stable stopgap

This means the current “it looks stable” Jetson state is:

- IMU online
- IMU not fused
- wheel odom off
- LiDAR-driven local SLAM input only

That state is a temporary stabilization point, not the final restore target.

## Stable LiDAR code that should currently be treated as the good base

The currently accepted LiDAR source base is:

- restored vendor snapshot:
  - `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/backups/Lslidar_ROS2_driver_2026-04-18_215404`
- current stable-input-safety snapshot:
  - `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/backups/Lslidar_ROS2_driver_stable_input_safety_2026-04-18_232420`

The local vendor tree now keeps that restored base plus only narrow input-safety
fixes:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/vendor/Lslidar_ROS2_driver/lslidar_driver/src/input.cc`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/vendor/Lslidar_ROS2_driver/lslidar_driver/src/lslidar_driver.cc`

The important runtime files around that LiDAR baseline are:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/scan_metadata_normalizer.py`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/cartographer_launch.py`

The intentionally preserved LiDAR-side fixes are narrow and low-level:

- reject too-short UDP packets correctly
- stop parsing stale packets after timeout/error
- clamp packet length to the real packet buffer size
- remove the bad `new[]` / `delete` / double-free path
- clamp scan index writes so out-of-range angle rounding does not write outside
  the scan array

The snapshot that should not be redeployed blindly is:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/backups/Lslidar_ROS2_driver_pre_restore_2026-04-18_223416`

That tree included geometry / timing experiments that correlated with a much
worse drifting state and was intentionally backed away from.

## What the current measurements actually imply

The current evidence says:

1. The original LiDAR path was not perfectly clean on Jetson even after the
   vendor restore.
2. The current IMU fusion path on Jetson still amplifies drift.
3. The current `no-IMU` stopgap is more stable than the current `IMU on`
   Jetson path.
4. That does not prove IMU should be removed forever.
5. It proves Jetson has not yet recreated the Pi5 formal
   `LiDAR + IMU + wheel odom` quality level.
