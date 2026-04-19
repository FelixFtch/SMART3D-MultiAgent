# Jetson 2D Laser Odometry A/B Results

Last updated: 2026-04-18

This note records the real on-robot A/B results for the two scan-based odometry
candidates installed on the Jetson Orin NX 16GB Smart3D runtime:

1. `rf2o_laser_odometry`
2. `ros2_laser_scan_matcher`

This is not a paper benchmark. It is the practical result on this exact robot
and this exact ROS graph.

## Test context

Robot and runtime:

- Jetson Orin NX 16GB
- ROS 2 Humble
- campus DDS domain: `42`
- M10P 2D LiDAR on `/scan`
- HiPNUC IMU on `/IMU_data`
- STP23L range on `/front_range`
- i9 reachable and confirmed to receive `/map_campus`

Preconditions used in this A/B:

- Jetson runtime was started in sensors-only mode first
- `ddsm115_odom.py` stayed off during odom candidate tests
- static fake `odom -> base_link` stayed off
- LiDAR was started via `lidar_supervisor.sh`, not bare `ros2 run`

## New runtime assets added in this round

These files now exist and are the correct entrypoints for the next session:

- `jetson/lidar_supervisor.sh`
- `jetson/rf2o_odom_params.yaml`
- `jetson/rf2o_odom_for_ekf_params.yaml`
- `jetson/ekf_rf2o_imu.yaml`
- `jetson/laser_scan_matcher_params.yaml`
- `jetson/start_rf2o_odom.sh`
- `jetson/start_rf2o_ekf_imu.sh`
- `jetson/start_laser_scan_matcher_odom.sh`
- `jetson/start_kinematic_icp.sh`
- `jetson/measure_odom_candidate.py`

Control entrypoints:

- `jetson/jetson_runtime_ctl.sh start-sensors`
- `jetson/jetson_runtime_ctl.sh start-rf2o`
- `jetson/jetson_runtime_ctl.sh start-rf2o-ekf`
- `jetson/jetson_runtime_ctl.sh start-laser-scan-matcher`
- `jetson/jetson_runtime_ctl.sh start-kinematic-icp`
- `jetson/jetson_runtime_ctl.sh measure-odom rf2o 60`
- `jetson/jetson_runtime_ctl.sh measure-odom rf2o_ekf 60`
- `jetson/jetson_runtime_ctl.sh measure-odom laser_scan_matcher 60`

## LiDAR hardening result

Before the odom A/B, the Jetson runtime was hardened in two ways:

1. `start_full_native.sh` no longer starts LiDAR as a naked `ros2 run`
2. startup now blocks on real `/scan` readiness before continuing

This matters because the LiDAR driver has already shown heap-corruption-style
failures on this stack (`malloc(): unaligned tcache chunk detected`), and the
supervisor gives the runtime a local self-heal path.

Observed proof after hardening:

- sensors-only startup succeeded
- local probe showed `/scan msgs=60`
- 6-second healthcheck showed:
  - `/scan`: `72`
  - `/IMU_data`: `601`
  - `/front_range`: `60`

## Candidate-only static results

Method:

- candidate became the only `/odom` source
- robot stayed stationary
- `/odom` was sampled for 60 seconds
- candidate process CPU and RSS were sampled during the same window

### rf2o

Command path used:

- `jetson_runtime_ctl.sh start-rf2o`
- `jetson_runtime_ctl.sh measure-odom rf2o 60`

Observed result:

- message count: `601`
- avg odom rate: `10.01 Hz`
- final translation drift: `0.013088 m`
- final yaw drift: `0.029057 rad`
- max translation from start: `0.016394 m`
- x span: `0.015661 m`
- y span: `0.009231 m`
- yaw span: `0.048026 rad`
- avg CPU: `45.32%`
- max CPU: `47.0%`
- avg RSS: `43.67 MB`

Interpretation:

- rf2o keeps up with the LiDAR much better
- rf2o is materially cheaper than laser_scan_matcher
- on this stationary test, rf2o drifts more than laser_scan_matcher

### ros2_laser_scan_matcher

Command path used:

- `jetson_runtime_ctl.sh start-laser-scan-matcher`
- `jetson_runtime_ctl.sh measure-odom laser_scan_matcher 60`

Observed result:

- message count: `376`
- avg odom rate: `6.28 Hz`
- final translation drift: `0.00105 m`
- final yaw drift: `-0.000106 rad`
- max translation from start: `0.002316 m`
- x span: `0.002848 m`
- y span: `0.002298 m`
- yaw span: `0.003914 rad`
- avg CPU: `98.67%`
- max CPU: `101.0%`
- avg RSS: `56.03 MB`

Interpretation:

- laser_scan_matcher is much quieter on a parked robot
- but it is effectively saturating one CPU core
- and it only produced about `6.3 Hz` odom while `/scan` was near `12 Hz`

## Backend compatibility smoke results

This section answers the real question: not just "does `/odom` exist?" but
"does Smart3D mapping still work with it?"

### rf2o + slam_toolbox

Result: **works**

Observed:

- `/map` appeared
- `/map_campus` appeared
- slam_toolbox registered the lidar
- queue-full warnings still appeared

Practical reading:

- rf2o is good enough to satisfy slam_toolbox's odom requirement on this robot
- but the scan path is still not perfectly relaxed; there is remaining queue
  pressure

### laser_scan_matcher + slam_toolbox

Result: **not healthy enough right now**

Observed:

- no `/map` within the smoke-test window
- no `/map_campus` within the smoke-test window
- repeated slam_toolbox message-filter queue-full warnings

Practical reading:

- this combination is not ready to be the next-session default
- the low odom rate and heavy CPU cost are the main suspects

### rf2o + cartographer

Result: **works**

Observed:

- `/map` appeared
- `/map_campus` appeared
- `/car1/pose` appeared
- cartographer logged:
  - `imu rate: 94.15 Hz`
  - `odom rate: 10.03 Hz`
  - `scan rate: 11.99 Hz`
- cartographer also logged many `Dropped earlier points`

Practical reading:

- rf2o is currently the cleanest external-scan-odom candidate for cartographer
- it preserves a full-rate odom stream
- but the scan/processing path is still not fully relaxed

### rf2o + robot_localization + IMU

Result: **works and is now the preferred odom contract**

Command path used:

- `jetson_runtime_ctl.sh start-sensors`
- `jetson_runtime_ctl.sh start-rf2o-ekf`
- `jetson_runtime_ctl.sh measure-odom rf2o_ekf 60`

Observed result:

- output topic: `/odom`
- message count: `1801`
- avg odom rate: `30.02 Hz`
- final translation drift: `0.011554 m`
- final yaw drift: `-0.000004 rad`
- max translation from start: `0.015213 m`
- x span: `0.018421 m`
- y span: `0.000009 m`
- yaw span: `0.001597 rad`
- avg CPU: `6.28%`
- max CPU: `7.2%`
- avg RSS: `44.6 MB`

Interpretation:

- the fusion stage is very cheap
- it preserves a much cleaner yaw output than raw rf2o on this parked test
- and it promotes the odom stream to a stable `30 Hz` contract for the rest of
  the stack

### rf2o + robot_localization + slam_toolbox

Result: **works and closes the Smart3D loop**

Observed:

- `/map` appeared
- `/map_campus` appeared
- `/car1/pose` appeared
- i9 saw `/map_campus`, `/car1/pose`, `/scan`, `/front_range`, `/odom`
- `goal-test` passed with non-empty `/car1/path`
- `pi5_client.py` logged `Goal forwarded` and `Path received: 2 pts`

Practical reading:

- this is now a real, re-validated changed-network path, not just a local
  smoke result
- on a short live process snapshot, this path looked lighter than cartographer
- queue-full warnings still exist, so there is tuning headroom left

### rf2o + robot_localization + cartographer

Result: **works and closes the Smart3D loop**

Observed:

- `/map` appeared
- `/map_campus` appeared
- `/car1/pose` appeared
- `goal-test` passed with non-empty `/car1/path`
- planner logs stayed healthy from i9

Practical reading:

- cartographer still works with the fused odom path
- but short live process snapshots showed it heavier than slam_toolbox on this
  Jetson
- cartographer also continued to report dropped earlier points

### laser_scan_matcher + cartographer

Result: **works, but second-best**

Observed:

- `/map` appeared
- `/map_campus` appeared
- `/car1/pose` appeared
- cartographer logged:
  - `imu rate: 94.12 Hz`
  - `odom rate: 6.21 Hz`
  - `scan rate: 12.00 Hz`
- more aggressive `Dropped earlier points` behavior was visible

Practical reading:

- cartographer can still function with laser_scan_matcher
- but the odom stream is too slow and too expensive to call this the preferred
  runtime choice

## Summary table

| Candidate | Static Drift | CPU | Odom Rate | slam_toolbox | Cartographer |
|---|---:|---:|---:|---|---|
| `rf2o_laser_odometry` | worse | better | ~10 Hz | works | works |
| `ros2_laser_scan_matcher` | better | worse | ~6 Hz | weak / no map in smoke window | works, but weaker |

## Current recommendation

If the next session wants one candidate to continue with first, use:

- `rf2o_laser_odometry + robot_localization + /IMU_data`

Reason:

- it kept a full-rate odom stream
- the fusion stage itself was cheap
- it worked with both slam_toolbox and cartographer
- it did not block the existing `/map_campus` contract to i9

`ros2_laser_scan_matcher` is still worth keeping around as a reference because:

- its stationary drift was much lower
- it may still improve after scan throttling or queue tuning

But it is not the best mainline choice today.

## Most relevant future candidate to inspect

The most interesting newer upstream candidate for this robot is not FAST-LIO or
KISS-ICP first. It is:

- `PRBonn/kinematic-icp`

Why this one matters:

- it explicitly targets wheeled mobile robots
- it supports ROS 2 Humble
- it explicitly supports `LaserScan` via `use_2d_lidar:=true`
- upstream GitHub showed release `v0.1.1` on `2025-01-09`

Important constraint from upstream:

- it requires an existing odometry source and `odom -> base_link` transform as
  an input prior
- so it is not a drop-in replacement for the current robot unless the wheel
  odom / prior-TF story is made trustworthy first

## Kinematic-ICP smoke on the Jetson

This section is a software-integration smoke only. It is not yet a valid
algorithm benchmark for this robot.

What was done on 2026-04-18:

- built upstream `PRBonn/kinematic-icp` on the Jetson in:
  - `/home/jetson/SMART3D/ws_kinematic_icp`
- launched it in 2D mode with:
  - `lidar_topic:=/scan`
  - `use_2d_lidar:=true`
  - `wheel_odom_frame:=odom`
  - `base_frame:=base_link`
  - `lidar_odom_frame:=odom_kinematic`
  - `lidar_odometry_topic:=/lidar_odometry_kinematic`

Observed result:

- Kinematic-ICP initialized successfully
- `/lidar_odometry_kinematic` was live
- `tf2_echo odom_kinematic base_link` resolved successfully

Most important finding:

- with default launch behavior it spammed:
  - `Lookup would require extrapolation into the future`
- the current robot prior TF is just late enough that a zero TF timeout is too
  strict
- explicitly setting `tf_timeout:=0.2` fixed startup for this smoke

Why this smoke is not yet a real benchmark:

- the prior used here was `rf2o + robot_localization`
- that is good enough to prove software integration
- but it is circular for deciding whether Kinematic-ICP is better than the
  current scan-odom mainline

Practical conclusion:

- Kinematic-ICP is worth keeping in the repo and on the Jetson
- but the next important engineering step is to clean up and validate the
  single-owner prior `odom -> base_link` contract before trying to score it as a
  serious odometry replacement

For this Smart3D robot, that makes it a more relevant "next modern odom"
candidate than the usual 3D point-cloud LIO stacks.

Other strong open-source odometry projects do exist, but they are not direct
fits here:

- `KISS-ICP`: excellent and modern, but primarily a point-cloud pipeline
- `STEAM-ICP`: powerful continuous-time lidar / lidar-inertial odometry, but
  much heavier and not a drop-in 2D `LaserScan` replacement
- `FAST-LIO` / `Faster-LIO`: strong 3D lidar-inertial systems, not a natural
  match for the current M10P 2D laser chain

## Sources

- [rf2o_laser_odometry](https://github.com/MAPIRlab/rf2o_laser_odometry)
- [ros2_laser_scan_matcher](https://github.com/AlexKaravaev/ros2_laser_scan_matcher)
- [Kinematic-ICP](https://github.com/PRBonn/kinematic-icp)
- [KISS-ICP](https://github.com/PRBonn/kiss-icp)
- [STEAM-ICP](https://github.com/utiasASRL/steam_icp)
