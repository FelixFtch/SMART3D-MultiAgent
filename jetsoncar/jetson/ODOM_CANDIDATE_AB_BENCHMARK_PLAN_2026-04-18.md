# Jetson 2D Laser Odometry A/B Benchmark Plan

Last updated: 2026-04-18

This note is the practical A/B benchmark design for two scan-based odometry
candidates on the Smart3D Jetson robot:

1. `rf2o_laser_odometry`
2. `ros2_laser_scan_matcher`

The goal is not academic benchmarking. The goal is to decide which candidate
is more usable in the real Smart3D chain on this exact robot:

- Jetson Orin NX 16GB
- M10P 2D LiDAR publishing `/scan`
- HiPNUC IMU publishing `/IMU_data`
- current SLAM backends: Cartographer and slam_toolbox
- current planner/i9 contract: `/map_campus`, `/car1/pose`, `/goal_pose`

This plan is written to be scriptable and to run without the user physically
present.

## Official package facts that matter

These are the source-of-truth behavior contracts to benchmark against:

- `rf2o_laser_odometry` is for "Estimation of 2D odometry based on planar
  laser scans" and the Humble launch file exposes:
  - `laser_scan_topic`
  - `odom_topic`
  - `publish_tf`
  - `base_frame_id`
  - `odom_frame_id`
  - `freq`
- `ros2_laser_scan_matcher` currently subscribes to:
  - `/scan`
  - `/tf`
  and publishes:
  - `/tf` with `odom -> base_link`
  - optional `/odom`
  The upstream README also states IMU support and odometry support are "will be
  released features", meaning they are not the current mainline behavior.
- `slam_toolbox` requires:
  - `/scan`
  - a valid transform from configured `odom_frame` to `base_frame`
- `cartographer_ros` can run without odometry, but when `use_odometry = true`
  it subscribes to `nav_msgs/Odometry` on `odom`.

Implication for this robot:

- both candidates should be treated as **scan-only odometry** in the first A/B
  round
- `/IMU_data` should still be recorded during every run, but neither candidate
  should be credited for consuming it unless that is explicitly added and
  verified later

## What "good" looks like on this robot

The winning candidate is not just the one with lower CPU. It must satisfy all
three:

1. it publishes a stable dynamic `odom -> base_link`
2. it materially improves the SLAM backends over the current static/no-odom
   placeholder
3. it does not break the existing Smart3D campus contract

The campus contract is still:

- Jetson local: `/scan`, `/map`, `/map_campus`, `/car1/pose`
- i9 remote: can receive `/scan`, `/map_campus`, `/car1/pose`
- active chain still supports `goal-test`

## High-level benchmark structure

Run each candidate in three phases.

### Phase A: candidate-only stability

Purpose:

- confirm the odom node itself stays alive
- measure resource cost
- measure zero-motion drift/jitter

Motion:

- no floor motion required
- robot can remain parked

Outputs required:

- odom topic from candidate
- optionally candidate TF

### Phase B: candidate + Cartographer

Purpose:

- test whether the candidate improves or destabilizes Cartographer
- measure the correction Cartographer has to apply on top of candidate odom

Motion:

- preferred: unattended low-speed symmetric motion script
- fallback: stationary-only if motion is unsafe

Outputs required:

- `/scan`
- `/map`
- `/map_campus`
- `/car1/pose`
- candidate `/odom`
- dynamic `odom -> base_link`

### Phase C: candidate + slam_toolbox

Purpose:

- verify the candidate is good enough to satisfy slam_toolbox's real odom
  requirement
- compare map stability and odom dependency behavior against Cartographer

Motion:

- same script as Phase B

Outputs required:

- `/scan_slam`
- `/map`
- `/map_campus`
- `/car1/pose`
- candidate `/odom`
- dynamic `odom -> base_link`

## Required preflight gates

Do not start any candidate run unless all preflight gates pass.

### Safety gates

- `/stp23l_hard_brake_active` is `false`
- `/front_range` is publishing
- `/front_range` stays above `0.35 m` for at least 10 seconds before motion
- `cmd_vel_driver.py` is the only RS485 owner during motion tests
- no planner-driven motion is active while odom is being benchmarked

### ROS gates

- `/scan` publisher count >= 1
- `/IMU_data` publisher count >= 1
- no stale static `odom -> base_link` TF if candidate is expected to publish a
  dynamic one
- no `ddsm115_odom.py` process during candidate odom compatibility tests

### Runtime gates

- `lslidar_driver_node` alive
- `pi5_client.py` alive
- `campus_map_bridge.py` alive
- no repeated LiDAR crash signatures in `/tmp/lidar.log`

## Candidate launch contract

The benchmark should use explicit candidate-specific topics first, then switch
to canonical `/odom` only in the compatibility phases.

### rf2o candidate-only

Use a dedicated topic first:

- `laser_scan_topic:=/scan`
- `odom_topic:=/odom_rf2o`
- `publish_tf:=false`
- `base_frame_id:=base_link`
- `odom_frame_id:=odom_rf2o`

Reason:

- avoids contaminating the live TF tree
- lets us collect zero-motion statistics safely

### ros2_laser_scan_matcher candidate-only

Use:

- `publish_odom:=/odom_lsm`
- `publish_tf:=false`

Reason:

- same isolation as above

### Compatibility-mode launch

For backend compatibility passes, the chosen candidate must become the sole odom
source:

- odom topic: `/odom`
- TF: `odom -> base_link`
- `ddsm115_odom.py` off
- static `odom -> base_link` off
- `odom_tf_relay.py` off unless the candidate publishes topic-only odom and no
  TF

## Existing local files to reuse

These already exist and should be reused rather than inventing a parallel test
stack:

- `jetson/jetson_runtime_ctl.sh`
- `jetson/start_full_native.sh`
- `jetson/compare_slam_backends.sh`
- `scripts/smart3d_jetson_campus.sh`
- `jetson/mapper_params_online_async.yaml`

Current local expectations:

- slam_toolbox params already use:
  - `odom_frame: odom`
  - `map_frame: map`
  - `base_frame: base_link`
  - `scan_topic: /scan_slam`
- current remote Jetson Cartographer configs include:
  - `robotcar_2d.lua` with `use_odometry = false`
  - `robotcar_2d_odom.lua` with `use_odometry = true`

That means the compatibility-mode switches are straightforward:

- Cartographer test:
  - use `robotcar_2d_odom.lua`
  - candidate must publish `/odom`
- slam_toolbox test:
  - candidate must publish dynamic `odom -> base_link`
  - current static odom TF must be disabled

## Metrics to capture every run

Every run should emit one machine-readable JSON and one human-readable summary.

### Process resource metrics

Capture at 2-second intervals:

- candidate avg CPU %
- candidate max CPU %
- candidate avg RSS MB
- candidate max RSS MB
- backend avg CPU %
- backend max CPU %
- backend avg RSS MB
- backend max RSS MB
- process alive sample count

Suggested groups:

- `rf2o`
- `laser_scan_matcher`
- `cartographer`
- `slam_toolbox`
- `scan_normalizer`
- `map_bridge`
- `pi5_client`
- `watchdog`

### Topic-rate and liveness metrics

Count messages over a 6-second and 30-second window for:

- `/scan`
- `/IMU_data`
- `/front_range`
- candidate odom topic
- `/map`
- `/map_campus`
- `/car1/pose`
- `/tf`

Also capture publisher/subscriber counts for:

- `/scan`
- candidate odom topic
- `/map`
- `/map_campus`
- `/car1/pose`

### TF metrics

For `odom -> base_link` and `map -> odom`:

- availability
- average rate
- latest transform age
- final transform
- max translation magnitude during run
- max yaw magnitude during run

## Drift proxies without ground truth

There is no trusted external ground truth on this robot. Use the following
proxies.

### Proxy 1: zero-motion odom jitter

Run stationary for 120 seconds with no commanded motion.

Measure from candidate odom:

- stddev of `x`
- stddev of `y`
- stddev of yaw
- peak-to-peak `x`
- peak-to-peak `y`
- peak-to-peak yaw

Pass target:

- translational jitter should remain small and bounded
- no monotonic drift trend over the full stationary window

This is the safest unattended baseline and should always run.

### Proxy 2: return-to-start error after a symmetric motion script

Run a small, low-speed, symmetric script:

1. forward pulse
2. stop
3. reverse pulse of equal duration
4. stop
5. rotate left pulse
6. stop
7. rotate right pulse of equal duration
8. stop

Suggested conservative commands:

- linear pulse: `0.03 m/s` for `4 s`
- angular pulse: `0.10 rad/s` for `3 s`
- `2 s` stop between pulses

Why this script:

- safe for unattended operation
- uses tiny motion
- should roughly return to the start if control symmetry is acceptable

Measure from candidate odom:

- final minus initial `x`
- final minus initial `y`
- final minus initial yaw

Interpretation:

- this is **not** true physical error
- it is a self-consistency proxy
- lower residual after a symmetric command script is better

### Proxy 3: SLAM correction load

This is the most useful proxy for real Smart3D use.

For both backends, measure how much global SLAM has to correct the candidate
odom by tracking `map -> odom`.

Metrics:

- RMS translation magnitude of `map -> odom`
- final translation magnitude of `map -> odom`
- RMS yaw of `map -> odom`
- final yaw of `map -> odom`
- step jumps in `map -> odom` after motion

Interpretation:

- smaller, smoother `map -> odom` correction means the candidate odom is doing
  more of the local pose work correctly
- large discontinuities suggest scan matcher instability, bad timestamps, or
  frame misuse

## Backend-specific compatibility criteria

### Cartographer compatibility

Run with:

- candidate providing `/odom`
- candidate providing dynamic `odom -> base_link`
- Cartographer config `robotcar_2d_odom.lua`

Judge compatibility using:

- `/map` alive
- `/map_campus` alive
- `/car1/pose` alive
- no Cartographer crash
- no repeated fatal/extrapolation errors in `/tmp/cartographer.log`
- candidate odom remains alive through the full run
- `goal-test` still passes after the benchmark run

Cartographer-specific positive signal:

- compared with the current no-odom baseline, `map -> odom` correction becomes
  smaller or smoother without reducing `/car1/pose` continuity

Cartographer-specific failure signals:

- `/map` stalls
- `/car1/pose` gaps increase
- repeated "Extrapolator" or transform timeout failures
- map becomes visibly torn or rotates unexpectedly in RViz

### slam_toolbox compatibility

Run with:

- candidate providing dynamic `odom -> base_link`
- static odom TF disabled
- `ddsm115_odom.py` disabled

Judge compatibility using:

- `/map` alive
- `/map_campus` alive
- `/car1/pose` alive
- no repeated `Failed to compute odom pose`
- no persistent message-filter drops caused by stale TF
- candidate odom remains alive through the full run
- `goal-test` still passes after the benchmark run

slam_toolbox-specific positive signal:

- map builds without relying on fake static odom
- `map -> odom` remains bounded and smooth over the motion script

slam_toolbox-specific failure signals:

- `Failed to compute odom pose`
- map stops updating while `/scan` continues
- LaserScan drops because transforms are too old or unavailable
- map rotates with the robot

## Pass/fail rubric

Each candidate gets one scorecard per backend.

### Hard fail

Any of the following is an immediate fail:

- candidate process crashes
- `odom -> base_link` disappears for more than 1 second during active test
- `/map` or `/car1/pose` dies during compatibility run
- `goal-test` fails after integrating the candidate
- hard-brake test no longer works after the candidate run

### Soft fail

Soft fail means "works, but do not merge into mainline":

- CPU materially higher than current baseline without better correction load
- stationary drift is obviously monotonic
- slam_toolbox only works with fake static odom
- Cartographer correction load is worse than the no-odom baseline
- repeated warning storms in backend logs

### Pass

Declare a candidate pass only if all are true:

- survives 120-second stationary run
- survives one symmetric unattended motion run
- passes Cartographer compatibility run
- passes slam_toolbox compatibility run
- keeps `goal-test` working
- does not regress hard-brake behavior

## What to record to rosbag

For each run, record:

- `/scan`
- `/IMU_data`
- `/front_range`
- candidate odom topic
- `/tf`
- `/tf_static`
- `/map`
- `/map_campus`
- `/car1/pose`
- `/cmd_vel`
- `/goal_pose`
- `/stp23l_hard_brake_active`

Even though neither odom candidate currently uses `/IMU_data`, record it so the
same bag can later be replayed for:

- `robot_localization`
- timestamp debugging
- scan/IMU correlation inspection

## Recommended run matrix

For each candidate:

1. stationary candidate-only run
2. symmetric-motion candidate-only run
3. stationary + Cartographer run
4. symmetric-motion + Cartographer run
5. stationary + slam_toolbox run
6. symmetric-motion + slam_toolbox run
7. post-run `goal-test`
8. post-run `estop-test`

This is intentionally a little repetitive. The robot has already shown startup
and driver fragility, so a single lucky run is not enough.

## Practical recommendation for this project

Start with this order:

1. `rf2o_laser_odometry`
2. `ros2_laser_scan_matcher`

Reason:

- `rf2o` is explicitly positioned for planar laser odometry on robots with bad
  base odometry
- `ros2_laser_scan_matcher` is useful, but its current upstream README still
  lists IMU and odometry support as future work, so it should be treated as the
  simpler fallback candidate rather than the primary bet

## Minimal JSON fields per run

The benchmark script should at least emit:

```json
{
  "candidate": "rf2o",
  "backend": "cartographer",
  "duration_sec": 120,
  "motion_profile": "stationary",
  "preflight": {
    "front_range_ok": true,
    "hard_brake_clear": true,
    "scan_alive": true,
    "imu_alive": true
  },
  "process_metrics": {},
  "topic_counts": {},
  "tf_metrics": {},
  "drift_proxy": {
    "stationary_jitter": {},
    "return_to_start_error": {},
    "map_to_odom_correction": {}
  },
  "logs": {
    "candidate": [],
    "backend": [],
    "lidar": []
  },
  "verdict": "pass"
}
```

## Sources

- `rf2o_laser_odometry` upstream repo:
  https://github.com/Adlink-ROS/rf2o_laser_odometry
- `rf2o_laser_odometry` Humble launch contract:
  https://raw.githubusercontent.com/Adlink-ROS/rf2o_laser_odometry/humble-devel/launch/rf2o_laser_odometry.launch.py
- `ros2_laser_scan_matcher` upstream repo:
  https://github.com/AlexKaravaev/ros2_laser_scan_matcher
- `ros2_laser_scan_matcher` README:
  https://raw.githubusercontent.com/AlexKaravaev/ros2_laser_scan_matcher/main/README.md
- `slam_toolbox` Humble README:
  https://docs.ros.org/en/humble/p/slam_toolbox/__README.html
- `cartographer_ros` configuration docs:
  https://google-cartographer-ros.readthedocs.io/en/stable/configuration.html
