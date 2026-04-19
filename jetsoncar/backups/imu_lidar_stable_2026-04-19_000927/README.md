## IMU + LiDAR Stable Backup

Backup time: `2026-04-19 00:09:27` America/New_York

### What this backup is

This is the current Jetson `IMU + LiDAR` stable version that stopped the sudden catastrophic drift seen earlier in RViz.

The key fix in this version is:

- keep Cartographer on `robotcar_2d.lua`
- keep IMU enabled
- keep wheel odom disabled from fusion for now
- let `scan_metadata_normalizer.py` fix scan timing for Cartographer
- do **not** let the Cartographer path rewrite `angle_increment`

### Files included

- `start_full_native.sh`
- `scan_metadata_normalizer.py`
- `jetson_runtime_ctl.sh`
- `deploy_runtime_to_jetson.sh`
- `inspect_imu_bias.py`
- `inspect_stationary_drift.py`
- `robotcar_2d.lua`
- `cartographer_launch.py`
- `input.cc`
- `lslidar_driver.cc`
- `remote_process_snapshot.txt`
- `remote_logs_snapshot.txt`
- `remote_topics_snapshot.txt`

### Stable runtime shape

Current good runtime logic:

- LiDAR driver running
- HiPNUC IMU running on `/IMU_data`
- Cartographer config: `robotcar_2d.lua`
- scan topic into Cartographer: `/scan_slam`
- `scan_metadata_normalizer.py` parameters for Cartographer:
  - `adjust_angle_increment=false`
  - `prefer_stamp_delta_for_scan_time=true`
  - `derive_time_increment_from_scan_time=true`
- wheel odom not fused
- no RS485 dependency required for this version

### Why this version became stable

The earlier severe drift was not a normal slow drift. It looked like rare but catastrophic scan-time failures:

- stable for a while
- then a sudden huge jump
- long white trail in RViz

The main fix was to stop feeding Cartographer inconsistent LaserScan timing/subdivision data.

Important observed change after the fix:

- the earlier `Ignored subdivision ... previous subdivision time ...` warning disappeared
- IMU-enabled stationary tests became stable again

### Validation notes

Observed stationary results after the timing-only normalizer fix:

- 30s stationary:
  - about `x/y < 1 cm`
  - yaw about `0.06 deg`
- 120s stationary:
  - `x_span_m ~= 0.0096`
  - `y_span_m ~= 0.0103`
  - `yaw_span_deg ~= 0.076`

This is the version that was stable when the user reported that after 15 minutes it still stayed in place without drift.

### About wheel odom

Wheel odom is **not required** for the current scanning/stationary mapping version.

Why wheel odom existed in the original Pi5 formal chain:

- better short-term motion prior
- better forward-distance estimate
- more robust active navigation when the car is moving
- better behavior in repetitive corridor geometry

Why it is intentionally not fused right now:

- RS485/common-ground was still being fixed
- bad wheel odom can hurt fusion more than help it
- current `IMU + LiDAR` path is already stable enough for scanning/mapping

### If wheels slip or get blocked by a wall

Wheel odom can become wrong in exactly those cases:

- tire slip
- wheel spinning in place
- chassis blocked against a wall
- partial drag on one side

Why the original system could still look okay:

- Cartographer does not blindly trust odom forever
- LiDAR geometry can pull the pose back when scan matching is strong
- IMU helps rotational consistency

But if wheel odom quality is poor, it should only be reintroduced after:

- proper common ground
- clean RS485 replies
- stable odom direction/sign
- verified low-speed motion tests

### Recommended next restart target

When coming back, treat this backup as the reference baseline:

1. restore/deploy these files if needed
2. start the `IMU + LiDAR` stable path first
3. confirm no drift in stationary RViz
4. only then bring wheel odom back as an optional enhancement
