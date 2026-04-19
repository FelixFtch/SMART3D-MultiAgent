## Jetson Perfect1 Backup

Timestamp: `2026-04-19 04:55:43` America/New_York

This backup freezes the user-approved `完美1` version: the first current Jetson campus run that feels clearly better in practice and is worth preserving before further changes.

### What was running when this backup was made

Jetson passive stack:

- LiDAR driver
- IMU publisher
- `scan_metadata_normalizer.py`
- Cartographer with `robotcar_2d.lua`
- campus map bridge
- robot state publisher

Jetson active stack:

- `cmd_vel_driver_pi5_reference.py`
- `pi5_client.py`

i9 side:

- `i9_planner.py`
- `rviz2`

### Active planner parameters in this backup

- `robot_speed = 0.24`
- `map_change_interval_sec = 1.2`
- `map_change_threshold = 0.05`
- `map_change_min_cells = 60`
- `emergency_stop_dist = 0.05`

### Why this version was preserved

- It restores the full passive + active SMART3D chain.
- RViz map rendering is back.
- LiDAR orientation is correct.
- Catastrophic long-distance SLAM explosions were not the dominant symptom in this run.
- It feels substantially better than the earlier unstable versions.

### Known imperfections still present

- Corner updates can still lag, especially around wall corners while turning.
- `pi5_client.py` can still receive stale paths whose first point is behind the car.
- Dynamic obstacle detection can still produce false positives.
- Full rebuilds still happen more often than ideal.

### Contents

- `playground/`: local wrappers and deploy scripts
- `private_repo/`: planner and follower source snapshot
- `i9/`: RViz config, planner log, and process snapshot
- `jetson/`: runtime snapshot, key logs, and process snapshot
