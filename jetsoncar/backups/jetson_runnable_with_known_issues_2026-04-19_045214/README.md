## Jetson runnable backup with known issues

Timestamp: `2026-04-19 04:52:14` America/New_York

This backup freezes the first Jetson campus version that:

- restores LiDAR orientation correctly
- restores RViz map rendering
- brings back full passive + active SMART3D chain
- can run end to end with real map, planner, RViz, and base control

It is intentionally kept even though it still has known flaws.

### Main launch shape at backup time

Passive stack:

- LiDAR
- IMU
- `scan_metadata_normalizer.py`
- Cartographer with `robotcar_2d.lua`
- campus map bridge
- robot state publisher

Active stack:

- `cmd_vel_driver_pi5_reference.py`
- `pi5_client.py`
- i9 `i9_planner.py`
- i9 `rviz2`

Planner thresholds in the active run:

- `map_change_interval_sec = 1.2`
- `map_change_threshold = 0.05`
- `map_change_min_cells = 60`

Emergency stop distance in the active run:

- `0.05 m`

### Known issues preserved in this backup

- Corner updates can still feel slow, especially at wall corners during turns.
- `pi5_client.py` can still chase stale or partly behind paths.
- Local behavior is much better than the earlier catastrophic drift builds, but not yet fully polished.
- Dynamic obstacle detection can still produce false positives.

### Important contents

- `playground/`: local wrapper and Jetson deploy scripts
- `private_repo/`: planner and follower source from the main SMART3D repo
- `i9/`: current RViz config and planner log
- `jetson/jetson_runtime_snapshot/`: full Jetson runtime directory snapshot
- `jetson/cartographer.log`: mapping log snapshot
- `jetson/pi5_client_campus.log`: follower log snapshot

### Why keep this backup

This version is a useful rollback point because it is already operational on real hardware, even though it still needs refinement.
