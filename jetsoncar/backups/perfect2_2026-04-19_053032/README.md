## Perfect2 Backup

Timestamp: `2026-04-19 05:30:32` America/New_York

This is the final approved `perfect2` snapshot for the current robotcar round.

It is intentionally small. It captures the three files that mattered most at
the end:

- `i9_planner.py`
- `pi5_client.py`
- `smart_real_robot.rviz`

## What this backup means

This is not a whole-runtime archive.
It is the approved planner + follower + RViz display trio that sat on top of
the later-stable Jetson runtime.

At the time this backup was created, the accepted runtime shape was:

- Jetson passive:
  - LiDAR
  - IMU
  - `scan_metadata_normalizer.py`
  - Cartographer with `robotcar_2d.lua`
  - `campus_map_bridge.py`
  - `robot_state_publisher`
- Jetson active:
  - `cmd_vel_driver_pi5_reference.py`
  - `pi5_client.py`
- i9:
  - `i9_planner.py`
  - `rviz2` with `smart_real_robot.rviz`

## Why this version was preserved

This snapshot preserves the end-state after the last ugly follower/display
regressions were backed out.

What it keeps:

- static wall-triggered repair through planner-side `update_static`
- separate display path topic for RViz
- display path refresh so walked segments visually disappear
- reverted follower logic without the rejected stale-path holding behavior

What it deliberately does not include:

- the rejected stale-path hold experiment
- blind "wait until the path disappears" behavior

## Important details in this snapshot

### `i9_planner.py`

Key behaviors present here:

- `fresh_walls` support
- `do_static_update()` calling planner-side `update_static`
- `/car1/path_display`
- `publish_empty_path()`
- `publish_display_path()`

Default planner parameters inside the snapshot:

- `map_change_interval_sec = 2.0`
- `map_change_threshold = 0.03`
- `map_change_min_cells = 20`

At runtime, the active wrapper later overrode those values more aggressively:

- `map_change_interval_sec = 0.8`
- `map_change_threshold = 0.03`
- `map_change_min_cells = 30`

### `pi5_client.py`

Key truth:

- this is the reverted good follower
- it still uses anchor-jump limits
- it does not contain the rejected stale-path hold logic

Important parameters present here:

- `max_anchor_jump_points = 2`
- `max_anchor_jump_dist = 1.25`

### `smart_real_robot.rviz`

This RViz config is expected to subscribe to:

- `/car1/path_display`

not the raw control path topic.

## How to use this backup

Use this backup when:

- a later change makes the robot wait stupidly for stale paths
- the green display path stops behaving properly
- someone accidentally edits the planner/follower pair into a worse state

The intended restore pattern is:

1. copy `i9_planner.py` back to the active source / deploy location
2. copy `pi5_client.py` back to the active source / deploy location
3. copy `smart_real_robot.rviz` back to the active RViz config location
4. redeploy
5. fully stop and relaunch the stack

## Limitations still present in Perfect2

This version is good, not magic.

Known remaining imperfections:

- wall corners can still become stable walls later than the operator wants
- local turning can still look uglier than the global map shape
- wheel odom is still not the unquestioned baseline
- camera load can still worsen SLAM timing if re-enabled during long runs
