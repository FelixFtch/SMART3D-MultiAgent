# SMART3D Campus Runtime

Last updated: 2026-04-17

This is the now-working campus-network runtime chain for the `robotcar` Pi 5 and the i9 Alienware desktop.

For the cleaner "one file to hand to the next person" version, use:

- [SMART3D_CAMPUS_CONNECTION_GUIDE.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_CAMPUS_CONNECTION_GUIDE.md)

This is also the default path to reuse from now on.

Two rules matter:

- use the current campus IPs for this boot, not stale remembered ones
- the accepted end-to-end proof is `goal-test`

One important correction from the first draft:

- Pi -> i9 sensing and pose traffic is healthy
- i9 central planning is healthy
- but the i9 `validate` participant is **not** a good acceptance test for local planner topics like `/car1/path`

So the real planner acceptance test is now `goal-test`, not just `ros2 topic list`.

## Machines

- i9 Alienware
  - SSH during the recovery pass: `felix@10.66.112.64`
  - ROS interface: `wlp3s0`
- robotcar Pi 5
  - SSH during the recovery pass: `robotcar@10.66.38.108`
  - password: `234123`
  - ROS interface: `wlan0`
  - Docker container: `ros2-humble`

Those two IPs are examples from the successful recovery, not permanent addresses.

## What is confirmed working

- campus CycloneDDS data plane between i9 and robotcar Pi
- Pi detached sensor stack:
  - `start_full.sh`
  - `pi5_client.py`
  - `cmd_vel_driver.py`
- i9 detached runtime:
  - `i9_planner.py`
  - `rviz2`
- live Pi-origin ROS topics on i9:
  - `/map`
  - `/scan`
  - `/tf`
  - `/car1/pose`
- small motion test:
  - `odom` moved from near zero to about `x=0.02`
- real planner test:
  - i9 accepted `/goal_pose`
  - `i9_planner.py` built walls/bounds and returned a path
  - Pi received `/car1/path`
  - `pi5_client.py` logged `Path received`
  - planner logged `Build OK`

## The key local files

- [smart3d_campus.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh)
- [smart_real_robot.rviz](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/rviz/smart_real_robot.rviz)
- [importantfori9andpiconnect.md](/Users/felixfang/Documents/Playground/importantfori9andpiconnect.md)

## Why the old attempts were confusing

There were five separate traps:

1. `ros2 topic list` on i9 could give a false negative through the ROS2 CLI daemon.
   - Fix: always use `ROS2CLI_NO_DAEMON=1` for validation.

2. i9 fixed Cyclone participant ports can collide.
   - `planner` now uses participant index `31`
   - `rviz` now uses participant index `32`
   - validation CLI now uses participant index `33`
   - keeping them in a higher reserved band avoided stale port collisions during recovery

3. old detached startup could kill itself.
   - `pkill` and new launches were too close together in the same remote shell.
   - Fix: use a real detached launch with `setsid -f`, not a fragile foreground SSH chain.

4. the old RViz config was the wrong one for real hardware.
   - it mostly watched `/smart/*`
   - without an active goal, RViz could look blank even though `/map`, `/scan`, and `/tf` were healthy
   - Fix: use `smart_real_robot.rviz`

5. door and local wall changes could take too long to clear.
   - planner-side map rebuild detection was too insensitive for small local changes
   - Pi-side Cartographer `motion_filter` was also too slow while the robot was nearly stationary
   - Fix:
     - i9 `i9_planner.py`: `map_change_interval_sec=0.5`, `map_change_threshold=0.001`, `map_change_min_cells=6`
     - Pi `robotcar_2d_odom.lua`: `motion_filter.max_time_seconds=0.5`, `max_distance_meters=0.05`, `max_angle_radians=math.rad(0.5)`

## Canonical startup

Use the helper script:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh full-active
```

If the campus IPs changed since the last run, override the SSH endpoints for that boot:

```bash
I9_HOST='felix@<current_i9_campus_ip>' \
PI_HOST='robotcar@<current_pi_campus_ip>' \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh full-active
```

This handles:

- current campus IP detection from i9 `wlp3s0` and Pi `wlan0`
- campus Cyclone XML deployment
- Pi container restart
- Pi detached sensors
- Pi detached `pi5_client.py`
- passive i9 validation
- Pi detached `cmd_vel_driver.py`
- i9 detached planner
- i9 detached RViz
- tiny motion pulse

## Day-to-day commands

Full status:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh status
```

Start watchdogs without restarting the stack:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh watch-start
```

Watch the recent Pi-side diagnostics:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh watch-status
```

Tiny motion pulse:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh move-test
```

Stop active pieces:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh stop
```

Re-push XML and RViz config:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh prepare
```

`prepare` now re-detects the current campus IPs before writing the Cyclone peer XMLs.

End-to-end planner proof:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh goal-test
```

Optional explicit goal:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh goal-test 0.37 -0.16
```

## If RViz looks blank

Do not assume DDS is broken.

Check these first:

1. is RViz using `/home/felix/smart_rviz_bridge/smart_real_robot.rviz`
2. is `Fixed Frame` equal to `map`
3. do these topics exist on i9:
   - `/map`
   - `/scan`
   - `/tf`
   - `/car1/pose`
4. did anyone accidentally relaunch the old `smart_rviz.rviz`

Quick topic check on i9:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclone-campus-validate.xml
export ROS2CLI_NO_DAEMON=1

ros2 topic echo /map --once --qos-durability transient_local
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 topic echo /car1/pose --once
```

## Goal test that already worked

This kind of short goal is known-good:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclone-campus-validate.xml
export ROS2CLI_NO_DAEMON=1

ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 0.60, y: 0.00, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

Expected planner log shape:

- `Goal: (...)`
- `[build] from=... goal=... walls=...`
- `Build OK`
- maybe `GOAL REACHED` for a near target

Expected `goal-test` shape:

- Pi-side `/car1/path` dump contains real pose points, not just `poses: []`

## Watchdogs

There are now two Pi-side watchdogs for the “it drifted and glitched for five seconds” class of failure:

- i9 host watchdog log: `/tmp/i9_host_watchdog.log`
- i9 ROS watchdog log: `/tmp/i9_ros_watchdog.log`
- host watchdog log: `/tmp/pi_host_watchdog.log`
- ROS/container watchdog log: `/tmp/pi_ros_watchdog.log`

What they record:

- i9 host side:
  - CPU load
  - memory headroom
  - GPU utilization and temperature
  - `wlp3s0` wireless snapshot
  - planner/RViz process and UDP socket snapshots on spike events
- i9 ROS side:
  - `/scan` gap time from the i9 point of view
  - `/map_campus` age
  - `/car1/pose` gap time
  - `/tf` gap time
  - planner process disappearance
  - planner/RViz log tails on events
- host side:
  - CPU load
  - memory headroom
  - thermal state
  - Raspberry Pi throttling flags
  - `wlan0` link quality snapshot
  - host and container top CPU consumers on spike events
  - recent kernel messages on spike events
- ROS side:
  - `/scan` gap time
  - `/IMU_data` gap time
  - `/car1/pose` gap time
  - `/map` age
  - load and temperature
  - top processes plus recent Cartographer/LiDAR/client logs when a gap is detected

This is there specifically to prove whether the failure moment was:

- i9 receive-side DDS stall
- i9 planner/RViz self-stall
- LiDAR starvation
- Pi CPU overload
- thermal throttling
- wireless instability
- or some ugly combination of the above
- `pi5_client.py` shows `Goal forwarded: ...`
- planner tail shows `[build] ...` and `Build OK`

## The important process split

i9:

- `i9_planner.py` owns participant `31`
- `rviz2` owns participant `32`
- validation CLI should use participant `33`

Pi:

- `start_full.sh`
- `pi5_client.py`
- `cmd_vel_driver.py`
- all use the shared campus Cyclone file in the container

## Faster wall refresh

If a door opens and RViz still shows that doorway blocked for too long, there are two layers now involved:

1. Planner rebuild sensitivity on i9
   - now checks map change every `0.5s`
   - now rebuilds on much smaller local occupancy changes

2. Cartographer scan acceptance on Pi
   - the old config allowed up to `3.0s` before accepting a fresh scan while nearly stationary
   - the current config cuts that down to `0.5s`

This means the map can still only clear after the LiDAR actually sees through the opened doorway, but it should no longer wait on the old multi-second filter before reacting.

## The most useful logs

i9:

- `/tmp/i9_planner_campus.log`
- `/tmp/rviz_campus.log`

Pi:

- `/tmp/start_full_campus.log`
- `/tmp/pi5_client_campus.log`
- `/tmp/cmd_vel_driver_campus.log`
- `/tmp/odom.log`

## Bottom line

The campus chain is no longer just "partly visible". It is a real closed loop:

- map and LiDAR arrive on i9
- planner receives pose and goals
- planner builds a path on i9
- `/car1/path` is received on the Pi side
- commands go back down to the Pi
- the base moves
- RViz can show the real robot map and scan

What this baseline does **not** depend on:

- the old USB/AP transport
- `ros2 topic list` as proof
- the temporary Pi-side visualization relay workaround
