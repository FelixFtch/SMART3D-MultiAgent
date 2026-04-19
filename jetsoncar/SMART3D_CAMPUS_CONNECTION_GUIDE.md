# SMART3D Campus Connection Guide

Last updated: 2026-04-17

This is the single-guide version of the SMART3D `i9 Alienware <-> robotcar Pi 5` recovery.

Use this as the default runbook from now on.

## What is the default now

The default is now the **campus Wi-Fi** path:

- i9 uses `wlp3s0`
- Pi uses `wlan0`
- `ROS_DOMAIN_ID=42`
- DDS implementation: `rmw_cyclonedds_cpp`

Do **not** treat the old USB/AP path as the baseline anymore.
Keep it only as rollback history.

## The most important truths

1. Campus IPs are dynamic.
2. `ros2 topic list` is not proof.
3. The accepted end-to-end proof is `goal-test`.
4. Startup order matters.
5. RViz being blank does not automatically mean DDS is broken.
6. Door opening latency is partly a Cartographer tuning issue, not just a planner issue.

## Machines

- i9 Alienware
  - user: `felix`
  - campus interface: `wlp3s0`
  - example recovery IP: `10.66.112.64`
- robotcar Pi 5
  - user: `robotcar`
  - password: `234123`
  - campus interface: `wlan0`
  - example recovery IP: `10.66.38.108`
  - ROS container: `ros2-humble`

Those two IPs are only examples from the successful pass. They are not permanent truth.

## Canonical command

If the SSH target IPs for this boot are already known:

```bash
I9_HOST='felix@<current_i9_campus_ip>' \
PI_HOST='robotcar@<current_pi_campus_ip>' \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh full-active
```

The script then:

- SSHes to those hosts
- re-detects the current peer IPs from `wlp3s0` and `wlan0`
- rewrites the CycloneDDS XMLs
- restarts the Pi container
- starts the Pi sensing chain
- starts the Pi client
- validates Pi-origin traffic on i9
- starts the motor driver
- starts the planner
- starts RViz
- sends a tiny motion pulse

## Exact startup order

Never improvise the order.

1. Stop old planner and RViz on i9.
2. Restart the Pi container.
3. Start Pi sensors and Cartographer with `start_full.sh`.
4. Verify Pi sensing is really alive.
5. Start `pi5_client.py`.
6. Verify `/car1/pose` is real.
7. Start `cmd_vel_driver.py`.
8. Start `i9_planner.py`.
9. Start RViz.
10. Run `goal-test`.

If something fails at step 3, do not continue to step 8 and hope it fixes itself.

## Acceptance tests that count

### Passive validation

These must be receivable on i9:

- `/map`
- `/scan`
- `/tf`
- `/car1/pose`

Use `ROS2CLI_NO_DAEMON=1` when checking.

### Real planner validation

This is the real final test:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh goal-test
```

That counts as success only if all of these happen:

- i9 publishes `/goal_pose`
- Pi receives non-empty `/car1/path`
- `pi5_client.py` logs `Goal forwarded`
- `pi5_client.py` logs `Path received`
- `i9_planner.py` logs `[build]`
- `i9_planner.py` logs `Build OK`

If only topic names appear, it is not enough.

## Commands worth remembering

Start everything:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh full-active
```

Status:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh status
```

Real planner proof:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh goal-test
```

Tiny motion pulse:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh move-test
```

Stop the stack:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh stop
```

## The biggest traps and how to avoid them

### 1. Hardcoding stale campus IPs

Bad habit:

- keep using `10.66.112.64` and `10.66.38.108` forever

Fix:

- override `I9_HOST` and `PI_HOST` for the current boot
- let `prepare` auto-detect the current peer IPs

### 2. Trusting `ros2 topic list`

Bad habit:

- assume topic names mean data is flowing

Fix:

- require real samples
- require `goal-test`
- use `ROS2CLI_NO_DAEMON=1`

### 3. Starting planner too early

Bad habit:

- planner first
- RViz first
- hope the Pi catches up

Fix:

- Pi sensing first
- Pi client second
- planner later
- RViz last

### 4. Reusing the wrong RViz config

Bad habit:

- launch the old visualization config
- then conclude SMART3D is dead when the screen looks wrong

Fix:

- use `/home/felix/smart_rviz_bridge/smart_real_robot.rviz`

### 5. Misreading a blank RViz window

Bad habit:

- assume DDS is broken just because `/smart/*` visuals are missing

Fix:

- first check `/map`, `/scan`, `/tf`, `/car1/pose`
- then run `goal-test`

The final accepted baseline is the closed planning loop, not whether every old marker layer renders.

### 6. Forgetting participant separation on i9

Bad habit:

- let planner, RViz, and validation all fight for the same CycloneDDS participant setup

Fix:

- planner: participant `31`
- RViz: participant `32`
- validation: participant `33`

### 7. Expecting doors to clear instantly while the robot is stationary

Bad habit:

- assume opening a door should immediately erase that wall in the map even if Cartographer is throttling scan acceptance

Fix:

- i9 planner now rebuilds faster on small local map changes
- Pi Cartographer `motion_filter` is now tuned for faster near-stationary updates:
  - `max_time_seconds = 0.5`
  - `max_distance_meters = 0.05`
  - `max_angle_radians = math.rad(0.5)`
- the LiDAR still must physically see through the opened doorway before the map can clear

### 8. Thinking the code was hard-bound to USB Wi-Fi

Bad habit:

- blame `i9_planner.py` or `pi5_client.py` for transport selection

Fix:

- the real binding is runtime DDS XML plus environment
- the code itself was not the cause

### 9. Leaving half-dead processes around

Bad habit:

- retry on top of stale `planner`, `rviz`, or sensor processes

Fix:

- stop first
- then restart in order

## What worked at the end

The final accepted campus-WiFi loop proved all of this:

- Pi sensing reached i9
- i9 planner accepted goals
- i9 planner built a path
- Pi received `/car1/path`
- the base moved

That is enough to call the chain recovered.

## What is historical only now

These older ideas should not be treated as current baseline:

- USB/AP as the default transport
- proving health from topic discovery alone
- temporary Pi-side SMART visualization relay as the default solution

## Useful files

- [smart3d_campus.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh)
- [SMART3D_CAMPUS_RUNTIME.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_CAMPUS_RUNTIME.md)
- [importantfori9andpiconnect.md](/Users/felixfang/Documents/Playground/importantfori9andpiconnect.md)

## Shutdown note

When you are done debugging:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh stop
```

If the Pi side ever looks stuck after that, reconnect and kill the remaining container processes before the next run. Do not stack a new launch on top of uncertain leftovers.
