# SMART3D Pitfalls and Failure Atlas (2026-04-19)

This file is organized by symptom.

Use it when something "looks wrong" and the next question is:

- what is this probably
- what should be checked first
- what file or command is closest to the truth

## 1. Symptom: LiDAR heading looks off by about 45 degrees

### What it usually means

The first suspect is not TF. The first suspect is LiDAR compensation in:

- `jetson/m10p_uart.yaml`

### Known bad case already seen

- `compensation: true`
- driver log showed a suspicious `degree_compensation = -327.670000`

### Known good direction fix

- set `compensation: false`

### First files to inspect

- `jetson/m10p_uart.yaml`
- `vendor/Lslidar_ROS2_driver/...`

## 2. Symptom: The robot suddenly jumps absurdly far away, outside the map

### What it usually means

This is not ordinary slow IMU drift.

This is closer to:

- SLAM wrong relocalization
- scan timing / subdivision order trouble
- Cartographer losing and re-locking wrong

### Evidence repeatedly associated with it

- `Dropped earlier points`
- `Ignored subdivision ... previous subdivision time ... is not before current`
- RViz local scan no longer matching the local pose

### First places to inspect

- `jetson/scan_metadata_normalizer.py`
- `jetson/start_full_native.sh`
- Cartographer logs on Jetson
- whether UDP camera was running and loading the board

## 3. Symptom: Global map shape is basically right, but local scan / local pose rotates or drifts during turns

### What it usually means

This is a local tracking problem, not a total global-map collapse.

Likely suspects:

- turn-time local pose quality
- IMU / scan timing interaction
- missing or weak short-term odom prior
- stale path follower behavior

### Why this is tricky

The system can be globally right and locally ugly at the same time.

That is exactly why the bug feels weird:

- the hallway shape is right
- but the robot can still rotate awkwardly or locally mismatch

### First places to inspect

- `pi5_client.py`
- IMU topic quality
- whether wheel odom is disabled
- turn probes and stationary debug outputs

## 4. Symptom: RViz is blank, or Smart3D markers are missing

### What it does not automatically mean

It does not automatically mean DDS is dead.

### Repeated real causes

- wrong RViz config
- planner not actually started
- old RViz from a previous run
- old map / no fresh path display topic

### First checks

- confirm `rviz2` process really restarted
- confirm `i9_planner.py` really restarted
- confirm RViz uses:
  - `/car1/path_display`
- confirm `/map_campus`, `/scan`, `/car1/pose` exist

## 5. Symptom: Green path stays behind the robot and does not visually shrink

### What it means

This is usually display-layer state, not core SMART logic.

### Current intended design

- `/car1/path` is for control
- `/car1/path_display` is for RViz

The planner should refresh display path so walked segments disappear.

### First checks

- RViz config topic
- planner restart
- whether an old planner copy is still running

## 6. Symptom: The robot waits until the path disappears, then moves

### What it means

That was caused by the rejected stale-path hold experiment.

### Important truth

That behavior was not "SMART3D being correct".
It was a bad follower-side experiment.

### What to do

Do not reintroduce stale-path holding logic without very strong evidence.

Prefer the reverted follower from:

- `backups/perfect2_2026-04-19_053032/pi5_client.py`

## 7. Symptom: Wall is visible in scan, but takes too long to become a stable wall in Smart3D

### What it means

There are two layers:

1. raw scan sees something immediately
2. occupancy map / extracted walls update later

That means the robot can "see" a corner before the planner has a stable wall
object for it.

### Current mitigation already used

- lower planner map-change thresholds
- allow fresh-wall-triggered static repair

### What not to assume

Do not assume every visible scan point should instantly become a gray 3D wall.

## 8. Symptom: Real wall or corner gets treated like a person

### What it means

Often the system has not yet promoted that geometry into known walls, so it is
temporarily living in the "unknown but not near known wall" bucket.

### Important nuance

This is not always a pure false positive.
Sometimes the object is genuinely not yet a stable wall in the map pipeline.

### Current mitigation

The planner now filters fresh walls before known walls when classifying dynamic
obstacles.

## 9. Symptom: Robot gets stuck rotating near a wall corner

### What it usually means

Several things can combine here:

- corner wall not yet stable in map extraction
- path anchor too close to the chassis
- follower still trying to chase a point that is awkwardly behind or beside the
  robot
- local pose not clean during the turn

### Important truth

This is not always a single-bug explanation.

Sometimes it is:

- map lag
- plus follower geometry
- plus weak short-term odom during a turn

## 10. Symptom: Wheel odom should help, but turning gets worse or the chain destabilizes

### What it usually means

Wheel odom quality is not yet trustworthy enough.

Likely causes:

- wheel slip
- partial drag
- bad sign / axis interpretation
- serial contention
- missing common ground
- bad return data

### Important operational truth

Wheel odom is not a free upgrade.
Bad odom can make the stack worse than no odom.

## 11. Symptom: DDSM115 commands go out, but no proper replies come back

### What it usually means

This is probably not a ROS problem.

It is probably one of:

- no common ground
- A/B wiring issue
- wrong ID assumptions
- multiple RS485 owners
- four-motor shared-bus confusion

### The strongest practical rule

Only one normal owner should control `/dev/ttyUSB0` at a time.

### If debugging raw return traffic

Start with:

- one motor
- common ground confirmed
- no other serial owner

## 12. Symptom: Camera looks delayed and map goes bad at the same time

### What it usually means

The camera is often a load amplifier, not always the only root cause.

### Repeated pattern

- ffmpeg / ffplay load rises
- Cartographer starts dropping points
- local tracking gets uglier

### Safe default

Turn camera off while diagnosing SLAM.

## 13. Symptom: `ros2 topic list` looks healthy, but nothing actually works

### What it means

Topic names are not proof.

### What actually counts

- real messages
- `goal-test`
- process ownership
- real RViz updates

### Strong rule

Never declare the chain healthy from topic names alone.

## 14. Symptom: One old map keeps coming back

### What it usually means

- planner was not fully restarted
- RViz was not fully restarted
- old processes were left alive

### Strong rule

If visuals look suspiciously old, stop everything and relaunch in order.

## 15. Symptom: The stack feels worse than a previous version

### What to do first

Do not keep "fixing forward" blindly.

Compare against the backups:

- `imu_lidar_stable_2026-04-19_000927`
- `jetson_perfect1_2026-04-19_045543`
- `perfect_final_2026-04-19_052436`
- `perfect_final_display_2026-04-19_052722`
- `perfect2_2026-04-19_053032`

### Strong rule

When in doubt, restore the simplest stable baseline first.

## 16. Symptom: The next AI is editing the wrong copy

### This happened already

There are multiple trees and one older local private repo copy became polluted.

### Safer sources of truth

Prefer:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d`
- `backups/perfect2_2026-04-19_053032`
- GitHub `SMART3D-MultiAgent/jetsoncar`

Do not assume the older private repo checkout is canonical without checking it.

## 17. The Biggest Meta-Rules

1. Start from the clean no-wheel stable baseline when the system is in doubt.
2. Add wheel odom only on purpose.
3. Treat giant jumps as SLAM timing/relocalization events, not simple IMU drift.
4. Treat local-turn ugliness as a different problem from total map destruction.
5. Do not trust topic names more than logs, process ownership, and actual motion.
6. Do not debug SLAM and high-load camera streaming at the same time unless that
   exact combined case is the subject.
