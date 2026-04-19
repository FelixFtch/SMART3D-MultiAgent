# SMART3D Jetson Orin NX 16GB Migration Notes (Based Only on the Working Pi 5 Chain)

This note is for a future session that wants to migrate the currently working
`robotcar Pi 5 + i9 planner` SMART3D chain onto a `Jetson Orin NX 16GB`.

This document intentionally uses only the newly working Pi 5 version as the
reference baseline.

Do **not** start from old M3 PRO snapshots, old Jetson experiments, or older
`smart3d-campus` copies.

## 1. Canonical Sources to Read First

Read these in this order:

1. `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md`
2. `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_CAMPUS_CONNECTION_GUIDE.md`
3. `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_CAMPUS_RUNTIME.md`
4. `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh`

Those four files define the runtime that actually worked.

## 2. The Only Files That Matter for the Pi 5 Side

If Jetson is replacing the Pi 5 runtime, these are the files to study and port.

### Core thin-client logic

- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_client.py`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cmd_vel_driver.py`

Why they matter:

- `pi5_client.py` is the whole thin-client control layer:
  - reads `map -> base_link` TF
  - publishes `/{car_ns}/pose`
  - forwards `/goal_pose`
  - subscribes to `/{car_ns}/path`
  - runs local pure pursuit
  - does local laser emergency-stop gating
- `cmd_vel_driver.py` is the RS485 owner that turns `/cmd_vel` into DDSM115 RPM

If the Jetson is replacing the Pi 5, these two files are the first two files to
preserve behaviorally.

### Pi-side compatibility helpers

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/campus_map_bridge.py`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/pi_ros_watchdog.py`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_m10p_dev.sh`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_ddsm_dev.sh`

Why they matter:

- `campus_map_bridge.py` is not optional in the current campus topology.
  It republishes local RELIABLE `/map` as BEST_EFFORT `/map_campus`.
- `pi_ros_watchdog.py` is the current field band-aid for Pi-side sensor collapse,
  especially LiDAR disappearing.
- `mk_m10p_dev.sh` binds the expected M10P LiDAR device.
- `mk_ddsm_dev.sh` binds the correct FTDI adapter to `/dev/ttyUSB0`.

If those helpers are missing on Jetson, the stack may start but behave
non-deterministically because device names and campus DDS behavior will drift.

### Direct debug / recovery tools

- `/Users/felixfang/Documents/Playground/robotics_helpers/robotcar_wasdqe_test.py`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/robotcar_wasd_max.sh`
- `/Users/felixfang/Documents/Playground/ddsm_hard_stop.py`
- `/Users/felixfang/Documents/Playground/ddsm_emergency_brake.py`

Why they matter:

- They are the shortest path to prove the drivetrain is alive independently of
  Cartographer, DDS, planner, and RViz.
- A future Jetson migration should keep these exact debug hooks early.

## 3. The i9 Side Files That Still Matter

Even if the compute role changes, the current working chain is defined against
these files:

- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/i9_planner.py`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/rviz/smart_real_robot.rviz`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/i9/i9_ros_watchdog.py`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/i9/i9_host_watchdog.sh`

Why they matter:

- `i9_planner.py` is the current planner interface contract.
- `smart_real_robot.rviz` is the known-good visualization contract.
- the i9 watchdogs show what was considered worth monitoring on the planner side.

If Jetson is going to replace only the Pi:

- keep `i9_planner.py` on i9
- port only the Pi-side stack to Jetson

If Jetson is going to replace both Pi and i9:

- `i9_planner.py` becomes the file to move onto Jetson too
- at that point the whole DDS topology can be simplified because thin-client and
  planner can live on one machine

## 4. What Not to Use as the Starting Point

Do not start from these unless you are explicitly mining them for historical
ideas:

- `/Users/felixfang/Documents/Playground/smart3d-campus`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/field/jetson_m3pro_original_2026-04-12`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/extras/m3pro_tools_mac_snapshot`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/extras/m3pro_auto_mapper21_overlay`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/extras/m3pro_auto_mapper_gpt5_1_4`

Reason:

- those trees encode older hardware assumptions
- they are not the chain that was just validated
- they will cause confusion about startup order, namespaces, camera stack, and
  transport assumptions

## 5. The Current Pi 5 Runtime Contract

The future Jetson port should preserve this contract first before trying to
"improve" anything.

### Inputs expected locally on the robot computer

- `/scan`
- `/tf`
- local Cartographer map on `/map`
- RS485 access to DDSM115 on `/dev/ttyUSB0`

### Outputs expected from the robot computer

- `/car1/pose`
- `/map_campus`
- `/cmd_vel`

### Inputs expected from the planner side

- `/goal_pose`
- `/car1/path`

### Safety behavior that must remain local

- emergency stop from front arc laser check
- serial motor ownership
- deadman timeout behavior

Do not move those offboard first.

## 6. The Two Migration Modes

### Mode A: Jetson replaces only Pi 5

This is the safer migration.

Keep:

- i9 as the planner/RViz machine

Move to Jetson:

- sensor drivers
- Cartographer or SLAM stack
- `pi5_client.py` logic
- `cmd_vel_driver.py`
- device-binding helpers
- local watchdogs

Why this is the right first step:

- it isolates onboard-compute migration from planner migration
- it keeps the current working offboard SMART3D topology intact
- it lets the team compare Jetson-vs-Pi without changing planner logic

### Mode B: Jetson replaces both Pi 5 and i9

This is a second-stage migration.

Move:

- thin-client side
- planner side
- RViz if desired

At that point:

- `campus_map_bridge.py` may become unnecessary if map and planner stay local
- DDS XMLs can be simplified
- latency and campus packet loss become much less important

But do **not** start here. First prove Mode A.

## 7. The First Files to Copy Onto Jetson

For a clean first port, copy these first:

- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_client.py`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cmd_vel_driver.py`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/campus_map_bridge.py`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/pi_ros_watchdog.py`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_m10p_dev.sh`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_ddsm_dev.sh`
- `/Users/felixfang/Documents/Playground/robotics_helpers/robotcar_wasdqe_test.py`
- `/Users/felixfang/Documents/Playground/ddsm_hard_stop.py`

Then adapt the orchestration from:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh`

Do not hand-write a new startup sequence before understanding what this script
currently deploys and in what order.

## 8. What the Next AI Should Extract from `smart3d_campus.sh`

The current script is important because it is the operational truth, not just a
note.

The next AI should extract these facts from it:

1. which files are copied to i9
2. which files are copied to the Pi host
3. which files are copied into the `ros2-humble` container
4. which Cyclone XMLs are written
5. the exact startup order
6. the exact detached launch commands
7. how `campus_mode:=true` is passed
8. why planner uses `map_topic:=/map_campus`

That script currently pushes:

- `i9_planner.py` to `/home/felix/SMART3D/i9_planner.py`
- `pi5_client.py` to `/tmp/pi5_client.py` and `/root/pi5_client.py`
- `campus_map_bridge.py` to `/tmp/...` and `/root/...`
- `pi_ros_watchdog.py` to `/tmp/...` and `/root/...`
- the serial-binding helpers to `/tmp/...` and `/root/...`

It also launches:

- `/root/start_full.sh`
- `robot_state_publisher`
- `pi5_client.py`
- `cmd_vel_driver.py`
- `i9_planner.py`
- RViz

That is the operational baseline.

## 9. What Must Be Verified on Jetson Before Saying the Port Works

The future AI should not call the Jetson port "working" until all of these pass.

### Stage 1: pure drivetrain

- direct WASD test moves all wheels correctly
- hard stop works
- no RS485 ownership conflict

### Stage 2: local sensing

- LiDAR publishes stable `/scan`
- IMU publishes
- TF is alive
- local map exists

### Stage 3: thin client

- `/{car_ns}/pose` is published
- `/goal_pose` forwarding works
- local path follower consumes `/{car_ns}/path`

### Stage 4: campus planning chain

- Jetson publishes `/map_campus`
- i9 receives `/scan`, `/map_campus`, `/car1/pose`
- i9 planner builds and publishes `/car1/path`
- Jetson follows the path

### Stage 5: final acceptance

- the same `goal-test` used in the Pi 5 chain succeeds

If `goal-test` does not pass, the Jetson port is not done.

## 10. The Most Important Behavioral Things Not to Accidentally Change

These are easy to break during migration:

1. `campus_mode:=true` behavior on the thin client
2. BEST_EFFORT vs RELIABLE assumptions across campus Wi-Fi
3. `map_topic:=/map_campus` on planner side
4. motor ID mapping:
   - left front = 1
   - left rear = 3
   - right front = 2
   - right rear = 4
5. right-side sign inversion in `cmd_vel_driver.py`
6. local emergency stop semantics in `pi5_client.py`
7. deadman timeout in `cmd_vel_driver.py`
8. device-path binding for LiDAR and FTDI

If any of those change silently, the robot may look half-alive while being wrong.

## 11. Recommended Migration Order

This is the recommended order for the next AI:

1. read the current handoff and campus runtime docs
2. ignore old Jetson/M3 PRO trees
3. port direct RS485 control first
4. port local sensor bring-up
5. port `pi5_client.py`
6. port `campus_map_bridge.py`
7. keep i9 planner unchanged at first
8. reconnect campus DDS
9. pass the existing `goal-test`
10. only then consider moving planner compute onto Jetson

## 12. Short Answer

If the next AI wants the minimum useful reading list for moving the current
working Pi 5 chain to Jetson Orin NX 16GB, tell it to read these files first:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_client.py`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cmd_vel_driver.py`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/campus_map_bridge.py`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/pi_ros_watchdog.py`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_m10p_dev.sh`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_ddsm_dev.sh`

That set is the real reference baseline.
