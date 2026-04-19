# SMART3D Next-Session Handoff (2026-04-18)

This is the canonical handoff for the current `i9 Alienware <-> robotcar Pi 5` SMART3D chain.

If a future session only reads one file before touching anything, read this file first.

## 1. Read This First

### Canonical local source tree

The canonical working tree is:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d`

There is an older sibling copy at:

- `/Users/felixfang/Documents/Playground/smart3d-campus`

Treat that sibling copy as historical convenience, not the authority. The newer watchdog and helper changes were made under `connection-guideline/smart3d`.

### Canonical supporting notes

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_CAMPUS_CONNECTION_GUIDE.md`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_CAMPUS_RUNTIME.md`
- `/Users/felixfang/Documents/Playground/importantfori9andpiconnect.md`

`importantfori9andpiconnect.md` is useful as a historical notebook, but it is too long and contains older hypotheses that were later refined. For real operation, prefer this file plus the two `SMART3D_CAMPUS_*` files above.

## 2. Current Architecture Summary

### Baseline that actually worked

The current accepted baseline is:

- i9 on campus Wi-Fi via `wlp3s0`
- robotcar Pi 5 on campus Wi-Fi via `wlan0`
- ROS domain: `42`
- DDS: `rmw_cyclonedds_cpp`
- Pi publishes sensing and pose
- i9 does central planning
- Pi follows `/car1/path`

The old USB/AP path is rollback history only. Do not use it as the default assumption.

### The most important operational truths

1. Campus IPs are dynamic.
2. `ros2 topic list` is not proof.
3. The accepted end-to-end proof is `goal-test`.
4. Startup order matters.
5. Old processes must be killed before relaunch.
6. Blank RViz does not automatically mean DDS is broken.
7. The most fragile piece is still the LiDAR driver on the Pi.

## 3. Canonical Local File Tree

### The working control tree

```text
/Users/felixfang/Documents/Playground/connection-guideline/smart3d
├── SMART3D_CAMPUS_CONNECTION_GUIDE.md
├── SMART3D_CAMPUS_RUNTIME.md
├── SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md
├── i9
│   ├── i9_host_watchdog.sh
│   └── i9_ros_watchdog.py
├── pi
│   ├── campus_map_bridge.py
│   ├── mk_ddsm_dev.sh
│   ├── mk_m10p_dev.sh
│   ├── pi_host_watchdog.sh
│   └── pi_ros_watchdog.py
├── rviz
│   └── smart_real_robot.rviz
└── scripts
    ├── robotcar_wasd_max.sh
    └── smart3d_campus.sh
```

### Other important local files outside that tree

```text
/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy
├── i9_planner.py
├── pi5_client.py
├── cmd_vel_driver.py
├── cartographer_config/...
└── pi5_reference/...

/Users/felixfang/Documents/Playground
├── ddsm_emergency_brake.py
├── ddsm_hard_stop.py
└── robotics_helpers/robotcar_wasdqe_test.py
```

### Ownership model

- Edit `deploy/i9_planner.py` if the planner logic changes.
- Edit `deploy/pi5_client.py` if the path follower / pose relay changes.
- Edit `connection-guideline/smart3d/scripts/smart3d_campus.sh` if the runtime orchestration changes.
- Edit `connection-guideline/smart3d/pi/pi_ros_watchdog.py` if Pi-side self-healing changes.
- Edit `robotics_helpers/robotcar_wasdqe_test.py` for direct keyboard motor control.

## 4. Remote Runtime File Tree

### i9 runtime targets

The startup script pushes files here:

```text
/home/felix/SMART3D/i9_planner.py
/home/felix/smart_rviz_bridge/smart_real_robot.rviz
/tmp/cyclone-campus-planner.xml
/tmp/cyclone-campus-rviz.xml
/tmp/cyclone-campus-validate.xml
/tmp/cyclone-campus-watch.xml
/tmp/i9_planner_campus.log
/tmp/rviz_campus.log
/tmp/i9_host_watchdog.log
/tmp/i9_host_watchdog.stdout
/tmp/i9_ros_watchdog.log
```

### Pi host runtime targets

The startup script pushes files here first:

```text
/tmp/cyclone-campus-pi.xml
/tmp/pi5_client.py
/tmp/campus_map_bridge.py
/tmp/pi_ros_watchdog.py
/tmp/pi_host_watchdog.sh
/tmp/mk_m10p_dev.sh
/tmp/mk_ddsm_dev.sh
/tmp/pi_host_watchdog.log
/tmp/pi_host_watchdog.stdout
```

### Pi container runtime targets

Then it copies the host-side files into `ros2-humble`:

```text
/root/start_full.sh                     (already existed in container)
/root/pi5_client.py                    (copied from local deploy/)
/root/campus_map_bridge.py             (copied from connection-guideline/)
/root/pi_ros_watchdog.py               (copied from connection-guideline/)
/root/mk_m10p_dev.sh                   (copied from connection-guideline/)
/root/mk_ddsm_dev.sh                   (copied from connection-guideline/)
/root/cmd_vel_driver.py                (already existed in container)
/root/m10p_ws/m10p_uart.yaml           (already existed in container)
/root/robotcar_description/robotcar.urdf
/tmp/start_full_campus.log
/tmp/lidar.log
/tmp/cartographer.log
/tmp/campus_map_bridge.log
/tmp/pi5_client_campus.log
/tmp/cmd_vel_driver_campus.log
/tmp/robot_state_publisher.log
/tmp/odom.log
/tmp/pi_ros_watchdog.log
```

## 5. SSH / Machine Identity

### i9

- user: `felix`
- typical current or example campus IP seen during recovery: `10.66.112.64`
- interface: `wlp3s0`
- SSH key from Mac:
  - `/Users/felixfang/.ssh/rpi5_robot`

### robotcar Pi 5

- user: `robotcar`
- password: `234123`
- typical current or example campus IP seen during recovery: `10.66.38.108`
- interface: `wlan0`
- Docker container: `ros2-humble`

Do not treat either example IP as permanent truth. The scripts are designed to accept current IP overrides.

## 6. Topics and Interaction Model

### Pi-origin topics that must be healthy on i9

- `/scan`
- `/map_campus`
- `/car1/pose`
- `/tf`

### Planner / command side

- i9 publishes `/goal_pose`
- Pi forwards `/goal_pose` to `/{ns}/goal_pose`
- i9 planner publishes `/car1/path`
- Pi client subscribes to `/car1/path`
- Pi client publishes `/cmd_vel`
- `cmd_vel_driver.py` owns RS485 and drives DDSM115

### Why `/map_campus` exists

`campus_map_bridge.py` converts:

- local RELIABLE `/map`

to:

- campus BEST_EFFORT `/map_campus`

This is intentional. Across the campus link, BEST_EFFORT has been more reliable than trying to force the original RELIABLE map topic end to end.

## 7. The Only Startup Order That Should Be Trusted

Never improvise this order.

1. Stop old planner, RViz, client, driver, and stale sensors.
2. Re-detect current i9 and Pi campus IPs.
3. Rewrite Cyclone XMLs.
4. Restart the Pi container.
5. Start Pi sensors and Cartographer.
6. Start `campus_map_bridge.py`.
7. Start `robot_state_publisher`.
8. Start `pi5_client.py`.
9. Confirm Pi-origin data is really flowing.
10. Start watchdogs.
11. Start `cmd_vel_driver.py`.
12. Start `i9_planner.py`.
13. Start RViz.
14. Run `goal-test`.

If any step fails, stop there and fix it. Do not continue and hope later stages magically repair the chain.

## 8. Commands the Next Session Should Use

### Preferred staged startup

From the Mac:

```bash
I9_HOST='felix@<current_i9_ip>' \
PI_HOST='robotcar@<current_pi_ip>' \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh prepare

I9_HOST='felix@<current_i9_ip>' \
PI_HOST='robotcar@<current_pi_ip>' \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh start-passive

I9_HOST='felix@<current_i9_ip>' \
PI_HOST='robotcar@<current_pi_ip>' \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh validate

I9_HOST='felix@<current_i9_ip>' \
PI_HOST='robotcar@<current_pi_ip>' \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh start-active

I9_HOST='felix@<current_i9_ip>' \
PI_HOST='robotcar@<current_pi_ip>' \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh goal-test
```

### Shortcut full startup

Use only when the system is already in a clean state:

```bash
I9_HOST='felix@<current_i9_ip>' \
PI_HOST='robotcar@<current_pi_ip>' \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh full-active
```

### Status

```bash
I9_HOST='felix@<current_i9_ip>' \
PI_HOST='robotcar@<current_pi_ip>' \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh status
```

### Stop

```bash
I9_HOST='felix@<current_i9_ip>' \
PI_HOST='robotcar@<current_pi_ip>' \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh stop
```

### Tiny motion pulse

```bash
I9_HOST='felix@<current_i9_ip>' \
PI_HOST='robotcar@<current_pi_ip>' \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh move-test
```

### Watchdog control

```bash
I9_HOST='felix@<current_i9_ip>' \
PI_HOST='robotcar@<current_pi_ip>' \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh watch-start

I9_HOST='felix@<current_i9_ip>' \
PI_HOST='robotcar@<current_pi_ip>' \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh watch-status
```

## 9. What "Normal Startup" Looks Like

### After `prepare`

Expected:

- remote XML files exist on i9 and Pi
- planner and RViz config have been copied to i9
- `i9_planner.py` from local deploy has been copied to `/home/felix/SMART3D/i9_planner.py`

### After `start-passive`

Expected Pi-side processes:

- `start_full.sh`
- `cartographer_node`
- `lslidar_driver_node`
- `imu_qos_bridge.py`
- `ddsm115_odom.py`
- `campus_map_bridge.py`
- `robot_state_publisher`
- `pi5_client.py`
- `pi_ros_watchdog.py`

Expected Pi-side topics with real samples:

- `/scan`
- `/map_campus`
- `/car1/pose`

Expected log shape:

- `campus_map_bridge.py` logs `Bridged map #...`
- `pi5_client.py` logs `Pi5 client [car1] ... waiting for path...`

Not acceptable:

- only topic names from `ros2 topic list`
- empty probes
- `malloc(): unaligned fastbin chunk detected` in `/tmp/lidar.log`

### After `validate`

Expected on i9:

- `ros2 topic echo /scan --once --qos-reliability best_effort` returns real data
- `ros2 topic echo /map_campus --once --qos-durability transient_local --qos-reliability best_effort` returns real data
- `ros2 topic echo /car1/pose --once --qos-reliability best_effort` returns real data

This proves Pi-origin data is truly crossing to i9.

### After `start-active`

Expected processes:

- Pi: `cmd_vel_driver.py`
- i9: `i9_planner.py`
- i9: `rviz2`

Expected planner readiness:

- planner log should not be dead or empty
- when a goal is later sent, planner should log `[build] ...`

### After `goal-test`

This is the final acceptance test.

Expected:

- i9 publishes `/goal_pose`
- Pi receives non-empty `/car1/path`
- `pi5_client.py` logs `Goal forwarded: (...)`
- `pi5_client.py` logs `Path received: ... pts`
- `i9_planner.py` logs `[build] ...`
- `i9_planner.py` logs `Build OK`

If `goal-test` does not pass, the chain is not accepted as healthy even if RViz shows a map and scan.

## 10. RS485 / DDSM115 Control

### Normal chain

- `pi5_client.py` publishes `/cmd_vel`
- `cmd_vel_driver.py` owns `/dev/ttyUSB0`
- DDSM115 motors follow through RS485

### Strong rule

Only one process should own the RS485 motor path at a time.

Do not run:

- `cmd_vel_driver.py`
- `pi5_client.py`
- direct keyboard serial tool

in competing combinations unless you intentionally stop the others first.

## 11. Direct Keyboard Control (WASD)

### Canonical direct-control script

- `/Users/felixfang/Documents/Playground/robotics_helpers/robotcar_wasdqe_test.py`

### Wrapper that pushes and launches it on the Pi

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/robotcar_wasd_max.sh`

### What the wrapper does

1. Copies the direct-control script to Pi host.
2. Starts `ros2-humble` if needed.
3. Kills `cmd_vel_driver.py`.
4. Kills `pi5_client.py`.
5. Copies the script into the container.
6. Launches interactive RS485 direct control.

### Direct wrapper command

```bash
PI_HOST='robotcar@<current_pi_ip>' \
PI_PASS='234123' \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/robotcar_wasd_max.sh
```

### Direct no-wrapper command

```bash
sshpass -p '234123' ssh -tt -o StrictHostKeyChecking=no robotcar@<current_pi_ip> '
docker start ros2-humble >/dev/null 2>&1 || true
docker exec ros2-humble bash -lc "pkill -f \"[c]md_vel_driver.py\" || true; pkill -f \"[p]i5_client.py\" || true"
docker exec -it ros2-humble python3 /root/robotcar_wasdqe_test.py \
  --port /dev/ttyUSB0 \
  --base-rpm 330 \
  --turn-rpm 330 \
  --arc-rpm 330 \
  --deadman-sec 0.35 \
  --max-rpm-limit 330
'
```

### Direct-control keys

- `w / s`: forward / backward
- `a / d`: in-place left / right
- `q / e`: arc left / right
- `x` or space: stop
- `z`: hard brake
- `1 2 3 4`: single motor forward
- `! @ # $`: single motor backward
- `r`: query 4 motor IDs
- `[` / `]`: lower / raise RPM
- `Ctrl-C`: exit and brake

## 12. Emergency Stop Commands

### First-level stop: stop the SMART3D chain

```bash
I9_HOST='felix@<current_i9_ip>' \
PI_HOST='robotcar@<current_pi_ip>' \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh stop
```

### Hard motor brake if the base keeps moving

Local file:

- `/Users/felixfang/Documents/Playground/ddsm_hard_stop.py`

Remote hard-stop command from the Mac:

```bash
cat /Users/felixfang/Documents/Playground/ddsm_hard_stop.py | \
sshpass -p '234123' ssh -o StrictHostKeyChecking=no -o PreferredAuthentications=password -o PubkeyAuthentication=no robotcar@<current_pi_ip> \
'cat > ~/ddsm_hard_stop.py && chmod +x ~/ddsm_hard_stop.py && python3 ~/ddsm_hard_stop.py'
```

### Softer brake variant

Local file:

- `/Users/felixfang/Documents/Playground/ddsm_emergency_brake.py`

### Physical reality

If software stop fails:

- lift the driven wheels
- cut motor power

Do not wait for ROS to save you if the chassis is physically moving away.

## 13. Known Pitfalls and Avoidance Rules

### Pitfall 1: stale campus IPs

Bad habit:

- reusing `10.66.112.64` and `10.66.38.108` forever

Correct habit:

- treat those as examples only
- override `I9_HOST` and `PI_HOST` for the current boot
- let `prepare` re-detect peer IPs

### Pitfall 2: `ros2 topic list` lies by omission

Bad habit:

- seeing names in the graph and deciding the link is healthy

Correct habit:

- require `ros2 topic echo --once`
- require `goal-test`
- use `ROS2CLI_NO_DAEMON=1`

### Pitfall 3: starting things out of order

Bad habit:

- planner first
- RViz first
- then maybe sensors later

Correct habit:

- clean restart
- passive Pi stack first
- validate Pi-origin traffic
- only then active planner + RViz

### Pitfall 4: leaving stale processes alive

This was a real source of confusion.

Stale processes can cause:

- old path topics
- old planner state
- lingering DDS participants
- RS485 contention
- false "it launched" impressions

The user explicitly prefers a hard clean restart instead of stacking new launches on top of old ones. Respect that.

### Pitfall 5: wrong RViz config

Use:

- `/home/felix/smart_rviz_bridge/smart_real_robot.rviz`

Do not accidentally use old SMART visualization configs that expect a different simulation or marker setup.

### Pitfall 6: assuming blank RViz means DDS failure

Check in this order:

1. `/scan`
2. `/map_campus`
3. `/car1/pose`
4. planner logs
5. `goal-test`

Only then decide DDS is broken.

### Pitfall 7: RS485 serial contention

Do not let:

- `cmd_vel_driver.py`
- `pi5_client.py`
- direct `robotcar_wasdqe_test.py`
- odom or other RS485 owners

fight over `/dev/ttyUSB0`.

### Pitfall 8: editing the wrong copy of the repo

The user has both:

- `/Users/felixfang/Documents/Playground/smart3d-campus`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d`

The newer work should go into `connection-guideline/smart3d`.

## 14. LiDAR Failure Mode: What It Looks Like

The most important unsolved instability is the Pi-side LiDAR driver crash.

### Observed symptoms

- `/scan` disappears
- `cartographer_node` starts waiting on scan
- IMU may still be healthy
- map stops evolving
- planning becomes stale or impossible

### Signature already observed

In `/tmp/lidar.log`:

```text
malloc(): unaligned fastbin chunk detected
```

In Cartographer-side behavior:

- `Queue waiting for data: (0, scan)`

### Current interpretation

This looks more like a driver memory-corruption bug that gets triggered under load, not a clean "map too large" limit.

### Current mitigation

`/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/pi_ros_watchdog.py`

now has LiDAR-only self-healing:

- if `/scan` gap exceeds 3 seconds
- it tries to restart only `lslidar_driver_node`
- using `/root/mk_m10p_dev.sh`
- without fully restarting the rest of the stack

This is mitigation, not a proof that the root cause is solved.

## 15. Device Binding Helpers

### LiDAR binding helper

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_m10p_dev.sh`

Why it exists:

- avoid hardcoding `/dev/ttyACM1`
- bind `/dev/m10p` by USB serial identity

Known serials encoded in that helper:

- primary M10P serial: `5A6D010263`
- fallback M10P serial: `5A6C069316`
- IMU serial to avoid: `5B0B030655`

### DDSM FTDI binding helper

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_ddsm_dev.sh`

Why it exists:

- avoid assuming the RS485 dongle always lands on the same `ttyUSB*`
- force the correct FTDI to appear at `/dev/ttyUSB0`

Known FTDI serial encoded there:

- `BG01LYTM`

## 16. Watchdogs

### Pi-side watchdogs

- host: `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/pi_host_watchdog.sh`
- ROS: `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/pi_ros_watchdog.py`

What they watch:

- load
- temperature
- memory
- `/scan`
- `/IMU_data`
- `/car1/pose`
- `/map`

### i9-side watchdogs

- host: `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/i9/i9_host_watchdog.sh`
- ROS: `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/i9/i9_ros_watchdog.py`

What they watch:

- `/scan`
- `/map_campus`
- `/car1/pose`
- `/tf`
- i9 load
- Wi-Fi stats on `wlp3s0`

### How to read watchdog health

Use:

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh watch-status
```

If watchdog logs show:

- repeated `scan_gap`
- repeated LiDAR restarts
- throttling
- Wi-Fi degradation

then treat the chain as unstable even if it comes back temporarily.

## 17. Optional Camera Chain

There is an optional UDP Logitech camera chain:

- `/Users/felixfang/Documents/Playground/smart3d-campus/robotcar_logitech_udp.sh`

It is not part of the accepted minimum SMART3D baseline.

Recent field behavior suggested that adding live video may correlate with worse end-to-end lag or make debugging harder, so the safest baseline is:

- SMART3D first
- camera off unless specifically needed

## 18. Strong Recommendations for the Next Session

1. Use `connection-guideline/smart3d` as the canonical code tree.
2. Start with staged commands, not `full-active`, when debugging.
3. Never trust `ros2 topic list` alone.
4. Never skip `goal-test`.
5. Kill stale processes before relaunching.
6. Treat LiDAR instability as still unresolved.
7. If the chassis is moving unsafely, stop it first and debug later.
8. If direct RS485 control is needed, stop `cmd_vel_driver.py` and `pi5_client.py` first.

## 19. Do Not Confuse the Pi5 Target with the Current Jetson Stopgap

The original Pi5 formal Cartographer baseline was:

- LiDAR `/scan`
- IMU `/IMU_data`
- wheel odom `/odom`
- all fused into Cartographer with `robotcar_2d_odom.lua`

The most important original files for that are:

- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/start_full.sh`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/cartographer_launch.py`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d_odom.lua`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/ddsm115_odom.py`

So the real restore target for Jetson is not “make no-IMU stable”.

The current Jetson stabilization point is only:

- LiDAR-driven SLAM input
- IMU device still online but not fused
- wheel odom off during the stability checks
- Cartographer using `robotcar_2d_noimu.lua`

That current stopgap is useful because it is the most stable Jetson state found
so far, but it is not a faithful reproduction of the Pi5 formal baseline.

The current LiDAR code that should be treated as the stable Jetson base is:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/vendor/Lslidar_ROS2_driver/lslidar_driver/src/input.cc`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/vendor/Lslidar_ROS2_driver/lslidar_driver/src/lslidar_driver.cc`

That base comes from restoring:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/backups/Lslidar_ROS2_driver_2026-04-18_215404`

while keeping only narrow low-level input-safety fixes.

The current frozen local backup of that safer LiDAR base is:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/backups/Lslidar_ROS2_driver_stable_input_safety_2026-04-18_232420`

Do not blindly redeploy:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/backups/Lslidar_ROS2_driver_pre_restore_2026-04-18_223416`

because that tree included timing / geometry experiments that correlated with a
much worse drifting state.
9. If the Pi seems dead, do not assume the old IP is still correct.
10. If a future session modifies planner or path follower behavior, edit the files in `private-repos/SMART3D-MultiAgent/deploy`, because `prepare` copies from there.

## 19. Minimal Checklist for a New Session

Before touching code:

- read this file
- read `SMART3D_CAMPUS_CONNECTION_GUIDE.md`
- identify current i9 and Pi IPs

Before declaring "healthy":

- `start-passive` succeeds
- `validate` returns real samples
- `start-active` starts planner and RViz
- `goal-test` proves Pi received `/car1/path`

Before declaring "broken":

- confirm current IPs
- run `status`
- run `watch-status`
- inspect:
  - `/tmp/lidar.log`
  - `/tmp/start_full_campus.log`
  - `/tmp/pi5_client_campus.log`
  - `/tmp/cmd_vel_driver_campus.log`
  - `/tmp/i9_planner_campus.log`

If those steps are not done, the diagnosis is incomplete.
