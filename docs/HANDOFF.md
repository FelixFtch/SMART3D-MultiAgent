# SMART-3D Multi-Agent: Real Robot Deployment Handoff

> **For the next AI assistant.** This document tells you everything you need to write
> `m3pro_client.py` and deploy multi-agent SMART-3D on real hardware.
> Felix Fang built the entire system solo.

---

## 1. What Already Works

### Simulation (100% complete)
- 8 scenarios: building, office, warehouse, bottleneck, mega_hospital, mega_airport, megacampus, mega_university
- 4-agent cross mode with Global King priority, flee, chain push
- Shared wall discovery (1.04x–2.09x faster than independent, 12000 trials)
- E3 full-tree repair (4–9x fewer rebuilds than path-only Senior)
- Spatial index v2 (−26% wall-clock on mega maps)
- `--no-wall-sharing` ablation flag
- All benchmark scripts and data

### Real Robot Single-Car (working, tested)
- Pi 5 car navigates real-world environments using i9 as central planner
- i9 receives LiDAR scan + map from Pi 5, runs SMART-3D planning, sends path back
- Pi 5 follows path with pure pursuit + local emergency stop
- CycloneDDS over WiFi AP, ROS_DOMAIN_ID=42

### Real Robot Multi-Car (code written, not fully tested)
- `i9_planner.py` already supports `car_names:="car1,m3pro"` parameter
- Each car spawns its own Rust `smart_planner_server` subprocess
- Each car's planner treats other cars as dynamic obstacles with OHZ

---

## 2. File Locations

### i9 Alienware Aurora R16 (10.42.0.160 via AP, 10.66.112.64 campus)

```
~/SMART3D/
├── Cargo.toml
├── src/
│   ├── smart/smart.rs              ← SMART core (prune + repair)
│   ├── obstacles/
│   │   ├── analytic_obstacle.rs    ← Original wall checker
│   │   ├── spatial_checker.rs      ← Spatial index v2 (for multi-agent sim)
│   │   └── mod.rs
│   └── rrt/                        ← RRT* builder + KD-tree
├── examples/
│   ├── smart_multi_agent.rs        ← ★ Multi-agent simulation runner
│   │                                  (has --no-wall-sharing flag)
│   ├── smart_rviz_json.rs          ← Single-agent simulation runner
│   └── smart_planner_server.rs     ← ★ Rust planner for real robot (JSON stdin/stdout)
├── i9_planner.py                   ← ★ Central ROS2 planner node (~750 lines)
├── bench_sharing_ablation.py       ← Shared vs Independent benchmark
├── bench_single_agent.py           ← Single-agent benchmark
├── bench_e3_vs_senior.py           ← E3 vs Senior benchmark
├── sharing_ablation.json           ← 12000-trial ablation results
├── single_agent_benchmark.json     ← 6000-trial single-agent results
└── target/release/examples/
    ├── smart_multi_agent            ← Compiled simulation binary
    └── smart_planner_server         ← Compiled real-robot planner binary

~/smart_rviz_bridge/
├── smart_rviz_bridge.py            ← Single-agent JSON → ROS2 Marker bridge
├── smart_multi_agent_bridge.py     ← Multi-agent JSON → ROS2 Marker bridge
└── smart_rviz.rviz                 ← RViz config
```

### Raspberry Pi 5 Car (10.42.0.1 AP host, 10.66.129.126 campus SSH)

```
SSH: robotcar@10.66.129.126, password: 234123
Inside Docker container "ros2-humble":

/root/
├── pi5_client.py               ← ★ Thin client for real robot
│                                   TF→pose relay, path→pure pursuit, emergency stop
├── cmd_vel_driver.py           ← RS485 motor control (4× DDSM115 Waveshare)
├── ddsm115_odom.py             ← Wheel odometry from motor encoders
├── imu_qos_bridge.py           ← IMU QoS adapter
├── start_full.sh               ← Sensor startup (LiDAR+IMU+Odom+Cartographer)
├── smart_nav_node_v4.py        ← Legacy single-machine nav (reference only)
├── m10p_ws/                    ← LiDAR driver workspace (Leishen M10P)
├── imu_ws/                     ← IMU driver workspace (HiPNUC N300)
└── cartographer_config/
    ├── cartographer_launch.py
    └── robotcar_2d_odom.lua
```

### Yahboom ROSMASTER M3 PRO (Jetson Orin NX, 10.66.147.9 campus)

```
SSH: jetson@10.66.147.9, password: yahboom
Needs USB RTL8812AU WiFi dongle to join robotcar AP (10.42.0.x)

Key facts:
- ROS2 Humble on Ubuntu 22.04
- Mecanum wheels (omnidirectional: linear.x + linear.y + angular.z)
- Dual LiDAR (front + rear, 360° coverage)
- Dabai DCW2 depth camera
- 6-DOF robotic arm with gripper
- micro-ROS agent bridges ESP32 chassis at /dev/ttyACM0
- Default ROS_DOMAIN_ID=30 in start_agent.sh → MUST change to 42
- Already has Cartographer, Nav2, SLAM Toolbox installed
- robot_radius = 0.25m (20cm wide + 40cm long, outer circle ~22cm + margin)
```

### Backups on i9

```
~/Desktop/SMART3D_backup_complete/
├── PERFECT_spatial_index_v2_*/     ← Spatial index v2 code + benchmark
├── PERFECT_sharing_ablation_*/     ← Ablation data + scripts
├── backup_3d_twolayer/             ← 3D simulation backup
├── SMART3D_RESEARCH_DASHBOARD.html ← Benchmark dashboard
├── SMART3D_EXPLAINER.html          ← English research explainer
└── SMART3D_EXPLAINER_CN.html       ← Chinese research explainer
```

---

## 3. Architecture

```
┌──────────────────────────────────────────────────┐
│           i9 Central Planner (i9_planner.py)     │
│                                                  │
│  Agent "car1"  → Rust smart_planner_server #1    │
│                  → publishes /car1/path           │
│                  → subscribes /car1/scan, /car1/pose, /map │
│                                                  │
│  Agent "m3pro" → Rust smart_planner_server #2    │
│                  → publishes /m3pro/path          │
│                  → subscribes /m3pro/scan, /m3pro/pose     │
│                                                  │
│  Cross-avoidance: car1 planning sees m3pro as    │
│  SphericalObstacleWithOhz (real-time pose+speed) │
└────────────────────┬─────────────────────────────┘
                     │ WiFi AP (10.42.0.0/24)
                     │ CycloneDDS, ROS_DOMAIN_ID=42
          ┌──────────┼──────────┐
          ▼          ▼          ▼
     Pi 5 Car    M3 PRO    (Future: Balance Bot)
     car1        m3pro      balbot

     pi5_client  m3pro_client (TO WRITE)
     - TF→pose   - TF→pose
     - path→     - path→mecanum
       pursuit     controller
     - e-stop    - e-stop (dual lidar)
```

---

## 4. What Needs to Be Written

### 4.1 m3pro_client.py (highest priority)

Based on `pi5_client.py` (~260 lines). Key differences:

| Aspect | Pi 5 Car (pi5_client.py) | M3 PRO (m3pro_client.py) |
|--------|--------------------------|--------------------------|
| Drive | Differential (linear.x + angular.z) | Mecanum (linear.x + linear.y + angular.z) |
| Path following | Pure pursuit (rotate then drive) | Direct vector to waypoint (can strafe) |
| Emergency stop | Front arc ±30°, 0.3m, single LiDAR | Full 360° dual LiDAR |
| Motor interface | cmd_vel_driver.py → RS485 | Direct /cmd_vel (Yahboom stack handles it) |
| TF frames | map → odom → base_link | map → odom → base_link (same, but verify) |
| Namespace | /car1/* | /m3pro/* |
| Speed | 0.2 m/s | 0.3 m/s (adjustable) |
| robot_radius | 0.40m | 0.25m |

**Path following for mecanum is simpler than pure pursuit:**
```python
# Pure pursuit (Pi 5, differential):
#   1. Calculate heading error to lookahead point
#   2. Publish angular.z to rotate, linear.x to drive
#   3. Can't move sideways

# Mecanum (M3 PRO):
#   1. Calculate vector from robot to waypoint in robot frame
#   2. Publish linear.x = forward component, linear.y = lateral component
#   3. angular.z = heading alignment (optional, can face forward while strafing)
```

### 4.2 i9_planner.py modifications (minor)

Already supports multi-car. Needs:
1. **Per-agent robot_radius**: Currently global. Change to dict:
   ```python
   agent_radii = {'car1': 0.40, 'm3pro': 0.25}
   ```
2. **Per-agent speed**: Different max speeds
3. **Per-agent rrt_iterations**: M3 PRO is smaller, might need fewer

### 4.3 M3 PRO network setup

```bash
# On M3 PRO (jetson@10.66.147.9):
# 1. Plug in USB RTL8812AU WiFi dongle
nmcli device wifi connect "robotcar" password "23412300" ifname wlxXXXXXX

# 2. Install CycloneDDS (if not installed)
sudo apt install ros-humble-rmw-cyclonedds-cpp

# 3. Create CycloneDDS config
cat > /tmp/cyclone.xml << 'EOF'
<CycloneDDS>
  <Domain id="42">
    <General><AllowMulticast>true</AllowMulticast></General>
    <Discovery><Peers><Peer address="10.42.0.160"/></Peers></Discovery>
  </Domain>
</CycloneDDS>
EOF

# 4. Change start_agent.sh domain ID from 30 to 42
sed -i 's/ROS_DOMAIN_ID=30/ROS_DOMAIN_ID=42/' ~/start_agent.sh

# 5. Environment (every terminal)
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclone.xml
source /opt/ros/humble/setup.bash
```

---

## 5. Startup Procedure (Multi-Agent)

### Terminal 1: Pi 5 sensors
```bash
ssh robotcar@10.66.129.126  # pw: 234123
docker exec ros2-humble bash -c 'cat > /tmp/cyclone.xml << "XMLEOF"
<CycloneDDS><Domain id="42"><General><AllowMulticast>true</AllowMulticast></General>
<Discovery><Peers><Peer address="10.42.0.160"/></Peers></Discovery></Domain></CycloneDDS>
XMLEOF'
docker exec -it ros2-humble bash -lc '
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///tmp/cyclone.xml
  export ROS_DOMAIN_ID=42
  source /opt/ros/humble/setup.bash
  /root/start_full.sh
'
```

### Terminal 2: Pi 5 client
```bash
ssh robotcar@10.66.129.126
docker exec -it ros2-humble bash -lc '
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///tmp/cyclone.xml
  export ROS_DOMAIN_ID=42
  source /opt/ros/humble/setup.bash
  python3 /root/cmd_vel_driver.py &
  sleep 1
  python3 /root/pi5_client.py --ros-args -p car_ns:=car1
'
```

### Terminal 3: M3 PRO sensors + client
```bash
ssh jetson@10.66.147.9  # pw: yahboom
# Start Yahboom sensor stack (LiDAR + Cartographer)
sh ~/start_agent.sh &
sleep 10
# Start SMART-3D client
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclone.xml
python3 ~/m3pro_client.py --ros-args -p car_ns:=m3pro
```

### Terminal 4: i9 planner (multi-car)
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclone.xml
cd ~/SMART3D
python3 i9_planner.py --ros-args -p car_names:="car1,m3pro"
```

### Terminal 5: i9 RViz
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclone.xml
rviz2
```

---

## 6. Key Parameters

| Parameter | Location | car1 (Pi5) | m3pro | Notes |
|-----------|----------|------------|-------|-------|
| robot_radius | smart_planner_server.rs | 0.40m | 0.25m | RRT* collision check |
| step_size | smart_planner_server.rs | 0.25m | 0.25m | Path point density |
| wall_pad | i9_planner.py | 0.20m | 0.20m | Wall inflation |
| rrt_iterations | i9_planner.py | 15000 | 15000 | RRT* samples |
| driving_speed | client.py | 0.2 m/s | 0.3 m/s | Max speed |
| emergency_stop | client.py | 0.30m front | 0.30m 360° | Collision avoidance |
| ROS_DOMAIN_ID | everywhere | 42 | 42 | Must match |
| DDS | everywhere | rmw_cyclonedds_cpp | same | Not FastDDS |

---

## 7. Hardware Summary

| | Pi 5 Car (car1) | M3 PRO (m3pro) |
|---|---|---|
| Brain | Raspberry Pi 5 8GB | Jetson Orin NX 16GB |
| LiDAR | Leishen M10P (single, 12Hz) | Dual LiDAR (front+rear) |
| Camera | RealSense D415 | Dabai DCW2 depth |
| Drive | Differential (4× DDSM115) | Mecanum (omnidirectional) |
| Arm | None | 6-DOF + gripper |
| Chassis | 400×350mm | ~200×400mm |
| WiFi AP | wlan1: 10.42.0.1 (host) | Needs USB WiFi dongle |
| Campus | 10.66.129.126 | 10.66.147.9 |
| SSH | robotcar / 234123 | jetson / yahboom |

---

## 8. Network Topology

```
Internet (UCONN-GUEST)
    │
    ├── i9 (campus: 10.66.112.64)
    │       └── USB WiFi → 10.42.0.160
    │               │
    │         robotcar AP (10.42.0.0/24) ← ALL ROS2 TRAFFIC
    │               │
    ├── Pi 5 (wlan1: 10.42.0.1 = AP host, campus: 10.66.129.126)
    │
    ├── M3 PRO (USB WiFi → 10.42.0.x, campus: 10.66.147.9)
    │
    └── (Future: Balance Bot → 10.42.0.x)
```

---

## 9. What the Simulation Already Proved

These numbers come from 50,000+ benchmark trials and apply directly to real-robot expectations:

| Finding | Data | Implication for real robot |
|---------|------|---------------------------|
| Shared wall discovery 2.09x faster | building 23.2s vs 48.4s | Two real robots mapping together will finish faster |
| Per-agent spread reduced 4.2x | megacampus 12.3s → 2.9s | Robots finish at similar times, less waiting |
| E3 repair 4–9x fewer rebuilds | mega_hospital 34 vs 103 | Fewer planning freezes during navigation |
| 100% success rate (mega maps) | 1000/1000 trials | Robust enough for real deployment |
| Coordination overhead 6–15% | megacampus 108.6s → 116.8s | Multi-agent adds minimal delay on large maps |

---

## 10. Files to Read First

If you are the next AI, read these files in this order:

1. **This README** (you're reading it)
2. **`pi5_client.py`** on Pi 5 (`/root/pi5_client.py` in ros2-humble container) — the template for m3pro_client.py
3. **`i9_planner.py`** on i9 (`~/SMART3D/i9_planner.py`) — the central planner you'll modify for per-agent radius
4. **`smart_planner_server.rs`** on i9 (`~/SMART3D/examples/smart_planner_server.rs`) — the Rust planner subprocess
5. **`smart_multi_agent.rs`** on i9 (`~/SMART3D/examples/smart_multi_agent.rs`) — simulation runner with --no-wall-sharing

To get the Pi 5 files without SSH:
```bash
# From i9
scp robotcar@10.66.129.126:/tmp/pi5_backup.tar.gz . 2>/dev/null
# Or from inside container:
ssh robotcar@10.66.129.126 "docker exec ros2-humble cat /root/pi5_client.py" > pi5_client.py
ssh robotcar@10.66.129.126 "docker exec ros2-humble cat /root/cmd_vel_driver.py" > cmd_vel_driver.py
ssh robotcar@10.66.129.126 "docker exec ros2-humble cat /root/start_full.sh" > start_full.sh
```

---

## 11. Known Issues and Gotchas

1. **CycloneDDS config must be recreated after every reboot/container restart** (it lives in /tmp)
2. **Pi CycloneDDS must NOT restrict interfaces** (no `<Interfaces>` tag) — breaks internal container comms
3. **M3 PRO start_agent.sh uses DOMAIN_ID=30** — must change to 42 before running
4. **micro-ROS agent on M3 PRO** bridges ESP32 chassis at /dev/ttyACM0 — if it crashes, the chassis stops responding. `sudo systemctl restart micro_ros_agent` or reboot
5. **RELIABLE QoS topics fail across subnets** — direct AP connection (10.42.0.0/24) solves this
6. **Pi 5 Docker container accumulates zombie processes** — `docker restart ros2-humble` if things get weird
7. **Rust planner binary path** — i9_planner.py expects `~/SMART3D/target/release/examples/smart_planner_server`. If you recompile, it auto-picks up the new binary

---

*Felix Fang (Tianchonghui Fang) — UConn LinksLab — March 2026*
