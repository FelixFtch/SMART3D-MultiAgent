# SMART-3D Distributed Deployment Guide
## i9/4090 Central Planner + Pi 5 Thin Client

**Date**: February 21, 2026
**Author**: Felix Fang — Auto-generated from deployment session

---

## Architecture Overview

```
┌─────────────────────────────────────────────┐
│  i9 Alienware (10.42.0.160 via USB WiFi)    │
│                                              │
│  ┌──────────────┐  ┌──────────────────────┐ │
│  │ i9_planner.py│  │smart_planner_server  │ │
│  │  (Python)    │──│  (Rust subprocess)   │ │
│  └──────┬───────┘  └──────────────────────┘ │
│         │                                    │
│  ┌──────┴───────┐                            │
│  │   RViz2      │  Visualization             │
│  └──────────────┘                            │
└─────────────────┬───────────────────────────┘
                  │  WiFi AP (10.42.0.0/24)
                  │  CycloneDDS + ROS2 Humble
┌─────────────────┴───────────────────────────┐
│  Pi 5 "robotcar" (10.42.0.1 AP host)        │
│  Docker: ros2-humble (--network=host)        │
│                                              │
│  ┌────────────┐ ┌───────────┐ ┌───────────┐ │
│  │Cartographer│ │ LiDAR M10P│ │ N300 IMU  │ │
│  │   SLAM     │ │ 12Hz scan │ │ 100Hz     │ │
│  └────────────┘ └───────────┘ └───────────┘ │
│  ┌────────────┐ ┌───────────────────────────┐│
│  │pi5_client  │ │cmd_vel_driver (4× DDSM115)││
│  │(relay+ctrl)│ │RS485 motor control        ││
│  └────────────┘ └───────────────────────────┘│
└──────────────────────────────────────────────┘
```

**Why distributed?**
- Pi 5 maxed out running Cartographer (~1.5 cores)
- i9 RRT* build: ~20ms vs Pi 5: ~300ms (15× faster)
- WiFi latency 5-20ms acceptable for planning (not safety-critical)
- Emergency stop runs locally on Pi 5 (<10ms, no network)

---

## Network Setup

### Hardware
- **i9**: Alienware Aurora R16, internal WiFi `wlp3s0` (campus network), USB WiFi `wlx54c9ff02ead9` (RTL8812AU → robotcar AP)
- **Pi 5**: Built-in `wlan0` (campus network), USB WiFi `wlan1` (AP mode, SSID: `robotcar`)

### RTL8812AU USB WiFi Driver Install (i9, one-time)
```bash
sudo apt install -y dkms git build-essential linux-headers-$(uname -r)
git clone https://github.com/aircrack-ng/rtl8812au.git
cd rtl8812au
sudo make dkms_install
sudo modprobe 88XXau
```

### Connect i9 to Pi's AP
```bash
# Wait for USB WiFi to appear
nmcli device status
# Look for wlx... interface (name may vary)

# Connect
nmcli device wifi connect "robotcar" password "23412300" ifname wlx54c9ff02ead9

# Verify
ip addr | grep 10.42
# Should show: inet 10.42.0.160/24
```

### DDS Configuration (CRITICAL)
ROS2 DDS does not work across different subnets without explicit configuration. The Pi runs an AP on `10.42.0.0/24`. Both sides must use CycloneDDS with proper config.

**i9 CycloneDDS config** (`/tmp/cyclone.xml`):
```xml
<CycloneDDS>
  <Domain id="42">
    <General>
      <AllowMulticast>true</AllowMulticast>
      <Interfaces>
        <NetworkInterface name="wlx54c9ff02ead9"/>
      </Interfaces>
    </General>
    <Discovery>
      <Peers>
        <Peer address="10.42.0.1"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

**Pi CycloneDDS config** (inside container `/tmp/cyclone.xml`):
```xml
<CycloneDDS>
  <Domain id="42">
    <General>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="10.42.0.160"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

> **Important**: Pi config must NOT restrict interfaces (no `<Interfaces>` tag), otherwise internal container communication breaks (LiDAR → Cartographer).

### Environment Variables (EVERY terminal needs these)

**i9**:
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclone.xml
```

**Pi (inside container)**:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclone.xml
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
```

### Verify Network
```bash
# i9 → Pi ping
ping 10.42.0.1 -c 2 -I wlx54c9ff02ead9

# i9 can receive Pi's scan data
ros2 topic echo /scan --qos-reliability best_effort --once

# i9 can receive Pi's TF
ros2 topic echo /tf --once
```

---

## Startup Procedure (4 Terminals)

### Terminal 1 — Pi 5 Sensors (SSH to Pi)
```bash
ssh robotcar@10.66.129.126    # password: 234123
# Or: ssh robotcar@10.42.0.1  # via AP network

# Write CycloneDDS config
docker exec ros2-humble bash -c 'cat > /tmp/cyclone.xml << "XMLEOF"
<CycloneDDS>
  <Domain id="42">
    <General>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="10.42.0.160"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
XMLEOF
echo "done"'

# Start all sensors + Cartographer
docker exec -it ros2-humble bash -lc '
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///tmp/cyclone.xml
  export ROS_DOMAIN_ID=42
  source /opt/ros/humble/setup.bash
  /root/start_full.sh
'
```

**Wait for**:
```
=== SCAN ===
average rate: 12.0
=== TF ===
average rate: 4.0
...
ALL STARTED. Drive with robot_base.py. Ctrl+C to stop.
```

### Terminal 2 — Pi 5 Client (new SSH to Pi)
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

**Wait for**: `Pi5 client [car1] | speed=0.2 | waiting for path...`

### Terminal 3 — i9 Planner (local)
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclone.xml
cd ~/SMART3D
python3 i9_planner.py --ros-args -p car_names:="car1"
```

**Wait for**:
```
Central planner for 1 car(s): ['car1']
[car1] Planner started
[car1/rust] [smart_planner_server] v2 ready (arbitrary bounds)
```

### Terminal 4 — i9 RViz (local)
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclone.xml
rviz2
```

**RViz Setup**:
1. Fixed Frame → `map`
2. Add → Map → topic `/map` (Durability: Transient Local)
3. Add → LaserScan → topic `/scan` (Reliability: Best Effort)
4. Add → Path → topic `/car1/path`
5. Add → MarkerArray → topic `/smart/markers`
6. Toolbar → **2D Goal Pose** → click on map to set goal

---

## Troubleshooting

### LiDAR not starting (zombie process)
```bash
# Check for defunct process
docker exec ros2-humble bash -c 'ps aux | grep lslidar'
# If <defunct>, restart container:
docker restart ros2-humble && sleep 5
# Then re-create cyclone.xml and start again
```

### Scan not visible on i9
```bash
# Test with explicit QoS
ros2 topic echo /scan --qos-reliability best_effort --once
```
If no data: check CycloneDDS config, USB WiFi connection, firewall.

### /tf not received but /scan works
DDS RELIABLE transport failing across subnets. Switch to CycloneDDS (not FastDDS). Ensure both sides have `rmw_cyclonedds_cpp`.

### RViz shows "dropping message: frame 'laser_link'"
TF not arriving. Check `/tf` and `/tf_static` topics are visible and streaming.

### Path has only 3 points (straight line through walls)
robot_radius too large for corridor width, or pad too large. Reduce values:
```bash
grep "robot_radius" examples/smart_planner_server.rs
grep "pad = " i9_planner.py
```

### Robot stuck, not rebuilding
Check planner terminal for `STUCK 3s → rebuild` messages. Current settings: 3s stuck timeout, 3x SMART fail → rebuild, 3s panic cooldown.

---

## Files Reference

### i9 Files (~/SMART3D/)
| File | Description |
|------|-------------|
| `i9_planner.py` | Central planner node (~750 lines) |
| `examples/smart_planner_server.rs` | Rust RRT* planner subprocess |
| `/tmp/cyclone.xml` | CycloneDDS config for i9 |

### Pi 5 Files (inside ros2-humble container)
| File | Description |
|------|-------------|
| `/root/pi5_client.py` | Thin client: pose relay + pure pursuit |
| `/root/cmd_vel_driver.py` | Motor RS485 driver |
| `/root/start_full.sh` | Sensor startup script |
| `/tmp/cyclone.xml` | CycloneDDS config for Pi |

### Key Parameters
| Parameter | Location | Current Value | Notes |
|-----------|----------|---------------|-------|
| `robot_radius` | `smart_planner_server.rs` | 0.40m | RRT* collision check |
| `step_size` | `smart_planner_server.rs` | 0.25m | Path point density |
| `pad` (wall inflation) | `i9_planner.py` | 0.20m | Wall expansion for planning |
| `rrt_iterations` | `i9_planner.py` | 15000 | RRT* sample count |
| `speed` | `pi5_client.py` | 0.2 m/s | Driving speed |

---

## RViz Visualization Legend

| Color | Shape | Meaning |
|-------|-------|---------|
| 🔵 Blue | Sphere | Robot current position |
| 🔴 Red | Sphere | Goal position |
| 🟠 Orange | Sphere | Dynamic obstacle (person, box) — auto-expires 0.5s |
| ⬛ Dark gray | Cubes | Known walls (from Cartographer map) |
| 🟢 Green | Line | Planned path |

---

## SMART-3D Algorithm Flow

```
                    ┌──────────────┐
    Set Goal ──────►│  build()     │ RRT* with walls
                    │  ~40ms on i9 │
                    └──────┬───────┘
                           │
                    ┌──────▼───────┐
              ┌────►│  plan_loop() │ 20Hz
              │     │  detect      │
              │     │  dynamic obs │
              │     └──────┬───────┘
              │            │
              │     ┌──────▼───────┐
              │     │  update()    │ Prune + repair
              │     │  ~5ms on i9  │ (SMART core)
              │     └──────┬───────┘
              │            │
              │       ┌────┴────┐
              │       │ Result? │
              │       └────┬────┘
              │            │
              │    safe ───┤─── replanned ──► new path
              │            │
              │    fail ───┤─── 3x fails ──► rebuild()
              │            │
              │    panic ──┴── cooldown 3s ─► rebuild()
              │
              │    stuck 3s ──────────────► rebuild()
              │    map changed 3% ────────► rebuild()
              │
              └────────────────────────────────────
```

---

## Multi-Agent (2+ Cars)

### i9 Side
```bash
# Just add car names
python3 i9_planner.py --ros-args -p car_names:="car1,car2"
```

Each car automatically treats others as dynamic obstacles with real speed estimation.

### Second Pi 5
Same setup as first Pi, but use `car_ns:=car2`:
```bash
python3 /root/pi5_client.py --ros-args -p car_ns:=car2
```

### Coordination Logic (built-in)
```
car1 planning: car2 = SphericalObstacle(pos, radius+0.15, real_speed)
car2 planning: car1 = SphericalObstacle(pos, radius+0.15, real_speed)
→ Automatic mutual avoidance via SMART update
```

---

## Quick Restart Cheatsheet

**Restart just planner** (i9):
```bash
# Ctrl+C in planner terminal, then:
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclone.xml
cd ~/SMART3D
python3 i9_planner.py --ros-args -p car_names:="car1"
```

**Restart RViz** (i9):
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclone.xml
rviz2
```

**Full restart** (everything):
```bash
# 1. Pi: Ctrl+C all terminals
docker restart ros2-humble && sleep 5
# 2. Re-create cyclone.xml in container (see Terminal 1 above)
# 3. Start Terminal 1 (sensors)
# 4. Start Terminal 2 (client)
# 5. Start Terminal 3 (planner)
# 6. Start Terminal 4 (rviz)
```

**Recompile Rust planner** (after changing smart_planner_server.rs):
```bash
cd ~/SMART3D
cargo build --release --example smart_planner_server
# Then restart planner
```
