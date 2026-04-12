# SMART-3D Two-Car Deployment Cheatsheet
## Pi5 Car (car1) + M3 PRO (m3pro)

**Date**: February 21, 2026

---

## Network Topology

```
                    i9 Alienware
                    wlx...: 10.42.0.160
                         │
              robotcar AP (10.42.0.0/24)
               ┌─────────┴─────────┐
               │                   │
         Pi 5 Car              M3 PRO
    wlan1: 10.42.0.1      wlx...: 10.42.0.XXX
         (AP host)            (USB WiFi)
```

All ROS2 traffic on `DOMAIN_ID=42`, CycloneDDS.

---

## One-Time Setup

### M3 PRO
```bash
# SSH in
ssh jetson@10.66.147.9   # pw: yahboom

# Run setup (after USB WiFi plugged in)
bash m3pro_setup.sh

# Note the IP it prints (10.42.0.XXX)
```

### i9 — Update CycloneDDS peers
```bash
# Edit /tmp/cyclone.xml, add M3 PRO peer:
#   <Peer address="10.42.0.XXX"/>
```

---

## Startup Sequence (7 Terminals)

### ① M3 PRO — micro-ROS Agent
```bash
ssh jetson@10.66.147.9
bash ~/start_agent_smart3d.sh
# Wait 5s
```

### ② M3 PRO — Cartographer + Sensors
```bash
ssh jetson@10.66.147.9
bash ~/run_cartographer_smart3d.sh
# Wait for scan + TF
```

### ③ M3 PRO — SMART-3D Client
```bash
ssh jetson@10.66.147.9
bash ~/run_m3pro_client.sh
# Wait for: "waiting for path..."
```

### ④ Pi5 — Sensors + Cartographer
```bash
ssh robotcar@10.66.129.126   # pw: 234123
# (existing procedure — see SMART3D_Deployment_Guide.md Terminal 1)
```

### ⑤ Pi5 — pi5_client
```bash
ssh robotcar@10.66.129.126
# (existing procedure — Terminal 2)
```

### ⑥ i9 — Planner (multi-agent)
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclone.xml
cd ~/SMART3D
python3 i9_planner.py --ros-args \
    -p car_names:="car1,m3pro" \
    -p agent_radii:="car1:0.40,m3pro:0.25" \
    -p agent_speeds:="car1:0.20,m3pro:0.25"
```

### ⑦ i9 — RViz
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclone.xml
rviz2
```

**RViz additions for M3 PRO:**
- Path → `/m3pro/path` (different color from `/car1/path`)
- Pose → `/m3pro/pose`

---

## Verify Communication

```bash
# On i9, check M3 PRO topics are visible:
ros2 topic list | grep m3pro

# Expected:
#   /m3pro/pose
#   /m3pro/scan
#   /m3pro/path      (after planner starts)
#   /m3pro/goal_pose

# Check M3 PRO pose is streaming:
ros2 topic hz /m3pro/pose
# Should be ~10 Hz

# Check M3 PRO scan relay:
ros2 topic echo /m3pro/scan --qos-reliability best_effort --once
```

---

## Key Parameters

| Parameter | car1 (Pi5) | m3pro (M3 PRO) | balbot (future) |
|-----------|-----------|----------------|-----------------|
| robot_radius | 0.40 m | 0.25 m | 0.15 m |
| speed | 0.20 m/s | 0.25 m/s | 0.15 m/s |
| e-stop lidar | 0.30 m | 0.30 m | 0.20 m |
| e-stop depth | — | 0.33 m | — |

---

## Troubleshooting

### M3 PRO topics not visible on i9
1. Check USB WiFi connected: `ip addr | grep 10.42`
2. Check CycloneDDS: `echo $CYCLONEDDS_URI` (all terminals)
3. Check domain: `echo $ROS_DOMAIN_ID` (must be 42 everywhere)
4. Ping test: `ping 10.42.0.XXX -I wlx54c9ff02ead9`

### M3 PRO motors not responding
- `start_agent_smart3d.sh` must use DOMAIN_ID=42
- micro-ROS agent must be running BEFORE Cartographer
- Check: `ros2 topic list | grep cmd_vel`

### Two cars colliding
- Increase `agent_radii` values
- Check planner log for mutual obstacle detection:
  `[car1] dynamic obs: m3pro at (x,y) r=0.40`

### M3 PRO oscillating / not following path
- Reduce speed: `-p robot_speed:=0.15`
- M3 PRO is heavier with arm — may need slower acceleration
- Check Cartographer localization: `ros2 topic hz /tf`
