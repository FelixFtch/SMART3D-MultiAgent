#!/usr/bin/env python3
"""
SMART-3D Multi-Agent Upgrade Patches
=====================================

This file contains the EXACT code changes needed in i9_planner.py
and smart_planner_server.rs to support per-agent parameters.

Apply these changes on the i9 Alienware.

Felix: Send me i9_planner.py and smart_planner_server.rs and
I'll produce the complete modified files instead of patches.
"""

# =====================================================================
# PATCH 1: i9_planner.py — Per-agent robot_radius and speed
# =====================================================================
#
# FIND this (around the parameter declarations):
#
#   self.robot_radius = 0.40
#
# REPLACE WITH:
#
PATCH_1_NEW = """
        # Per-agent robot radius (meters)
        # Format: "car1:0.40,m3pro:0.25,balbot:0.15"
        radius_str = self.get_parameter('agent_radii').value \
            if self.has_parameter('agent_radii') else ''
        self.agent_radii = {}
        if radius_str:
            for pair in radius_str.split(','):
                parts = pair.strip().split(':')
                if len(parts) == 2:
                    self.agent_radii[parts[0].strip()] = float(parts[1])

        # Per-agent speed limits
        speed_str = self.get_parameter('agent_speeds').value \
            if self.has_parameter('agent_speeds') else ''
        self.agent_speeds = {}
        if speed_str:
            for pair in speed_str.split(','):
                parts = pair.strip().split(':')
                if len(parts) == 2:
                    self.agent_speeds[parts[0].strip()] = float(parts[1])

        # Default fallbacks
        self.default_robot_radius = 0.40
        self.default_speed = 0.2
"""

# Also ADD these parameter declarations near the top:
PATCH_1_PARAMS = """
        self.declare_parameter('agent_radii', 'car1:0.40,m3pro:0.25')
        self.declare_parameter('agent_speeds', 'car1:0.20,m3pro:0.25')
"""

# =====================================================================
# PATCH 2: i9_planner.py — Pass robot_radius in Rust build command
# =====================================================================
#
# FIND the build command (where it sends JSON to the Rust subprocess):
#
#   cmd = {'cmd': 'build', 'from': [...], 'goal': [...], 'walls': walls,
#          'world_size': ..., 'speed': ..., 'iters': ...}
#
# REPLACE WITH (add robot_radius field):
#
PATCH_2_BUILD = """
        car_radius = self.agent_radii.get(car_name, self.default_robot_radius)
        car_speed = self.agent_speeds.get(car_name, self.default_speed)

        cmd = {
            'cmd': 'build',
            'from': [rx, ry],
            'goal': [gx, gy],
            'walls': walls,
            'world_size': self.ws,
            'speed': car_speed,
            'iters': self.rrt_iterations,
            'robot_radius': car_radius,   # NEW: per-agent radius
        }
"""

# =====================================================================
# PATCH 3: i9_planner.py — Per-agent obstacle radius in coordination
# =====================================================================
#
# FIND where other cars are added as obstacles:
#
#   SphericalObstacle(pos, radius+0.15, real_speed)
#
# REPLACE WITH:
#
PATCH_3_OBSTACLE = """
        # Use the OTHER car's actual radius + safety margin
        other_radius = self.agent_radii.get(other_car_name, self.default_robot_radius)
        obstacle = SphericalObstacle(pos, other_radius + 0.15, real_speed)
"""

# =====================================================================
# PATCH 4: i9_planner.py — Wall inflation pad per agent
# =====================================================================
#
# FIND:
#   pad = 0.20  (or self.pad)
#
# REPLACE WITH:
#
PATCH_4_PAD = """
        # Pad should be at least robot_radius * 0.5 for safety
        car_radius = self.agent_radii.get(car_name, self.default_robot_radius)
        pad = max(0.10, car_radius * 0.5)
"""

# =====================================================================
# PATCH 5: smart_planner_server.rs — Accept robot_radius from JSON
# =====================================================================
#
# FIND in the Rust code (build command handler):
#
#   let robot_radius = 0.40_f64;
#
# REPLACE WITH:
#
PATCH_5_RUST = """
    // Read robot_radius from JSON, fallback to 0.40
    let robot_radius = cmd.get("robot_radius")
        .and_then(|v| v.as_f64())
        .unwrap_or(0.40_f64);
"""

# =====================================================================
# PATCH 6: i9 CycloneDDS config — Add M3 PRO peer
# =====================================================================
#
# Update /tmp/cyclone.xml on i9 to include M3 PRO's IP.
# Run m3pro_setup.sh on M3 PRO first to get its IP.
#
CYCLONE_I9_XML = """
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
        <Peer address="10.42.0.XXX"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
"""
# ^^^ Replace 10.42.0.XXX with M3 PRO's actual IP from m3pro_setup.sh

# =====================================================================
# USAGE (i9 side):
# =====================================================================
#
# Single car (existing, unchanged):
#   python3 i9_planner.py --ros-args -p car_names:="car1"
#
# Two cars:
#   python3 i9_planner.py --ros-args \
#       -p car_names:="car1,m3pro" \
#       -p agent_radii:="car1:0.40,m3pro:0.25" \
#       -p agent_speeds:="car1:0.20,m3pro:0.25"
#
# Three cars (future):
#   python3 i9_planner.py --ros-args \
#       -p car_names:="car1,m3pro,balbot" \
#       -p agent_radii:="car1:0.40,m3pro:0.25,balbot:0.15" \
#       -p agent_speeds:="car1:0.20,m3pro:0.25,balbot:0.15"
#

# =====================================================================
# FULL STARTUP SEQUENCE (Two-Car)
# =====================================================================
#
# --- M3 PRO (3 terminals via SSH to jetson@10.66.147.9) ---
#
# Terminal M1: micro-ROS agent
#   bash ~/start_agent_smart3d.sh
#   (wait 5s)
#
# Terminal M2: Cartographer + sensors
#   bash ~/run_cartographer_smart3d.sh
#   (wait for "average rate: 12.0" on scan)
#
# Terminal M3: SMART-3D client
#   bash ~/run_m3pro_client.sh
#   (wait for "waiting for path...")
#
# --- Pi 5 Car (2 terminals via SSH to robotcar@10.66.129.126) ---
#
# Terminal P1: Sensors + Cartographer
#   (existing procedure from deployment guide)
#
# Terminal P2: pi5_client
#   (existing procedure from deployment guide)
#
# --- i9 Alienware (2 terminals, local) ---
#
# Terminal I1: Planner
#   source /opt/ros/humble/setup.bash
#   export ROS_DOMAIN_ID=42
#   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#   export CYCLONEDDS_URI=file:///tmp/cyclone.xml
#   cd ~/SMART3D
#   python3 i9_planner.py --ros-args \
#       -p car_names:="car1,m3pro" \
#       -p agent_radii:="car1:0.40,m3pro:0.25" \
#       -p agent_speeds:="car1:0.20,m3pro:0.25"
#
# Terminal I2: RViz
#   source /opt/ros/humble/setup.bash
#   export ROS_DOMAIN_ID=42
#   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#   export CYCLONEDDS_URI=file:///tmp/cyclone.xml
#   rviz2
#   → Add /m3pro/path (Path), /car1/path (Path)
#   → Add /smart/markers (MarkerArray)
#

if __name__ == '__main__':
    print("This file contains patches, not runnable code.")
    print("Apply the patches to i9_planner.py and smart_planner_server.rs")
    print("Or send me those files and I'll produce the complete modified versions.")
