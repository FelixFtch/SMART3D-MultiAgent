# Smart3D Vendor Reference Snapshots

Date: 2026-04-18

This directory stores local reference snapshots of official or primary-source upstream repos that matter for the Smart3D hardware stack.

These snapshots are for inspection and reproducibility. They are not automatically wired into the runtime.

## Downloaded references

### 1. Leishen / LSLiDAR M10P ROS2 driver

- Local path: `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/vendor/Lslidar_ROS2_driver`
- Upstream: `https://github.com/Lslidar/Lslidar_ROS2_driver.git`
- Branch used: `M10P/N10P`
- Snapshot commit: `8bb760c8f66c1b6964caaf3cf6f7a2a0ea24cefa`
- Purpose: official ROS2 source reference for the Smart3D M10P track

### 2. HiPNUC products repo

- Local path: `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/vendor/hipnuc_products`
- Upstream: `https://github.com/hipnuc/products.git`
- Branch used: repository default
- Snapshot commit: `d257b57a7c66092aa6735d77bba49745be241c20`
- Purpose: official upstream example reference for the HiPNUC ROS2 track

### 3. Waveshare DDSM example repo

- Local path: `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/vendor/ddsm_example`
- Upstream: `https://github.com/waveshareteam/ddsm_example.git`
- Branch used: repository default
- Snapshot commit: `79bfd6f21f9f8f15c231797184a3a93bf881c8a7`
- Purpose: official reference example for DDSM command/protocol behavior

### 4. LDROBOT STP23L ROS2 reference

- Local path: `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/vendor/ldlidar_stl_ros2`
- Upstream: `https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git`
- Branch used: repository default
- Snapshot commit: `bf668a89baf722a787dadc442860dcbf33a82f5a`
- Purpose: official ROS2 reference for LDROBOT STL/STP-style DTOF devices

### 5. LDROBOT unified SDK

- Local path: `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/vendor/ldlidar_sdk`
- Upstream: `https://github.com/ldrobotSensorTeam/ldlidar_sdk.git`
- Branch used: repository default
- Snapshot commit: `6148cd3c33ae39adbd7e12faf82567fa8f8e7b0d`
- Purpose: official SDK reference for LDROBOT lidar/range devices

## Usage note

For the current Smart3D Jetson effort:

- use these repos as upstream reference material
- do not blindly replace the Pi-proven runtime with upstream HEAD
- promote changes into the Smart3D runtime only after the exact device path is validated on hardware

