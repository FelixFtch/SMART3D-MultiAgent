# Smart3D Hardware Official Replication Master

Date: 2026-04-18

This is the top-level hardware replication index for the current Smart3D Jetson Orin NX 16GB effort.

Read this file first, then open the per-hardware reports under the same directory.

## 1. What was done in this pass

Six hardware/platform tracks were investigated in parallel:

1. DDSM115 / RS485 drivetrain
2. Leishen M10P LiDAR
3. HiPNUC / WHEELTEC IMU
4. STP23L single-point range sensor
5. Logitech BRIO camera
6. Jetson Orin NX 16GB platform + ROS 2 Humble

Each track now has its own report in:

- [HARDWARE_DDSM115_RS485.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/research/HARDWARE_DDSM115_RS485.md)
- [HARDWARE_M10P_LIDAR.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/research/HARDWARE_M10P_LIDAR.md)
- [HARDWARE_HIPNUC_IMU.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/research/HARDWARE_HIPNUC_IMU.md)
- [HARDWARE_STP23L_RANGE.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/research/HARDWARE_STP23L_RANGE.md)
- [HARDWARE_LOGITECH_BRIO_CAMERA.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/research/HARDWARE_LOGITECH_BRIO_CAMERA.md)
- [HARDWARE_JETSON_PLATFORM_AND_ROS2.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/research/HARDWARE_JETSON_PLATFORM_AND_ROS2.md)

## 2. Current master verdict per hardware

### 2.1 Jetson platform

- Use native ROS 2 Humble on Jetson.
- Do not start by recreating the Pi `ros2-humble` container.
- NVIDIA platform packages should come from NVIDIA apt.
- ROS core packages should come from ROS apt.
- Vendor/device-specific pieces should be built from source where needed.

Primary report:

- [HARDWARE_JETSON_PLATFORM_AND_ROS2.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/research/HARDWARE_JETSON_PLATFORM_AND_ROS2.md)

### 2.2 M10P LiDAR

- This is part of the required core chain.
- Official ROS2 path is the vendor repo `Lslidar_ROS2_driver`.
- Correct branch for this robot is `M10P/N10P`.
- Smart3D should keep using the `M10_P` code path and `/scan`.
- Jetson should keep using stable `/dev/serial/by-id/...`, not volatile `/dev/ttyUSB0`.

Primary report:

- [HARDWARE_M10P_LIDAR.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/research/HARDWARE_M10P_LIDAR.md)

### 2.3 HiPNUC IMU

- This is part of the required core chain.
- Smart3D should first replicate the Pi-stable promoted copy, not jump to the newest upstream example.
- Keep `/IMU_data` and `frame_id=base_link` until the full chain is revalidated.
- Jetson should use stable `/dev/serial/by-id/...`.

Primary report:

- [HARDWARE_HIPNUC_IMU.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/research/HARDWARE_HIPNUC_IMU.md)

### 2.4 DDSM115 / RS485

- This is part of the required active-motion chain.
- Official vendor documentation exists, but there is no official ROS driver path that matches Smart3D.
- Smart3D is correctly using its own raw RS485 + ROS bridge model.
- The biggest unresolved field issue is still "serial opens but feedback is empty".

Primary report:

- [HARDWARE_DDSM115_RS485.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/research/HARDWARE_DDSM115_RS485.md)

### 2.5 STP23L

- This is an auxiliary sensor, not part of the accepted goal-test baseline.
- The local parser and protocol alignment look credible.
- It should not be inserted into the default bring-up until the core chain stays stable.

Primary report:

- [HARDWARE_STP23L_RANGE.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/research/HARDWARE_STP23L_RANGE.md)

### 2.6 Logitech BRIO

- The camera is physically healthy on Jetson.
- It is not part of the accepted Smart3D baseline.
- Historic usage was an optional UDP stream path, not a default ROS camera path.
- Treat it as phase 2 after the core robot stack is stable.

Primary report:

- [HARDWARE_LOGITECH_BRIO_CAMERA.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/research/HARDWARE_LOGITECH_BRIO_CAMERA.md)

## 3. Official upstream references downloaded locally

The following official or primary-source upstream repos were downloaded into:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/vendor`

They are indexed in:

- [vendor/README.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/vendor/README.md)

## 4. Jetson package state added in this pass

The Jetson now has the optional BRIO/ROS camera packages installed:

- `ros-humble-v4l2-camera`
- `ros-humble-usb-cam`
- `ros-humble-image-transport`
- `ros-humble-image-transport-plugins`
- `ros-humble-camera-info-manager`
- `ros-humble-image-tools`
- `ros-humble-rqt-image-view`
- `ros-humble-gscam`
- `v4l-utils`

These do not make camera part of the default Smart3D runtime. They only remove future setup friction.

## 5. What is already proven vs not yet proven

### Proven enough

- Jetson native ROS 2 Humble platform shape
- M10P -> `/scan`
- HiPNUC -> `/IMU_data`
- Cartographer -> `/map`
- campus bridge -> `/map_campus`
- i9 planner -> Jetson goal-test shape

### Still not fully proven

- DDSM115 real feedback / motion confirmation on Jetson
- STP23L physical identity and production role
- BRIO as a real Smart3D ROS topic path instead of optional video side-channel

## 6. Recommended full-stack reproduction order

Do not bring every optional device into the chain at once.

Recommended order:

1. Platform baseline on Jetson native ROS 2 Humble
2. M10P LiDAR
3. HiPNUC IMU
4. Cartographer + map bridge + `pi5_client.py`
5. i9 planner goal-test
6. DDSM115 low-speed physical smoke test with human watching
7. STP23L as auxiliary sensor
8. BRIO camera as optional phase-2 capability

## 7. The two most important operational rules

1. Reproduce the proven Pi behavior before trying to "improve" it with newer upstream code.
2. Do not let optional hardware block the core goal-test chain.

## 8. Related canonical docs

- [SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md)
- [SMART3D_JETSON_ORIN_NX16_MIGRATION_FROM_PI5.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_JETSON_ORIN_NX16_MIGRATION_FROM_PI5.md)
- [SMART3D_CAMPUS_RUNTIME.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_CAMPUS_RUNTIME.md)

