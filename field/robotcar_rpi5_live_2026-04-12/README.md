# RobotCar Pi5 Live Snapshot

Source machine:
- Host: `robotcar@raspberrypi`
- IP: `10.66.127.173`
- Snapshot date: `2026-04-12`

Included:
- `cartographer_config/`: live Cartographer bring-up, Lua config, launch file, STP-23L helper scripts
- `deploy_smart_nav.sh`: Pi 5 SMART-3D deployment/bootstrap script
- `robot_base.py`: serial chassis control utility
- `checkusb.py`: serial/USB inspection helper
- `ros2_ws_src/hipnuc_*`: IMU/GNSS ROS2 packages copied from the live robot

Excluded on purpose:
- disk images, logs, recordings, transcripts, caches
- auth tokens, SSH keys, certificates, Wi-Fi secrets
- `ros2_ws/build`, `ros2_ws/install`, `ros2_ws/log`

This folder is a field snapshot, not a cleaned package release.
