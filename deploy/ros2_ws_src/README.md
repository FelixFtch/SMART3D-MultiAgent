# RobotCar Pi5 ROS2 Sensor Workspace

This folder is the promoted Pi5 sensor-side ROS2 source tree from the live
`robotcar@raspberrypi` snapshot.

Included packages:

- `hipnuc_lib_package`
- `hipnuc_imu`
- `hipnuc_gnss`

Recommended Pi5 layout:

```bash
mkdir -p ~/ros2_ws/src
cp -R deploy/ros2_ws_src/hipnuc_* ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select hipnuc_lib_package hipnuc_imu hipnuc_gnss
```

Notes:

- keep `field/robotcar_rpi5_live_2026-04-12/` as the immutable provenance copy
- use `deploy/ros2_ws_src/` as the canonical repo-side source for Pi5 sensor packages
