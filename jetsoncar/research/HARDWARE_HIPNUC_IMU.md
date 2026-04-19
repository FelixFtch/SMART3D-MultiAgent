# HiPNUC / WHEELTEC IMU Track for Smart3D

This note focuses on the IMU path only: hardware identity, upstream driver/package reality, the Pi baseline, and what Jetson is actually doing now.

## 1. Hardware identity

What Smart3D currently treats as the IMU:

- The Pi/Jetson runtime comments call it a `HiPNUC N300 Pro` IMU:
  - [carto_bringup.sh](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/carto_bringup.sh)
  - [robotcar_2d.lua](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d.lua)
  - [robotcar_2d_odom.lua](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d_odom.lua)
- The actual decoder used in Smart3D is the HiPNUC binary protocol decoder:
  - [hipnuc_dec.h](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/ros2_ws_src/hipnuc_lib_package/include/hipnuc_lib_package/hipnuc_dec.h)
  - [hipnuc_dec.c](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/ros2_ws_src/hipnuc_lib_package/src/hipnuc_dec.c)

Inference from code: regardless of the reseller label, the Smart3D stack is using a HiPNUC protocol driver, not a Witmotion/YESENSE-style driver.

Current Jetson device identity observed on the live box:

- IMU stable path: `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B030655-if00`
- Kernel node behind it: `/dev/ttyACM1`
- That is not the official HiPNUC example path (`/dev/ttyUSB0` on a CP210x bridge). It means the current unit is enumerating through a different USB-serial bridge and should be treated as a stable-by-id serial device, not as a hard-coded `ttyUSB0`.

## 2. Official docs and packages

### Official HiPNUC sources I found

- HiPNUC resource page for software examples and drivers: [hipnuc.com resource page](https://www.hipnuc.com/resource_hi02.html)
- HiPNUC product examples repo: [hipnuc/products](https://github.com/hipnuc/products)
- Official ROS2 example readme in that repo: [examples/ROS2/Readme.md](https://raw.githubusercontent.com/hipnuc/products/master/examples/ROS2/Readme.md)
- Official ROS2 IMU config in that repo: [official hipnuc_config.yaml](https://raw.githubusercontent.com/hipnuc/products/master/examples/ROS2/hipnuc_ws/src/hipnuc_imu/config/hipnuc_config.yaml)
- Official ROS2 IMU package manifest: [official hipnuc_imu/package.xml](https://raw.githubusercontent.com/hipnuc/products/master/examples/ROS2/hipnuc_ws/src/hipnuc_imu/package.xml)
- Official ROS2 GNSS package manifest: [official hipnuc_gnss/package.xml](https://raw.githubusercontent.com/hipnuc/products/master/examples/ROS2/hipnuc_ws/src/hipnuc_gnss/package.xml)
- Official ROS2 IMU source: [official serial_port.cpp](https://raw.githubusercontent.com/hipnuc/products/master/examples/ROS2/hipnuc_ws/src/hipnuc_imu/src/serial_port.cpp)
- HiPNUC datasheet example showing ROS examples and UART output: [HI02 datasheet](https://www.hipnuc.com/static/file/hi02/HI02_Data_Sheet_1.0_EN.pdf)

### Official WHEELTEC sources I found

- WHEELTEC/Seeed H30 page: [WHEELTEC IMU Sensor wiki](https://wiki.seeedstudio.com/wheeltec_imu/)
- Its documentation repo: [Seeed-Projects/WHEELTEC-H30-IMU](https://github.com/Seeed-Projects/WHEELTEC-H30-IMU)

Important conclusion: the public WHEELTEC H30 docs point to a different SDK track. Smart3D is not using that SDK. Smart3D is using the HiPNUC ROS2 example family.

### What the official HiPNUC ROS2 example says

From the official repo/readme:

- Official tested environment is `Ubuntu 20.04` + `ROS2 Foxy`, not Humble.
- Official serial expectation is:
  - USB-UART bridge appears as `/dev/ttyUSBx`
  - default port `/dev/ttyUSB0`
  - default baud `115200`
- Official launch examples:
  - `ros2 launch hipnuc_imu imu_spec_msg.launch.py`
  - `ros2 launch hipnuc_gnss nav_spec_msg.launch.py`
- Official README says the ROS2 workspace builds `hipnuc_imu` and `hipnuc_gnss`.

From the official package manifests and source:

- `hipnuc_imu` depends on `hipnuc_lib_package`, so for a clean build on Jetson the real set is:
  - `hipnuc_lib_package`
  - `hipnuc_imu`
  - optional `hipnuc_gnss`
- `hipnuc_gnss` additionally needs `gps_msgs`

## 3. Official docs vs official code: they do not fully agree

This was the biggest upstream surprise.

The official ROS2 README and the official current ROS2 source do not perfectly match:

- Official README still shows:
  - `frame_id: "base_link"`
  - only `/IMU_data`
  - no euler/magnetic toggles
- Official current `hipnuc_config.yaml` in the repo now shows:
  - `frame_id: "imu_link"`
  - `imu_switch`
  - `euler_switch`
  - `magnetic_switch`
  - `/euler_data`
  - `/magnetic_data`
- Official current `serial_port.cpp` uses `rclcpp::SensorDataQoS()` and publishes IMU, euler, and magnetic data.

Recommended reading order: trust the actual source/config in the repo before trusting the README screenshots.

## 4. Canonical local Smart3D files inspected

Promoted IMU source packages and build hints:

- [README.md](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/ros2_ws_src/README.md)
- [hipnuc_imu/package.xml](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/ros2_ws_src/hipnuc_imu/package.xml)
- [hipnuc_imu/CMakeLists.txt](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/ros2_ws_src/hipnuc_imu/CMakeLists.txt)
- [hipnuc_imu/config/hipnuc_config.yaml](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/ros2_ws_src/hipnuc_imu/config/hipnuc_config.yaml)
- [hipnuc_imu/launch/imu_spec_msg.launch.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/ros2_ws_src/hipnuc_imu/launch/imu_spec_msg.launch.py)
- [hipnuc_imu/src/serial_port.cpp](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/ros2_ws_src/hipnuc_imu/src/serial_port.cpp)
- [hipnuc_gnss/package.xml](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/ros2_ws_src/hipnuc_gnss/package.xml)
- [hipnuc_gnss/config/hipnuc_config.yaml](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/ros2_ws_src/hipnuc_gnss/config/hipnuc_config.yaml)
- [hipnuc_gnss/src/ins_module.cpp](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/ros2_ws_src/hipnuc_gnss/src/ins_module.cpp)
- [hipnuc_lib_package/package.xml](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/ros2_ws_src/hipnuc_lib_package/package.xml)

Pi baseline runtime files:

- [start_full.sh](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/start_full.sh)
- [start_all.sh](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/start_all.sh)
- [imu_qos_bridge.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/imu_qos_bridge.py)
- [carto_bringup.sh](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/carto_bringup.sh)
- [cartographer_launch.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/cartographer_launch.py)
- [robotcar_2d.lua](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d.lua)
- [robotcar_2d_odom.lua](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d_odom.lua)

Canonical Smart3D tree files that wire the Jetson copy:

- [start_full_native.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh)
- [mk_imu_dev.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/mk_imu_dev.sh)
- [robotcar.urdf](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/robotcar.urdf)
- [deploy_runtime_to_jetson.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/deploy_runtime_to_jetson.sh)
- [pi_ros_watchdog.py](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/pi_ros_watchdog.py)
- [smart3d_campus.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh)

## 5. What package(s) should be built on Jetson Humble

For the IMU-only path on Jetson:

1. Install ROS 2 Humble.
2. Install build helpers:
   - `python3-colcon-common-extensions`
   - `python3-serial`
3. Build these packages from the promoted Smart3D source tree:
   - `hipnuc_lib_package`
   - `hipnuc_imu`
4. Only if this unit is also being used as a GNSS/INS device, add:
   - `ros-humble-gps-msgs`
   - `hipnuc_gnss`

For the full Smart3D runtime, also ensure:

- `ros-humble-robot-state-publisher`
- Cartographer-related packages are present, but those belong to the SLAM track, not the IMU package itself.

Recommended Humble build command:

```bash
source /opt/ros/humble/setup.bash
cd ~/SMART3D/ws_imu
colcon build --symlink-install --packages-select hipnuc_lib_package hipnuc_imu hipnuc_gnss
```

If GNSS is not needed, use:

```bash
source /opt/ros/humble/setup.bash
cd ~/SMART3D/ws_imu
colcon build --symlink-install --packages-select hipnuc_lib_package hipnuc_imu
```

## 6. Data topics, QoS, and frame usage

### Official current upstream behavior

From the official current repo:

- IMU topic: `/IMU_data`
- Optional extra topics: `/euler_data`, `/magnetic_data`
- Default frame in current config: `imu_link`
- Publisher QoS: `SensorDataQoS()` in source

### Pi baseline behavior

From the promoted Smart3D copy:

- IMU topic: `/IMU_data`
- Frame: `base_link`
- No euler/magnetic outputs in the promoted IMU node
- Publisher QoS is inferred from code as default `KEEP_LAST(20)` publisher QoS, which means the promoted copy is not using `SensorDataQoS()`

Files:

- [serial_port.cpp](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/ros2_ws_src/hipnuc_imu/src/serial_port.cpp)
- [hipnuc_config.yaml](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/ros2_ws_src/hipnuc_imu/config/hipnuc_config.yaml)

### Jetson current runtime behavior

Current Jetson replication follows the Pi baseline, not the latest upstream:

- IMU topic: `/IMU_data`
- Frame: `base_link`
- Port override: stable by-id path
- Still launches `imu_qos_bridge.py`

Files:

- [start_full_native.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh)
- [imu_qos_bridge.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/imu_qos_bridge.py)

### Cartographer and frame choice

Current Smart3D SLAM is explicitly aligned to `base_link` as the IMU message frame:

- [cartographer_launch.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/cartographer_launch.py) remaps `imu -> /IMU_data`
- [robotcar_2d.lua](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d.lua) sets `tracking_frame = "base_link"`
- [robotcar_2d_odom.lua](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d_odom.lua) also sets `tracking_frame = "base_link"`

### robot_state_publisher usage

The runtime also publishes a URDF with an `imu_link`:

- [robotcar.urdf](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/robotcar.urdf)
- [smart3d_campus.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh)
- [start_full_native.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh)

But current IMU messages are still stamped as `base_link`, not `imu_link`.

So the current system has a modeled `imu_link` in TF, but the IMU driver itself does not use it.

## 7. Pi baseline behavior

The Pi baseline is very stable and very explicit:

1. A helper script creates `/dev/imu`.
2. The IMU node always opens `/dev/imu`.
3. It publishes `/IMU_data` with `frame_id:=base_link`.
4. Cartographer subscribes to `/IMU_data`.
5. `robot_state_publisher` is started separately for TF/RViz completeness.

Relevant files:

- [start_full.sh](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/start_full.sh)
- [start_all.sh](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/start_all.sh)
- [carto_bringup.sh](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/carto_bringup.sh)
- [smart3d_campus.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh)

Pi baseline IMU launch command:

```bash
ros2 run hipnuc_imu talker --ros-args \
  -p serial_port:=/dev/imu \
  -p baud_rate:=115200 \
  -p imu_topic:=/IMU_data \
  -p frame_id:=base_link
```

## 8. Jetson replication steps

The safest Jetson replication is to copy the Pi behavior first and only then consider upgrading to the official latest upstream driver.

### Step A: build the promoted Smart3D IMU workspace

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/SMART3D/ws_imu/src
cp -R /path/to/promoted/hipnuc_* ~/SMART3D/ws_imu/src/
cd ~/SMART3D/ws_imu
colcon build --symlink-install --packages-select hipnuc_lib_package hipnuc_imu hipnuc_gnss
```

Promoted source tree:

- [README.md](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/ros2_ws_src/README.md)

### Step B: bind the real serial device by stable path

Current Jetson runtime already does the right thing here:

- authoritative port in runtime: `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B030655-if00`
- local helper still creates `/dev/imu`, but the runtime now overrides directly to the by-id path

Files:

- [mk_imu_dev.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/mk_imu_dev.sh)
- [start_full_native.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh)

### Step C: launch just the IMU first

```bash
source /opt/ros/humble/setup.bash
source ~/SMART3D/ws_imu/install/setup.bash
ros2 run hipnuc_imu talker --ros-args \
  -p serial_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B030655-if00 \
  -p baud_rate:=115200 \
  -p imu_topic:=/IMU_data \
  -p frame_id:=base_link
```

### Step D: validate just the IMU

```bash
ros2 topic echo /IMU_data --once
ros2 topic hz /IMU_data
ros2 topic info -v /IMU_data
tail -n 80 /tmp/imu_talker.log
```

### Step E: only then launch the full Smart3D runtime

```bash
cd /home/jetson/SMART3D/jetson_runtime
export CYCLONEDDS_URI=file:///tmp/cyclone-campus-jetson.xml
START_ACTIVE=0 START_WATCHDOG=1 START_ODOM=1 ./start_full_native.sh
```

## 9. Exact validation commands I trust

### Official standalone-style validation

```bash
source /opt/ros/humble/setup.bash
source ~/SMART3D/ws_imu/install/setup.bash
ros2 run hipnuc_imu talker --ros-args \
  -p serial_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B030655-if00 \
  -p baud_rate:=115200 \
  -p imu_topic:=/IMU_data \
  -p frame_id:=base_link
```

Then in another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/SMART3D/ws_imu/install/setup.bash
ros2 topic echo /IMU_data --once
ros2 topic hz /IMU_data
ros2 topic info -v /IMU_data
```

### Smart3D full-stack validation

```bash
tail -n 80 /tmp/imu_talker.log
tail -n 80 /tmp/imu_bridge.log
tail -n 80 /tmp/cartographer.log
```

Sanity checks:

```bash
ps -ef | grep hipnuc_imu | grep -v grep
ls -l /dev/serial/by-id | grep 5B0B030655
```

## 10. Common failure modes

### 10.1 Official path assumption does not match Jetson reality

Official examples assume:

- CP210x USB bridge
- `/dev/ttyUSB0`

Current Jetson reality:

- device shows up under `/dev/ttyACM1`
- stable alias is `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B030655-if00`

Recommended fix: do not hard-code `/dev/ttyUSB0` on Jetson.

### 10.2 Upstream README and upstream code drift

The official README and current official code disagree on `frame_id` and extra topics.

Recommended fix: pin the runtime parameters explicitly in launch commands and commit the exact chosen behavior.

### 10.3 Current promoted Smart3D copy is not the latest upstream code

Observed differences between official current source and promoted Smart3D copy:

- promoted copy strips euler/magnetic publishers
- promoted copy uses simpler publisher creation
- promoted copy keeps `frame_id` centered on `base_link`
- promoted copy is closer to the Pi-stable path than to the current upstream example

Inference: the local promoted copy is a deliberately simplified, older snapshot of the official Hipnuc ROS2 example.

### 10.4 `imu_qos_bridge.py` looks vestigial in the current Smart3D tree

In the inspected trees, only the bridge itself references `/imu_reliable`:

- [imu_qos_bridge.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/imu_qos_bridge.py)

I did not find any inspected Smart3D consumer subscribing to `/imu_reliable`.

That means the bridge currently appears unused for the main Smart3D path.

### 10.5 Frame mismatch: `imu_link` exists, but messages are stamped `base_link`

The URDF defines `imu_link`, but the current Pi-compatible launch keeps IMU messages on `base_link`.

This is not automatically wrong, but it means:

- `robot_state_publisher` does not "fix" IMU frames
- changing to `imu_link` later would require full Cartographer/TF revalidation

### 10.6 Duplicate IMU publishers

On the current Jetson, I observed multiple `hipnuc_imu talker` processes still present at once.

Recommended validation:

```bash
ps -ef | grep hipnuc_imu | grep -v grep
```

Recommended operational fix: always verify there is exactly one IMU publisher before judging SLAM behavior.

### 10.7 `gps_msgs` missing when GNSS package is built

If `hipnuc_gnss` is part of the workspace on Humble, install `ros-humble-gps-msgs` first.

## 11. Concrete recommended fixes or confirmations

### Keep now

1. Keep Jetson on the Pi-compatible IMU behavior for now:
   - `/IMU_data`
   - `frame_id:=base_link`
   - stable by-id serial path
2. Keep the promoted `hipnuc_lib_package + hipnuc_imu` build as the compatibility baseline.
3. Keep `robot_state_publisher` for TF/RViz completeness, but do not mistake it for the IMU frame source of truth.

### Confirm explicitly

1. Confirm exactly one IMU publisher is alive.
2. Confirm `/IMU_data` is live before Cartographer starts.
3. Confirm the serial path by id, not by `ttyUSB0`.

### Do later, not during parity bring-up

1. Do not switch to the latest official upstream `imu_link` + euler/magnetic variant until the Jetson reproduces Pi goal-test parity consistently.
2. Do not swap in the WHEELTEC H30 SDK unless the hardware is proven to be that product family. The inspected Smart3D code is HiPNUC-protocol-based.
3. Consider removing or rewiring `imu_qos_bridge.py` only after verifying whether any out-of-tree consumer actually needs `/imu_reliable`.

## 12. Bottom line

The safest interpretation is:

- Smart3D is using the HiPNUC ROS2 serial example family, not the WHEELTEC H30 SDK.
- The Pi baseline relies on a simplified, Pi-stable promoted copy of the HiPNUC driver.
- Jetson should replicate that promoted copy first, but with stable `/dev/serial/by-id/...` device selection instead of official `/dev/ttyUSB0`.
- The current official HiPNUC repo has moved ahead of the promoted copy, and its own README and current code are not perfectly aligned.

Brief summary: I found that the current Smart3D IMU path is a Pi-stable forked/simplified snapshot of the official HiPNUC ROS2 example. The only file I changed is [HARDWARE_HIPNUC_IMU.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/research/HARDWARE_HIPNUC_IMU.md).
