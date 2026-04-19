# Jetson Orin NX 16GB Platform and ROS 2 Humble for Smart3D

Last updated: 2026-04-18

This report owns the Jetson Orin NX 16GB platform and ROS 2 Humble track for the Smart3D stack. It is based on:

- the canonical local Smart3D tree at `/Users/felixfang/Documents/Playground/connection-guideline/smart3d`
- the current live Jetson runtime observations from 2026-04-18
- official NVIDIA and ROS documentation for JetPack 6 / Jetson Linux R36.x / Ubuntu 22.04 / ROS 2 Humble

This document is intentionally narrow: platform, package-source, runtime-shape, and verification guidance for the Jetson track. Per-device driver deep dives for LiDAR, IMU, DDSM115, and camera should live in separate hardware notes.

## 1. Platform identity

### 1.1 Canonical local files reviewed

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_JETSON_ORIN_NX16_MIGRATION_FROM_PI5.md`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_CAMPUS_RUNTIME.md`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/deploy_runtime_to_jetson.sh`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/m10p_uart.yaml`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/mk_imu_dev.sh`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/robotcar.urdf`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_m10p_dev.sh`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_ddsm_dev.sh`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/campus_map_bridge.py`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/pi_ros_watchdog.py`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_client.py`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cmd_vel_driver.py`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/i9_planner.py`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/cartographer_launch.py`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d_odom.lua`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/imu_qos_bridge.py`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/ddsm115_odom.py`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/ros2_ws_src/hipnuc_imu/package.xml`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/ros2_ws_src/hipnuc_gnss/package.xml`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/ros2_ws_src/hipnuc_lib_package/package.xml`

### 1.2 Current live Jetson identity observed on 2026-04-18

Observed directly over SSH on `jetson@10.66.121.109`:

- host: `jetson-desktop`
- user: `jetson`
- OS: `Ubuntu 22.04.5 LTS`
- kernel: `5.15.136-tegra`
- L4T / JetPack identity from `/etc/nv_tegra_release`: `R36 (release), REVISION: 3.0`
- platform class: Jetson Orin NX 16GB on a Seeed reComputer image

Observed runtime directories:

- `/home/jetson/SMART3D/ws_lidar`
- `/home/jetson/SMART3D/ws_imu`
- `/home/jetson/SMART3D/jetson_runtime`

Observed onboard device map:

- DDSM115 RS485 adapter: `/dev/ttyUSB0`, stable by-id `usb-FTDI_FT232R_USB_UART_BG01LYTM-if00-port0`
- primary M10P LiDAR: `/dev/ttyACM0`, stable by-id `usb-1a86_USB_Single_Serial_5A6D010263-if00`
- HiPNUC / WHEELTEC IMU: `/dev/ttyACM1`, stable by-id `usb-1a86_USB_Single_Serial_5B0B030655-if00`
- extra ACM serial device: `/dev/ttyACM2`, stable by-id `usb-1a86_USB_Single_Serial_5A6C069316-if00`
- Logitech camera family: `/dev/video0` and siblings

Observed user groups on the live Jetson:

- `dialout`
- `video`
- `plugdev`
- `render`
- `i2c`
- `gpio`

That group membership is operationally important because the current Smart3D stack relies on direct access to serial and video devices.

## 2. Official package sources and support matrix

### 2.1 NVIDIA platform support that matches this board

Official NVIDIA sources confirm the following:

- JetPack 6.0 production release ships Jetson Linux 36.3, Linux 5.15, and an Ubuntu 22.04 root filesystem, and supports Jetson Orin modules and developer kits.
- Jetson Linux 36.3 GA is production-ready and uses Ubuntu 22.04 on `aarch64`.
- NVIDIA distributes JetPack components as Debian packages via `repo.download.nvidia.com/jetson`, and `nvidia-jetpack` is the canonical metapackage for installing JetPack components on-device.

Primary sources:

- [JetPack 6.0 Release Notes](https://docs.nvidia.com/jetson/jetpack/6.0/release-notes/index.html)
- [JetPack 6.0 Install and Setup](https://docs.nvidia.com/jetson/jetpack/6.0/install-setup/index.html)
- [Jetson Linux 36.3 GA Release Notes PDF](https://docs.nvidia.com/jetson/archives/r36.3/ReleaseNotes/Jetson_Linux_Release_Notes_r36.3.pdf)

Why this matters for Smart3D:

- this Jetson is not an unsupported distro experiment
- ROS 2 Humble on Jammy arm64 is aligned with the actual BSP the board is running
- the correct platform source of truth for CUDA / TensorRT / multimedia / container runtime is the NVIDIA JetPack apt repo, not the ROS repo and not a random Ubuntu PPA

### 2.2 Official ROS 2 support that matches this board

Official ROS sources confirm the following:

- ROS 2 Humble supports Ubuntu Jammy 22.04 on both `amd64` and `arm64`
- Debian packages are provided for Humble on Jammy
- `rmw_cyclonedds_cpp` is Tier 1 in REP-2000, which matches the current Smart3D decision to standardize on CycloneDDS
- the current recommended way to add the ROS apt source on Jammy is through `ros2-apt-source`

Primary sources:

- [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)
- [ROS 2 Humble Ubuntu binary install](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Install-Binary.html)
- [REP-2000 target platforms for Humble](https://www.ros.org/reps/rep-2000.html)

Why this matters for Smart3D:

- native ROS 2 Humble on this Jetson is officially supported by ROS, not a workaround
- CycloneDDS is a supported production choice, not a one-off hack
- the ROS apt repo should be treated as separate from the NVIDIA JetPack repo

### 2.3 NVIDIA Isaac ROS caveat for this specific board image

This Jetson is on JetPack 6.0 / L4T 36.3. Official NVIDIA Isaac ROS documentation matters only if we decide to add Isaac packages later.

Important version caveat:

- Isaac ROS release notes show JetPack 6.0 / Ubuntu 22.04 support in the 3.0.0 release line
- newer Isaac ROS 3.2 "Getting Started" docs target JetPack 6.1 and 6.2 on Jetson Orin and recommend the Isaac ROS Dev Docker workflow

Primary sources:

- [Isaac ROS release notes](https://nvidia-isaac-ros.github.io/releases/index.html)
- [Isaac ROS 3.2 Getting Started](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/index.html)
- [Isaac ROS Dev Docker environment](https://nvidia-isaac-ros.github.io/v/release-3.1/concepts/docker_devenv/index.html)

Operational conclusion:

- for the current Smart3D LiDAR + IMU + RS485 + Cartographer stack, Isaac ROS is optional
- if Isaac ROS is added later, the release must match the board's JetPack line; do not blindly follow newer 6.1/6.2 quickstarts on a 6.0 machine

### 2.4 Known uncertainty and incomplete source chains

The first-pass report is strong on platform support and package-source authority, but a few hardware-source chains are still not "fully officialized" inside this document:

1. LSLiDAR M10P ROS 2 packaging:
   - current conclusion is based on the live Jetson workspace using the vendor GitHub repo and on the absence of an apt package on the live machine
   - this report does not yet include a vendor PDF/manual for the exact M10P serial ROS 2 branch

2. HiPNUC IMU ROS 2 packaging:
   - current conclusion is based on the live Jetson workspace and the source snapshot under the Smart3D deploy tree
   - this report does not yet include a separate official vendor support matrix for Jammy/Humble

3. Logitech camera runtime use inside Smart3D:
   - device presence and Linux access are verified
   - this report does not claim that camera topics are already part of the default Jetson Smart3D bring-up path

Current best conclusion despite those gaps:

- the Jetson platform + JetPack + ROS 2 Humble foundation is well-supported and suitable now
- the current Smart3D serial LiDAR / IMU / RS485 runtime should proceed on native ROS 2 Humble now
- per-device official driver packaging notes can be filled in as separate hardware reports without changing the platform decision

## 3. What should be apt-installed vs built from source

### 3.1 Must come from NVIDIA apt

Install and maintain the Jetson platform packages from the NVIDIA apt server, not from source:

- `nvidia-jetpack`
- `nvidia-jetpack-runtime`
- `nvidia-jetpack-dev`
- `nvidia-container`
- `nvidia-container-toolkit`

Current live Jetson confirmation:

- `nvidia-jetpack` is installed from `https://repo.download.nvidia.com/jetson/common r36.3/main`
- the NVIDIA container packages are already present on the board

This is the correct place to get:

- CUDA
- TensorRT
- cuDNN
- VPI
- NVIDIA multimedia components
- the NVIDIA container runtime integration

### 3.2 Should come from ROS apt on Jammy

For the Smart3D ROS base, install from the ROS apt repo on Jammy arm64:

- `ros-humble-desktop`
- `ros-humble-cartographer`
- `ros-humble-cartographer-ros`
- `ros-humble-rmw-cyclonedds-cpp`
- `ros-humble-robot-state-publisher`
- `ros-humble-rviz2`
- `ros-humble-imu-tools`
- `ros-humble-gps-msgs`
- `ros-humble-diagnostic-updater`
- `python3-rosdep`
- `python3-colcon-common-extensions`
- `python3-serial`

Recommended Ubuntu/Jammy support packages for this robot:

- `software-properties-common`
- `curl`
- `git`
- `build-essential`
- `libpcap-dev`
- `usbutils`
- `v4l-utils`

Current live Jetson confirmation:

- `ros-humble-desktop` is installed from `http://packages.ros.org/ros2/ubuntu jammy/main`
- `ros-humble-cartographer` is installed from the same ROS source
- `ros-humble-rmw-cyclonedds-cpp` is installed from the same ROS source

### 3.3 Must be built from source for the current Smart3D stack

These parts should stay source-built for now:

1. LiDAR ROS driver:
   - official vendor repo currently in use on the Jetson:
     - remote workspace: `/home/jetson/SMART3D/ws_lidar/src/Lslidar_ROS2_driver`
     - git remote: `https://github.com/Lslidar/Lslidar_ROS2_driver.git`
     - branch: `M10P/N10P`
   - reason: this exact M10P serial driver path is not provided by the standard ROS Jammy apt repo

2. IMU ROS packages:
   - source snapshot currently carried from:
     - `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/ros2_ws_src/hipnuc_imu/package.xml`
     - `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/ros2_ws_src/hipnuc_gnss/package.xml`
     - `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/ros2_ws_src/hipnuc_lib_package/package.xml`
   - reason: no Jammy/Humble apt package was found on the live Jetson for `hipnuc*`

3. Smart3D application/runtime code:
   - `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_client.py`
   - `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cmd_vel_driver.py`
   - `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/imu_qos_bridge.py`
   - `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/ddsm115_odom.py`
   - `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/cartographer_launch.py`
   - `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d_odom.lua`
   - `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/campus_map_bridge.py`
   - `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/pi_ros_watchdog.py`
   - `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_m10p_dev.sh`
   - `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_ddsm_dev.sh`
   - `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/mk_imu_dev.sh`
   - `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/m10p_uart.yaml`
   - `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/robotcar.urdf`
   - reason: these are project-owned runtime files, not external packages

4. Planner side:
   - `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/i9_planner.py`
   - keep this as project source until the planner itself is intentionally relocated to Jetson

### 3.4 Practical package split for this robot

The clean split is:

- NVIDIA apt: board support, GPU stack, multimedia stack, container runtime
- ROS apt: ROS core, Cartographer, CycloneDDS RMW, RViz, robot_state_publisher
- source workspaces: LSLiDAR M10P driver, HiPNUC IMU stack, Smart3D application code

That split matches both official support boundaries and the current working robot behavior.

## 4. Device-access expectations for USB serial and video on Jetson

### 4.1 What the live Jetson actually shows

Observed on 2026-04-18:

- `/dev/ttyUSB0` is owned by `root:dialout`
- `/dev/ttyACM0`, `/dev/ttyACM1`, `/dev/ttyACM2` are owned by `root:dialout`
- `/dev/video0` is owned by `root:video`
- `/dev/serial/by-id` has stable names for DDSM, M10P, and IMU
- the `jetson` user is already a member of both `dialout` and `video`

This matches the current Smart3D runtime assumptions.

### 4.2 What to expect on Jetson for this stack

For this robot, direct host access is expected for:

- serial RS485 motor bus
- serial LiDAR
- serial IMU
- V4L2 camera nodes

Ubuntu device-access guidance historically ties serial access to `dialout` and video access to `video`. The live board already matches that expectation. For this Smart3D stack, the practical rule is:

- serial nodes should run as a user in `dialout`
- camera nodes should run as a user in `video`
- stable `/dev/serial/by-id/*` names should be preferred over hand-made symlinks whenever possible

Primary Ubuntu references:

- [Ubuntu Security/Privileges](https://wiki.ubuntu.com/Security/Privileges)
- [Ubuntu Serial Console HOWTO](https://help.ubuntu.com/community/SerialConsoleHowto)

### 4.3 Smart3D-specific device guidance

For this robot, the correct device policy is:

1. Prefer stable by-id names for serial sensors:
   - the current LiDAR params file `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/m10p_uart.yaml` already points M10P at `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6D010263-if00`
   - the current launcher `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh` already points the IMU at `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B030655-if00`

2. Keep the helper scripts for compatibility and recovery:
   - `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_m10p_dev.sh`
   - `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_ddsm_dev.sh`
   - `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/mk_imu_dev.sh`

3. Treat `/dev/ttyUSB0` as a single-owner bus:
   - `cmd_vel_driver.py` and `ddsm115_odom.py` both touch the DDSM bus
   - operational inference from the current Smart3D code: only one RS485 owner should actively drive the port at a time when doing low-level diagnostics

4. If a container is used later, device passthrough is not optional:
   - `/dev/ttyUSB0`
   - `/dev/ttyACM*`
   - `/dev/video*`
   - `/dev/serial/by-id`

That last point is an operational inference from the current runtime shape and DDS/device usage, not a direct quote from NVIDIA docs.

## 5. Pi-vs-Jetson runtime differences that matter

### 5.1 Pi 5 baseline

The Pi baseline described by:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_CAMPUS_RUNTIME.md`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh`

uses:

- a `ros2-humble` container on the Pi
- `pi5_client.py` as the thin onboard client
- `cmd_vel_driver.py` as the RS485 `/cmd_vel` owner
- local Cartographer on the robot
- `campus_map_bridge.py` to translate local RELIABLE `/map` into campus `BEST_EFFORT /map_campus`
- i9 as offboard planner and RViz host

### 5.2 Jetson runtime today

The Jetson runtime described by:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/deploy_runtime_to_jetson.sh`

is already a native-host layout:

- native `/opt/ros/humble`
- native `ws_lidar`
- native `ws_imu`
- project runtime in `/home/jetson/SMART3D/jetson_runtime`

Current launch order from `start_full_native.sh` is:

1. static TF
2. LiDAR driver
3. IMU talker
4. IMU QoS bridge
5. wheel odom
6. Cartographer
7. campus map bridge
8. robot_state_publisher
9. `pi5_client.py`
10. `pi_ros_watchdog.py`
11. optional `cmd_vel_driver.py`

### 5.3 The meaningful behavioral differences

Pi behavior that should be preserved:

- thin-client control split: planner offboard, pose/path follower onboard
- local safety behavior remains onboard
- campus map bridge remains in the loop while planner stays offboard
- RS485 ownership remains local

Jetson-specific improvements already adopted:

- no dependence on container bind mounts to reach serial devices
- direct use of `/dev/serial/by-id` for IMU and LiDAR
- direct native ROS workspaces for vendor drivers
- easier debugging of `/tmp/*.log` and `/opt/ros/humble` environment

Operational inference:

- the Jetson should preserve Pi behavior first, then simplify topology later
- do not collapse planner and onboard runtime onto Jetson in the same step that introduces a new hardware platform

## 6. Native ROS 2 Humble on Jetson vs recreating the Pi `ros2-humble` container

### 6.1 Native ROS 2 Humble on this Jetson

Advantages:

- officially supported combination: JetPack 6.0 + Ubuntu 22.04 + ROS 2 Humble on arm64
- simplest path to serial, camera, and `/dev/serial/by-id` device access
- matches the runtime that is already proven on this Jetson
- easiest to debug DDS, udev, and sensor logs
- no extra container network and device-pass-through layer to get wrong

Costs:

- environment is less hermetic than a dev container
- source-built vendor packages live directly on the machine
- long-term reproducibility depends on documenting apt state and workspaces well

### 6.2 Recreating the Pi `ros2-humble` container on Jetson

Advantages:

- closer operational symmetry with the Pi baseline
- easier to snapshot and redeploy once stable
- useful if we later standardize on a build/deploy artifact
- aligns with NVIDIA Isaac ROS guidance if accelerated Isaac packages become important

Costs for this hardware stack:

- Docker is currently installed but inactive on the live board
- every serial and video device would need explicit passthrough
- DDS networking would need host-mode parity with the current campus CycloneDDS setup
- JetPack/NVIDIA runtime integration becomes another moving piece
- for this robot's present serial LiDAR + serial IMU + RS485 drivetrain + V4L2 camera stack, the container does not solve the current hard problems

### 6.3 Decision

Recommended decision for this board:

- use native ROS 2 Humble as the production commissioning path
- optionally add a containerized dev/build workflow later
- do not make the first successful full-stack Jetson commissioning depend on recreating the Pi container

This is both the most practical Smart3D choice and the most compatible with the board's current verified state.

## 7. Recommended full-stack deployment shape

### 7.1 Recommended near-term shape

For the next stable milestone, use this shape:

1. Jetson host OS and JetPack stay native
2. ROS 2 Humble stays native on host
3. vendor driver workspaces stay native:
   - `ws_lidar`
   - `ws_imu`
4. Smart3D robot runtime stays in:
   - `/home/jetson/SMART3D/jetson_runtime`
5. onboard Jetson owns:
   - LiDAR driver
   - IMU driver
   - wheel odom
   - Cartographer
   - `campus_map_bridge.py`
   - `pi5_client.py`
   - `cmd_vel_driver.py`
   - local watchdogs
6. i9 continues to own:
   - `i9_planner.py`
   - RViz
   - offboard goal publication

That is the correct "full stack" for the robot runtime while still honoring the proven Smart3D architecture.

### 7.2 Why this is the right complete shape now

It preserves the real contract already validated by Pi and Jetson:

- sensing is local
- SLAM is local
- safety is local
- motor actuation is local
- planning is remote
- goal-test remains the real end-to-end acceptance check

### 7.3 Second-stage shape only after the above is stable

Only after the above passes repeated goal-tests should we consider:

- moving `i9_planner.py` onto Jetson
- removing the campus map bridge
- switching to a one-box planner + runtime layout
- introducing an Isaac ROS dev container or deployment container

That is a second move, not the first move.

## 8. Exact verification commands

The commands below are the exact ones worth running for this Jetson track.

### 8.1 Platform identity and package-source verification

On Jetson:

```bash
whoami
uname -a
cat /etc/os-release | sed -n '1,8p'
cat /etc/nv_tegra_release
apt-cache policy nvidia-jetpack ros-humble-desktop ros-humble-cartographer ros-humble-rmw-cyclonedds-cpp
```

Expected shape:

- Ubuntu 22.04.x
- `R36 (release), REVISION: 3.0`
- `nvidia-jetpack` from `repo.download.nvidia.com/jetson`
- ROS packages from `packages.ros.org/ros2/ubuntu jammy`

### 8.2 Device-access verification

On Jetson:

```bash
id
groups
ls -l /dev/ttyUSB0 /dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2 /dev/video0
ls -l /dev/serial/by-id
```

Expected shape:

- user is in `dialout` and `video`
- serial devices are `root:dialout`
- video devices are `root:video`
- stable `by-id` entries exist for FTDI, M10P, and IMU

Optional if `v4l-utils` is installed:

```bash
v4l2-ctl --list-devices
```

### 8.3 Source-workspace verification

On Jetson:

```bash
source /opt/ros/humble/setup.bash
cd /home/jetson/SMART3D/ws_lidar
colcon list
cd /home/jetson/SMART3D/ws_imu
colcon list
```

Expected shape:

- LiDAR workspace contains `lslidar_driver` and `lslidar_msgs`
- IMU workspace contains `hipnuc_imu`, `hipnuc_gnss`, and `hipnuc_lib_package`

### 8.4 Passive onboard runtime verification

On Jetson:

```bash
cd /home/jetson/SMART3D/jetson_runtime
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclone-campus-jetson.xml
START_ACTIVE=0 START_WATCHDOG=1 START_ODOM=1 ./start_full_native.sh
```

Expected shape:

- LiDAR starts
- IMU starts
- Cartographer starts
- map bridge starts
- `pi5_client.py` starts
- watchdog starts
- final topic probe shows non-zero counts on `/scan`, `/IMU_data`, `/map`, and `/car1/pose`

### 8.5 Log verification after passive bring-up

On Jetson:

```bash
tail -n 80 /tmp/lidar.log
tail -n 80 /tmp/imu_talker.log
tail -n 80 /tmp/imu_bridge.log
tail -n 80 /tmp/cartographer.log
tail -n 80 /tmp/campus_map_bridge.log
tail -n 80 /tmp/pi5_client_campus.log
tail -n 80 /tmp/pi_ros_watchdog.log
```

Expected shape:

- LiDAR log shows the serial port opened successfully
- IMU bridge log shows increasing bridged message counts
- Cartographer log shows scan / IMU / odom rates
- campus map bridge log shows bridged maps
- `pi5_client.py` is alive and publishing pose

### 8.6 Active drivetrain smoke-test prep

Only with a human watching the robot:

```bash
cd /home/jetson/SMART3D/jetson_runtime
pkill -f '[d]dsm115_odom.py|[c]md_vel_driver.py' || true
python3 ddsm_emergency_brake.py /dev/ttyUSB0
python3 robotcar_wasdqe_test.py
```

Expected shape:

- the emergency-brake script can reach the bus
- manual motion test happens under observation

Warning:

- do not claim drivetrain verification from serial-open success alone
- actual RS485 feedback or physical wheel response is required

### 8.7 End-to-end planner verification with i9

Jetson passive runtime first, then on i9:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclone-campus-planner-jetson.xml
python3 /home/felix/SMART3D/i9_planner.py --ros-args \
  -p car_names:=car1 \
  -p robot_speed:=0.24 \
  -p campus_mode:=true \
  -p map_topic:=/map_campus
```

Then publish a short goal from the validation participant and confirm:

- planner logs `Build OK`
- Jetson `pi5_client.py` logs `Path received`
- the full chain reaches `GOAL REACHED!`

For Smart3D, that goal-test remains the acceptance check. Topic visibility alone is not enough.

## 9. Final recommendation

For this exact board and this exact Smart3D stack:

- keep JetPack and ROS native on the Jetson
- keep vendor sensor drivers source-built in native workspaces
- keep Smart3D runtime files deployed into `/home/jetson/SMART3D/jetson_runtime`
- keep i9 planner external until repeated goal-tests are stable
- postpone any Pi-style container recreation until after the native full-stack path is boringly reliable

If later work adds Isaac ROS perception or GPU-heavy camera pipelines, revisit a Docker-based workflow, but match the Isaac ROS release line to the JetPack line first.

## 10. Brief summary

What I found:

- JetPack 6.0 / Jetson Linux 36.3 / Ubuntu 22.04 on Jetson Orin is officially supported by NVIDIA.
- ROS 2 Humble on Ubuntu 22.04 arm64 is officially supported by ROS, including `rmw_cyclonedds_cpp`.
- The live Jetson already matches that supported combination and already has the correct device-group shape for serial and video access.
- For the current Smart3D hardware stack, native ROS 2 Humble is the right production path; rebuilding the Pi container first would add complexity without solving the actual commissioning risks.
- The correct package split is NVIDIA apt for platform components, ROS apt for ROS core, and source builds for LSLiDAR, HiPNUC, and Smart3D-owned runtime code.
