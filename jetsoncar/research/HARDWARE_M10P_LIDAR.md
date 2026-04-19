# Smart3D M10P LiDAR Research Report

Last updated: 2026-04-18

## Scope

This report covers the Leishen / LSLiDAR M10P track for Smart3D, with three goals:

1. Identify the official vendor docs and the official ROS2 driver path for M10P.
2. Compare the official usage model with the current Pi baseline and the current Jetson replication.
3. Leave exact, reproducible commands and a list of remaining gaps so the next worker does not have to rediscover the same ground.

## Local Files Inspected

Canonical Smart3D tree:

- [SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md)
- [SMART3D_JETSON_ORIN_NX16_MIGRATION_FROM_PI5.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_JETSON_ORIN_NX16_MIGRATION_FROM_PI5.md)
- [mk_m10p_dev.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_m10p_dev.sh)
- [pi_ros_watchdog.py](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/pi_ros_watchdog.py)
- [campus_map_bridge.py](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/campus_map_bridge.py)
- [smart3d_campus.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh)
- [m10p_uart.yaml](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/m10p_uart.yaml)
- [start_full_native.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh)
- [deploy_runtime_to_jetson.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/deploy_runtime_to_jetson.sh)

Deploy tree:

- [cartographer_launch.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/cartographer_launch.py)
- [robotcar_2d.lua](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d.lua)
- [robotcar_2d_odom.lua](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d_odom.lua)
- [carto_bringup.sh](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/carto_bringup.sh)
- [README.md](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/README.md)
- [start_full.sh](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/start_full.sh)
- [start_all.sh](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/start_all.sh)
- [pi5_client.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_client.py)
- [pi5_client.py (reference)](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/pi5_client.py)

## Hardware Identity

- Smart3D uses a Leishen / LSLiDAR M10P-class 2D navigation LiDAR as the single `/scan` source that feeds Cartographer and local obstacle-stop logic in [pi5_client.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_client.py).
- The official ROS2 driver code does **not** use the marketing string `M10P`. It expects `lidar_name: M10_P`.
- The official vendor materials also mention `M10P PHY`, which is a different naming track and appears to refer to a smaller serial-only device family. Do not casually substitute `M10P PHY` for `M10_P` in the ROS2 driver.
- The current Smart3D Jetson runtime is aligned with the `M10_P` driver path, not with a separate `M10P PHY` package, as shown in [m10p_uart.yaml](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/m10p_uart.yaml).

Inference: the current Smart3D hardware is far more likely to be the standard M10P / `M10_P` branch path than `M10P PHY`, because the local working runtime, the official branch we used, and the driver parameterization all align on `M10_P`. The local tree does not contain a photo of the model sticker, so the exact printed SKU on the device was not independently verified here.

## Official Docs and Packages

Official vendor sources:

- Official vendor GitHub org: [https://github.com/Lslidar](https://github.com/Lslidar)
- Official ROS2 driver repo: [https://github.com/Lslidar/Lslidar_ROS2_driver](https://github.com/Lslidar/Lslidar_ROS2_driver)
- Official product page family entry that exposes M10 / M10P specs: [https://www.lslidar.com/ru/product/m10-navigation-obstacle-avoidance-lidar/](https://www.lslidar.com/ru/product/m10-navigation-obstacle-avoidance-lidar/)
- Official older product brochure PDF: [https://www.lslidar.com/wp-content/uploads/2022/10/LSLiDAR-Product-Brochure_CompressPdf.pdf](https://www.lslidar.com/wp-content/uploads/2022/10/LSLiDAR-Product-Brochure_CompressPdf.pdf)
- Official newer brochure PDF: [https://www.lslidar.com/wp-content/uploads/2024/12/LSLiDAR-Product-Brochure%EF%BC%88EN-V2.2-24.10.23%E7%89%88%EF%BC%89.pdf](https://www.lslidar.com/wp-content/uploads/2024/12/LSLiDAR-Product-Brochure%EF%BC%88EN-V2.2-24.10.23%E7%89%88%EF%BC%89.pdf)

What the official ROS2 repo says:

- The top-level README still says the driver was tested on Ubuntu 20.04 with ROS2 Foxy.
- The same README also tells users to clone a model-specific branch rather than use a released Debian package.
- The official branch list contains `M10P/N10P`.
- The checked-out branch used for this report was:

```bash
git clone --depth 1 --branch 'M10P/N10P' https://github.com/Lslidar/Lslidar_ROS2_driver.git
```

- The branch snapshot inspected here was commit `8bb760c8f66c1b6964caaf3cf6f7a2a0ea24cefa`.
- The package declared inside the branch is `lslidar_driver` version `1.2.0`.
- The branch `version.txt` says version `LSLIDAR_M10_N10_V2.5.2_230110_ROS2` added "compatible with ROS2 humble."

Conclusion for Jetson Orin NX + ROS 2 Humble:

- The correct official starting point is **not** the generic `main` README text.
- The correct official starting point is the **`M10P/N10P` branch** of the vendor ROS2 repo.
- There is no evidence in the official repo of a separate Humble-only branch for M10P.
- I did not find an official released ROS2 Debian package from Leishen for this driver; the official workflow is source checkout by model branch.

Important official-doc inconsistency:

- The README is partly stale: it mentions Foxy and even says "Catkin package."
- The actual branch package metadata uses `ament_cmake`, and the official version history explicitly mentions Humble support.
- For Smart3D, treat the branch code and `version.txt` as more authoritative than the README prose.

## Official Driver Options and Parameters That Matter

The current working Smart3D Jetson config in [m10p_uart.yaml](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/m10p_uart.yaml) is a custom Smart3D adaptation of the vendor parameter model. The official vendor parameter examples live in the `lslidar_driver/params` folder of the `M10P/N10P` branch, but none of those examples is a clean dedicated `M10_P` YAML.

The parameters that actually matter:

| Parameter | Official meaning | Smart3D choice | Notes |
| --- | --- | --- | --- |
| `lidar_name` | Selects protocol path and internal baud/packet handling | `M10_P` | Must match the driver enum, not the marketing label `M10P`. |
| `interface_selection` | `net` or `serial` | `serial` | Smart3D is using USB serial transport, not Ethernet. |
| `serial_port_` | Serial device path | Pi: `/dev/m10p`; Jetson: by-id path | Official examples default to `/dev/ttyUSB0`, which is too fragile for this robot. |
| `frame_id` | Laser frame name | `laser_link` | Must match Cartographer TF expectations in [cartographer_launch.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/cartographer_launch.py). |
| `scan_topic` | LaserScan topic | `/scan` | Smart3D standard topic. |
| `compensation` | Angle compensation for M10 family | `true` on Jetson | Current Smart3D uses compensated scans. |
| `pubScan` | Publish LaserScan | `true` | Required for Cartographer and local obstacle stop. |
| `pubPointCloud2` | Publish PointCloud2 | `false` on Jetson | Smart3D does not need point clouds for the current full chain. |
| `high_reflection` | M10_P-specific tuning flag | `false` currently | Official comments say ask vendor support if unsure. Still unvalidated in Smart3D. |

Two subtle but important details from official code:

1. The vendor driver hardcodes `baud_rate_ = 500000` for `M10_P`.
2. The vendor driver sets M10_P diagnostics around a 12 Hz scan rate, not 10 Hz.

This matters because the official vendor materials are inconsistent:

- Older brochure text points to `512000 bps`.
- Newer brochure text for `M10P PHY` mentions `230400 bps`.
- Other snippets suggest `460800 bps`.
- The **actual official M10_P driver code path** opens serial at `500000`.

For Smart3D, the code path wins. Do not try to "fix" the runtime by inventing an external baud parameter unless the official driver code is also changed.

## Serial Device Binding Strategy

### Official baseline

The official vendor params default to `/dev/ttyUSB0`. That is acceptable for a one-sensor lab demo but it is not robust enough for Smart3D, where the LiDAR, IMU, RS485 adapter, and other USB devices can reorder on boot.

### Pi baseline

Smart3D uses [mk_m10p_dev.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_m10p_dev.sh) to create `/dev/m10p` by scanning `/dev/ttyACM*`, reading USB serial identities, and preferring:

- primary LiDAR serial: `5A6D010263`
- fallback LiDAR serial: `5A6C069316`
- IMU serial to avoid: `5B0B030655`

That helper exists because the Pi container path still expects a stable symbolic name rather than a volatile kernel enumeration.

### Jetson replication

The current Jetson runtime improves on the Pi approach by putting the by-id device path directly into [m10p_uart.yaml](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/m10p_uart.yaml):

```yaml
serial_port_: /dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6D010263-if00
```

This is the better long-term approach on Jetson because:

- it survives ttyACM index churn;
- it does not depend on a writable `/dev/m10p` symlink;
- it still remains compatible with the official driver because the vendor node only needs a valid serial path string.

Current Smart3D behavior:

- [start_full_native.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh) still runs [mk_m10p_dev.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_m10p_dev.sh) for compatibility and logging.
- The actual steady-state source of truth on Jetson is the by-id path inside [m10p_uart.yaml](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/m10p_uart.yaml).

Recommendation:

- Keep `/dev/m10p` compatibility on Pi because the container flow already assumes it.
- Keep direct `/dev/serial/by-id/...` binding on Jetson because it is more stable and simpler.

## Pi Baseline Behavior

The Pi baseline is not just "run the vendor LiDAR node." It is a containerized Smart3D pipeline assembled around the vendor node.

The important Pi-side files are:

- [smart3d_campus.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh)
- [start_full.sh](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/start_full.sh)
- [start_all.sh](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/start_all.sh)
- [cartographer_launch.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/cartographer_launch.py)
- [robotcar_2d.lua](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d.lua)
- [robotcar_2d_odom.lua](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d_odom.lua)
- [campus_map_bridge.py](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/campus_map_bridge.py)
- [pi5_client.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_client.py)

Pi startup order, reconstructed from the current tree:

1. Rebind LiDAR and IMU devices.
2. Start `static_transform_publisher` for `base_link -> laser_link`.
3. Start `lslidar_driver_node` with `/root/m10p_ws/m10p_uart.yaml`.
4. Start the HiPNUC IMU talker on `/IMU_data`.
5. Start `imu_qos_bridge.py`.
6. Optionally start `ddsm115_odom.py`.
7. Launch Cartographer with [cartographer_launch.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/cartographer_launch.py).
8. Start [campus_map_bridge.py](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/campus_map_bridge.py) so `/map` becomes `/map_campus`.
9. Start `robot_state_publisher`.
10. Start [pi5_client.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_client.py).
11. Start [pi_ros_watchdog.py](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/pi_ros_watchdog.py).

How `/scan` is consumed downstream:

- [cartographer_launch.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/cartographer_launch.py) expects `/scan` on `laser_link` and remaps IMU to `/IMU_data`.
- [robotcar_2d.lua](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d.lua) and [robotcar_2d_odom.lua](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d_odom.lua) clamp useful lidar range to `0.15 .. 12.0` and expect a single LaserScan stream.
- [pi5_client.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_client.py) subscribes to `/scan` using BEST_EFFORT and uses it only for local emergency-stop logic.

Important Pi gap:

- The active Pi LiDAR params file is still `/root/m10p_ws/m10p_uart.yaml` inside the container.
- That file is **referenced** by [start_full.sh](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/start_full.sh), [start_all.sh](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/start_all.sh), [carto_bringup.sh](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/carto_bringup.sh), and [pi_ros_watchdog.py](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/pi_ros_watchdog.py).
- But that Pi YAML is not versioned under the canonical local Smart3D tree. This is a real reproducibility hole.

## Jetson Replication Steps

The current Jetson path is a native ROS2 Humble bring-up, not a container clone of the Pi.

### 1. Build the official vendor M10P driver from source

Recommended Jetson workspace build:

```bash
mkdir -p ~/SMART3D/ws_lidar/src
cd ~/SMART3D/ws_lidar/src
git clone --branch 'M10P/N10P' https://github.com/Lslidar/Lslidar_ROS2_driver.git
cd ~/SMART3D/ws_lidar
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --event-handlers console_direct+
```

Notes:

- The vendor branch claims Humble compatibility in `version.txt`.
- In practice, the package needs normal ROS2 dependencies such as `diagnostic_updater`.
- `libpcap-dev` is relevant if PCAP mode is needed.

### 2. Deploy the Smart3D runtime overlay

Use [deploy_runtime_to_jetson.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/deploy_runtime_to_jetson.sh), which copies:

- the Pi-origin helper scripts;
- the Smart3D Cartographer config files;
- the current Jetson LiDAR YAML;
- the path-follower and watchdog nodes.

### 3. Keep the Jetson M10P YAML aligned with the official driver schema

The current working Smart3D Jetson config in [m10p_uart.yaml](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/m10p_uart.yaml) should stay aligned to the official vendor parameter names:

```yaml
/lslidar_driver_node:
  ros__parameters:
    frame_id: laser_link
    lidar_name: M10_P
    scan_topic: /scan
    interface_selection: serial
    serial_port_: /dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6D010263-if00
    high_reflection: false
    compensation: true
    pubScan: true
    pubPointCloud2: false
```

### 4. Start the full Jetson runtime

The current full-chain entrypoint is [start_full_native.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh):

```bash
cd /home/jetson/SMART3D/jetson_runtime
export CYCLONEDDS_URI=file:///tmp/cyclone-campus-jetson.xml
START_ACTIVE=0 START_WATCHDOG=1 START_ODOM=1 ./start_full_native.sh
```

That script starts the LiDAR in the same slot in the order that the Pi baseline uses:

1. static TF;
2. LiDAR;
3. IMU;
4. IMU bridge;
5. odom;
6. Cartographer;
7. map bridge;
8. robot_state_publisher;
9. path follower;
10. watchdog.

## Exact Launch and Validation Commands

### Vendor-style LiDAR-only smoke test

This is the closest thing to the official bare driver usage:

```bash
source /opt/ros/humble/setup.bash
source ~/SMART3D/ws_lidar/install/setup.bash
ros2 run lslidar_driver lslidar_driver_node \
  --ros-args --params-file ~/SMART3D/jetson_runtime/m10p_uart.yaml
```

Expected LiDAR log shape:

```text
Lidar is M10_P
open_port /dev/... OK !
Initialised lslidar without error
```

The official repo README also documents the order-control topic:

```bash
ros2 topic pub -1 /lslidar_order std_msgs/msg/Int8 data:\ 1
ros2 topic pub -1 /lslidar_order std_msgs/msg/Int8 data:\ 0
```

Smart3D currently does not use that topic in its main bring-up.

### Pi container validation

Current Pi validation shape, reconstructed from [smart3d_campus.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh):

```bash
docker exec ros2-humble bash -lc '
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS2CLI_NO_DAEMON=1
  source /opt/ros/humble/setup.bash
  ros2 topic echo /scan --once --qos-reliability best_effort
'
```

Pi LiDAR-only launch shape:

```bash
docker exec ros2-humble bash -lc '
  source /opt/ros/humble/setup.bash
  source /root/m10p_ws/install/setup.bash
  /root/mk_m10p_dev.sh >/tmp/mk_m10p_dev.log 2>&1 || true
  ros2 run lslidar_driver lslidar_driver_node \
    --ros-args --params-file /root/m10p_ws/m10p_uart.yaml
'
```

### Jetson validation

After [start_full_native.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh):

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E '^/scan$|^/map$|^/car1/pose$'
ros2 topic hz /scan
ros2 topic echo /scan --once --qos-reliability best_effort
tail -n 80 /tmp/lidar.log
tail -n 80 /tmp/cartographer.log
```

Healthy downstream behavior:

- `/scan` is present and non-empty.
- `/map` updates through Cartographer.
- `/car1/pose` updates from the TF-derived pose publisher in [pi5_client.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_client.py).
- `/map_campus` is republished by [campus_map_bridge.py](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/campus_map_bridge.py).

## Cartographer Fit: Official Usage vs Smart3D Usage

The official vendor driver is a generic sensor node. Smart3D adds a lot of robot-specific assumptions on top of it.

The most important Smart3D assumptions are:

- `frame_id` must be `laser_link`.
- A static TF `base_link -> laser_link` must exist before Cartographer starts.
- The SLAM stack expects a single `/scan` LaserScan stream, not a point cloud.
- The SLAM config intentionally clamps usable range to `0.15 .. 12.0` in [robotcar_2d.lua](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d.lua) and [robotcar_2d_odom.lua](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d_odom.lua), even though the driver YAML allows much wider min/max values.

One local mismatch to keep in mind:

- The Cartographer Lua comments still talk about "M10P ~10Hz."
- The official `M10_P` driver code path and the observed Jetson runtime behavior are closer to 12 Hz.

That mismatch is not fatal, but the comments are stale and should not be treated as authoritative sensor-rate documentation.

## Crash Signatures and Watchdog Handling

### What the official driver already does

The official vendor driver has an internal serial reconnection path:

- it keeps reading serial packets;
- if too many reads fail, it closes and reopens the serial device.

That helps with temporary serial silence, but it does **not** solve process crashes.

### What Smart3D has actually observed

The current field handoff in [SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md) records a Pi-side failure signature:

```text
malloc(): unaligned fastbin chunk detected
```

When that happens:

- `/scan` disappears;
- Cartographer starts reporting `Queue waiting for data: (0, scan)`;
- map growth stops;
- everything above LiDAR begins to look stale even if IMU is still alive.

This is why Smart3D added an external ROS watchdog on top of the vendor driver's internal reconnect logic.

### Current Smart3D mitigation

The current watchdog is [pi_ros_watchdog.py](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/pi_ros_watchdog.py).

What it does:

- monitors `/scan`, `/IMU_data`, `/car1/pose`, and `/map`;
- if `/scan` gap exceeds the configured threshold, it restarts only `lslidar_driver_node`;
- rebuilds the device binding first via [mk_m10p_dev.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_m10p_dev.sh);
- leaves the rest of the stack alone if possible.

Jetson reuses the same watchdog logic through environment overrides injected by [start_full_native.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh), so the same LiDAR-only recovery path is available there.

Interpretation:

- the vendor driver's internal reconnect path is a transport recovery mechanism;
- the Smart3D watchdog is a process-level recovery mechanism;
- both are needed if the failure mode includes either serial silence or outright process death.

## Concrete Gaps Still Remaining

1. **The Pi LiDAR YAML is still not versioned in the canonical tree.**  
   The active Pi container references `/root/m10p_ws/m10p_uart.yaml`, but the canonical local source tree does not contain that file.

2. **The official vendor docs are internally inconsistent.**  
   README text says Foxy and "Catkin package"; `version.txt` says Humble compatibility; brochure baud/spec text does not cleanly match the actual `M10_P` code path.

3. **There is no dedicated official `M10_P` example YAML in the vendor branch.**  
   The branch ships generic `lsx10*.yaml` files and comments that mention `M10_P`, but not a clean one-file "this is the M10_P serial recipe" example.

4. **`high_reflection` is still unvalidated in Smart3D.**  
   Current runtime uses `false`. The official comments imply this parameter matters for `M10_P`, but there is no local calibration note showing why `false` was chosen.

5. **The full driver snapshot is not yet vendored or pinned in the Smart3D tree.**  
   Jetson was built from the official branch commit `8bb760c8f66c1b6964caaf3cf6f7a2a0ea24cefa`, but the canonical tree does not yet vendor that code or record that exact commit in a machine-readable way.

6. **The Pi-side crash is mitigated, not root-caused.**  
   The watchdog restart path makes the system more survivable, but it does not prove that the underlying vendor driver memory bug is gone.

7. **Jetson still needs a long soak test for the LiDAR driver.**  
   The current Jetson bring-up showed healthy `/scan`, `/map`, and `/car1/pose`, but the important remaining question is whether the Pi-side `malloc()` crash reproduces under longer native Jetson runtime.

## Recommended Next Actions

1. Copy the exact working Pi `/root/m10p_ws/m10p_uart.yaml` back into the canonical tree and diff it against [m10p_uart.yaml](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/m10p_uart.yaml).
2. Record the official vendor branch and commit hash in a checked-in bootstrap note or lockfile near [deploy_runtime_to_jetson.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/deploy_runtime_to_jetson.sh).
3. Run a longer Jetson soak with [start_full_native.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh) and watch `/tmp/lidar.log` for the Pi-side crash signature.
4. If scan quality looks odd around shiny surfaces, test the exact same runtime with `high_reflection: true` and compare map stability.
5. Treat `/dev/serial/by-id/...` as the Jetson source of truth and `/dev/m10p` as Pi compatibility glue.

## Brief Summary

The correct official M10P ROS2 path for Smart3D is the vendor repo [Lslidar_ROS2_driver](https://github.com/Lslidar/Lslidar_ROS2_driver) on the `M10P/N10P` branch, not the stale generic README text. Smart3D already uses the right high-level shape: serial mode, `/scan`, `laser_link`, Cartographer downstream, and a robot-specific watchdog around the vendor node. The biggest remaining M10P gaps are the untracked Pi-side YAML, the unresolved Pi `malloc()` crash root cause, and the need to pin the official driver snapshot more explicitly.
