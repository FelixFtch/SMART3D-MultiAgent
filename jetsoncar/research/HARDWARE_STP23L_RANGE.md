# STP23L / Single-Point Range Sensor Research

Date: 2026-04-18

This note covers the Smart3D single-point LiDAR / forward range-sensor track only.

## 1. Local file inventory actually found

Canonical local Smart3D tree:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/deploy_runtime_to_jetson.sh`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_m10p_dev.sh`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/pi_ros_watchdog.py`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md`

Deploy tree:

- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/stp23l_range.py`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/mk_stp23l_dev.sh`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/README.md`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/install_cartographer.sh`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/carto_bringup.sh`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/cartographer_launch.py`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/start_full.sh`

What this inventory means:

- The actual STP23L implementation lives only in `deploy/cartographer_config/`.
- The canonical Smart3D tree does not contain an active STP23L runtime of its own.
- The canonical Jetson tree only copies the STP23L helper files over; it does not start them.

## 2. Hardware identity confidence

### Confidence level

- High confidence: the Smart3D codebase intends to talk to an LDROBOT `STP-23L`-style single-point DTOF range sensor on serial `5A6C069316`.
- Medium confidence: the physical module currently attached to the robot is exactly an STP-23L and not some other LDROBOT serial range module or a mislabeled fallback device.

### Evidence for the high-confidence part

From local files:

- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/stp23l_range.py` explicitly identifies the hardware as `STP-23L Single-Point Range Sensor`.
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/mk_stp23l_dev.sh` binds USB serial `5A6C069316` to `/dev/stp23l`.
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/stp23l_range.py` expects:
  - UART `230400`
  - header `0xAAAAAAAA`
  - distance command `0x02`
  - payload length `0x00B8` (184 bytes)
  - topic `/front_range`
  - estop topic `/emergency_stop`

From live hardware observation on the current Jetson:

- `/dev/ttyACM2` is `ID_SERIAL=1a86_USB_Single_Serial_5A6C069316`.
- Reading `/dev/ttyACM2` at `230400` produced bytes beginning with:

```text
aaaaaaaa00020000b800...
```

That matches the local parser assumptions exactly:

- `aaaaaaaa` -> header
- `02` -> distance command
- `00b8` -> 184-byte data length

### Why the exact hardware identity is still only medium confidence

There is a real naming conflict in the current Smart3D tree:

- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/mk_stp23l_dev.sh` says serial `5A6C069316` is the STP23L.
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_m10p_dev.sh` also treats the same serial `5A6C069316` as a fallback `M10P` serial.
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md` repeats that same fallback-M10P assumption.

So the software history currently contains two mutually inconsistent stories about the same USB serial number.

My best reading is:

- The `stp23l_range.py` parser is real and matches live bytes.
- The `mk_m10p_dev.sh` fallback entry is likely historical overreach or a convenience fallback that can accidentally grab the single-point sensor when the real M10P is missing.

That is an inference from the local files plus live serial capture, not something explicitly stated in official vendor docs.

## 3. Official docs / packages / protocol status

### Official product identity

Most likely official product:

- LDROBOT `STP-23L` Point LiDAR / single-point DTOF range sensor.

Primary-source or vendor-source material found:

- Official LDROBOT product URL: [https://www.ldrobot.com/product/en/122](https://www.ldrobot.com/product/en/122)
  - In browser crawl it is JavaScript-only and not directly readable here.
- Official LDROBOT customer docs portal: [https://gitlab.ldrobot.com/users/sign_in](https://gitlab.ldrobot.com/users/sign_in)
  - The portal says customer specification docs, firmware packages, and host-computer tools are maintained there and require account access.
- Public STP-23L datasheet mirror with LDROBOT copyright: [https://cdn.robotshop.com/rbm/54696a9c-3d36-46d3-9713-0702899f1a68/0/016e7679-e434-4126-bd9f-a6dcffeb6beb/3e2d4afc_ldrobot_stp-23l_datasheet_en_v1.0.pdf](https://cdn.robotshop.com/rbm/54696a9c-3d36-46d3-9713-0702899f1a68/0/016e7679-e434-4126-bd9f-a6dcffeb6beb/3e2d4afc_ldrobot_stp-23l_datasheet_en_v1.0.pdf)

### What the public datasheet gives us

The mirrored datasheet says:

- STP-23L is a point LiDAR using DTOF.
- Range: `0.03m` to `7.5m`
- Frequency: `120 Hz`
- Interface: `UART`
- Baud rate: `230400`
- Voltage: `5V`
- UART is one-way transmission after power-on
- `RX` is for production use only and does not support user-side commands
- Demo tooling is available through vendor FAE / host computer

Important implication:

- This is not a sensor that needs a heavy official ROS stack to be useful.
- The public official model is "power it, read serial stream, parse frames."

### Official public packages I found

Official LDROBOT public GitHub organization:

- [https://github.com/ldrobotSensorTeam](https://github.com/ldrobotSensorTeam)

Official public packages for related LDROBOT LiDAR families:

- DTOF ROS2 package: [https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2)
- DTOF Linux SDK: [https://github.com/ldrobotSensorTeam/ldlidar_stl_sdk](https://github.com/ldrobotSensorTeam/ldlidar_stl_sdk)
- New unified SDK: [https://github.com/ldrobotSensorTeam/ldlidar_sdk](https://github.com/ldrobotSensorTeam/ldlidar_sdk)

But the official public model list in those repos does **not** include STP-23L:

- `ldlidar_stl_ros2` names `LD06`, `LD19`, and `STL-27L`.
- `ldlidar_stl_sdk` names `LD06` and `LD19`.
- `ldlidar_sdk` names `LD06`, `LD19`, `LD14`, and `LD14P`.

### Bottom line on official software support

- I did **not** find a public official STP-23L ROS1/ROS2 package.
- I did **not** find a public official STP-23L Linux SDK repository.
- I **did** find official evidence that vendor-side host-computer tools and customer docs exist behind LDROBOT GitLab / FAE access.

So the best current conclusion is:

- Public official integration path for STP-23L is the datasheet plus vendor host-computer tooling through FAE, not a public ROS package.
- Smart3D's local `stp23l_range.py` is therefore a custom parser, not a copy of a public official ROS driver.

### Public protocol status

Official-public facts:

- UART, one-way
- `230400 8N1`
- data starts automatically after power-on

Not fully public in the sources I found:

- full byte-level frame layout
- checksum rules
- point payload layout

The byte-level layout used by Smart3D is an inference from:

- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/stp23l_range.py`
- live serial bytes from the current sensor

That inference appears internally consistent and probably correct, but it is not the same thing as a vendor-published protocol manual.

## 4. Pi baseline behavior

### What the Pi-side code says

The STP23L files exist in deploy:

- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/stp23l_range.py`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/mk_stp23l_dev.sh`

But the accepted Pi baseline does **not** start them:

- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/start_full.sh`
  - starts static TF, M10P LiDAR, IMU, IMU bridge, wheel odom, Cartographer
  - does not launch `stp23l_range.py`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/carto_bringup.sh`
  - also starts only M10P LiDAR, IMU, static TF, Cartographer
  - does not launch `stp23l_range.py`
- `/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/cartographer_launch.py`
  - remaps only `/IMU_data -> /imu`
  - has no `front_range` or `emergency_stop` integration
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/pi_ros_watchdog.py`
  - watches `/scan`, `/IMU_data`, `/car1/pose`, `/map`
  - does not watch `/front_range`
- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md`
  - describes accepted runtime around `/scan`, `/map_campus`, `/car1/pose`, planner, and `goal-test`
  - does not treat STP23L as a required success condition

### Practical conclusion for Pi

On Pi, STP23L is not part of the accepted baseline that proved the car chain works.

It is only:

- an auxiliary sensor implementation present in deploy,
- with a helper script,
- but not actually wired into the trusted startup sequence.

### Accepted-baseline verdict

- Baseline sensor: `M10P` 2D LiDAR on `/scan`
- Auxiliary sensor: `STP-23L` single-point range sensor on `/front_range`

## 5. Jetson replication steps if we keep it

### Current status on Jetson

The Jetson deployment script already copies the helper files:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/deploy_runtime_to_jetson.sh`

It copies:

- `stp23l_range.py`
- `mk_stp23l_dev.sh`

But the Jetson runtime does not start them:

- `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh`

So Jetson currently has STP23L files present but inactive.

### Recommended replication approach

If we keep this sensor on Jetson, do it as an optional sidecar node, not as a boot-critical dependency.

Recommended order:

1. Bring up the normal trusted chain first:
   - M10P
   - IMU
   - odom
   - Cartographer
   - campus bridge
   - `pi5_client.py`
   - i9 planner / goal-test
2. Only after that, start the STP23L node separately.
3. Validate that it publishes sane `/front_range` values.
4. Do not wire `/emergency_stop` into drivetrain gating until there is a reviewed consumer.

### Recommended device path

Prefer the stable by-id path over `/dev/stp23l`:

```bash
/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6C069316-if00
```

Reason:

- the current canonical Smart3D tree already had one symlink-misidentification problem on this serial number,
- and by-id is less ambiguous than reusing `/dev/m10p` or `/dev/stp23l`.

### Recommended Jetson launch command

After the main native runtime is already healthy:

```bash
ssh jetson@10.66.121.109 'bash -lc "
source /opt/ros/humble/setup.bash &&
python3 /home/jetson/SMART3D/jetson_runtime/stp23l_range.py --ros-args \
  -p serial_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6C069316-if00 \
  -p baud_rate:=230400 \
  -p frame_id:=stp23l_link \
  -p min_range:=0.03 \
  -p max_range:=7.5 \
  -p emergency_stop_dist:=0.25 \
  >/tmp/stp23l.log 2>&1
"'
```

If you specifically want the old helper style:

```bash
ssh jetson@10.66.121.109 'bash -lc "
cd /home/jetson/SMART3D/jetson_runtime &&
sudo ./mk_stp23l_dev.sh &&
source /opt/ros/humble/setup.bash &&
python3 ./stp23l_range.py --ros-args -p serial_port:=/dev/stp23l
"'
```

I do **not** recommend the helper-first route as the default, because the by-id path is clearer.

## 6. Exact validation commands

### A. Confirm the device identity

On Jetson:

```bash
ssh jetson@10.66.121.109 'bash -lc "
ls -l /dev/serial/by-id | grep 5A6C069316
"'
```

Expected:

- one entry pointing to `ttyACM2` or whichever `ttyACM*` the sensor landed on

### B. Confirm the sensor is really streaming UART data

```bash
ssh jetson@10.66.121.109 'bash -lc "
python3 - <<\"PY\"
import binascii, serial
ser = serial.Serial(\"/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6C069316-if00\", 230400, timeout=1)
data = ser.read(96)
ser.close()
print(binascii.hexlify(data).decode())
PY
"'
```

Expected prefix:

```text
aaaaaaaa00020000b800
```

If you do not see that or something very close to it, do not trust the parser yet.

### C. Run the ROS node by itself

```bash
ssh jetson@10.66.121.109 'bash -lc "
source /opt/ros/humble/setup.bash &&
python3 /home/jetson/SMART3D/jetson_runtime/stp23l_range.py --ros-args \
  -p serial_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6C069316-if00 \
  -p baud_rate:=230400 \
  -p frame_id:=stp23l_link
"'
```

Expected startup log:

- `STP-23L connected: ... @ 230400`

### D. Validate ROS topics

In another terminal:

```bash
ssh jetson@10.66.121.109 'bash -lc "
source /opt/ros/humble/setup.bash &&
ros2 topic echo /front_range --once
"'
```

```bash
ssh jetson@10.66.121.109 'bash -lc "
source /opt/ros/humble/setup.bash &&
ros2 topic hz /front_range
"'
```

```bash
ssh jetson@10.66.121.109 'bash -lc "
source /opt/ros/humble/setup.bash &&
ros2 topic echo /emergency_stop --once
"'
```

Healthy expectations:

- `/front_range` publishes `sensor_msgs/Range`
- values are inside `0.03 .. 7.5`
- rate is roughly around the configured sensor rate, or at least clearly alive
- `/emergency_stop` flips to `true` only when an obstacle is very close

### E. Verify nobody important depends on it yet

Local repo check:

```bash
rg -n --hidden -S "/front_range|/emergency_stop|front_range|emergency_stop" \
  /Users/felixfang/Documents/private-repos/SMART3D-MultiAgent \
  /Users/felixfang/Documents/Playground/connection-guideline/smart3d
```

Current result:

- only the STP23L node itself publishes those topics
- no accepted baseline node consumes them

## 7. Should it be included in the full runtime now?

### Recommendation

Not as a required default runtime component.

### What I do recommend right now

- Keep the files deployed.
- Keep it available as an auxiliary sidecar sensor.
- Allow manual start for validation, logging, and future safety work.
- Do not make the full Smart3D boot depend on it.
- Do not block `goal-test` success on it.

### Why

Because today it fails all four tests for "baseline-critical":

1. It is not part of the Pi accepted startup path.
2. It is not part of the Jetson accepted startup path.
3. No trusted runtime node consumes `/emergency_stop`.
4. The device serial identity is currently entangled with an old fallback-M10P assumption.

If the user wants "full hardware coverage", the right interpretation is:

- deploy it,
- understand it,
- validate it,
- but keep it optional until the integration story is real.

## 8. Ambiguities that still need physical confirmation

1. Is the module physically labeled `STP-23L` on the board or housing?
2. Is serial `5A6C069316` always the single-point sensor, or can another LDROBOT board reuse that adapter path in this platform?
3. What is the actual mounting pose relative to `base_link`?
   - `stp23l_link` exists only as a name right now; there is no trusted TF integration in baseline.
4. Is the sensor intended only for front bumper emergency stop, or also for close-range docking / wall-distance control?
5. Does the current USB adapter board expose true 3.3V UART-level STP23L output through a CDC ACM bridge, or is there any vendor-side framing done by an adapter board?
6. Is there any unpublished vendor host-computer or protocol note already sitting in LDROBOT GitLab or in the project's historical notes?
7. Should `mk_m10p_dev.sh` stop treating `5A6C069316` as an M10P fallback?

That last question matters a lot. Right now it is the biggest avoidable confusion in this whole sensor track.

## 9. Final conclusion

Best current classification:

- `STP-23L` is a real auxiliary single-point DTOF range sensor present on the robot.
- Smart3D has a plausible custom parser for it.
- The parser matches live bytes on the current Jetson.
- It is not part of the accepted Pi baseline and not part of the accepted Jetson runtime.
- The correct operational status today is "understood and available, but optional."

If a later session wants to promote it into the default full runtime, the minimum bar should be:

1. confirm the physical module label,
2. clean up the serial-number conflict with `mk_m10p_dev.sh`,
3. add a reviewed consumer for `/front_range` or `/emergency_stop`,
4. verify that enabling it does not destabilize the proven `goal-test` chain.
