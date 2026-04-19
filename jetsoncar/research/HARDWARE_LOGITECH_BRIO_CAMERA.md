# Logitech BRIO Camera Track for Smart3D

Date: 2026-04-18

## Executive take

The Logitech BRIO exists in this project as an optional, historical side-channel for live video, not as part of the accepted Pi 5 Smart3D baseline.

What I found:

- The only actual local BRIO/Logitech camera runtime file in the Smart3D trees is the old UDP helper script [robotcar_logitech_udp.sh](/Users/felixfang/Documents/Playground/smart3d-campus/robotcar_logitech_udp.sh).
- The canonical handoff explicitly says this camera chain is optional and not part of the accepted minimum baseline in [SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md).
- The current Pi baseline start script [start_full.sh](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/start_full.sh) does not start any camera stack.
- The current Jetson runtime start path [start_full_native.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh) also does not start any camera stack.
- On the actual Jetson, the BRIO is present and healthy as a standard UVC/V4L2 webcam: `046d:085e Logitech, Inc. BRIO Ultra HD Webcam`, with stable symlinks under `/dev/v4l/by-id/`.

Bottom line: if Smart3D "complete setup" means "everything ever plugged in", BRIO can be replicated on Jetson cleanly. If "baseline parity" means "the accepted Pi chain that passed goal-test", BRIO is still phase 2, not phase 1.

## 1. Hardware identity

Observed directly on the Jetson at `10.66.121.109`:

- `lsusb` reports: `046d:085e Logitech, Inc. BRIO Ultra HD Webcam`
- `v4l2-ctl --list-devices` reports one Logitech BRIO device with:
  - `/dev/video0`
  - `/dev/video1`
  - `/dev/video2`
  - `/dev/video3`
- Stable symlinks exist:
  - `/dev/v4l/by-id/usb-046d_Logitech_BRIO_6A53C801-video-index0`
  - `/dev/v4l/by-id/usb-046d_Logitech_BRIO_6A53C801-video-index1`
  - `/dev/v4l/by-id/usb-046d_Logitech_BRIO_6A53C801-video-index2`
  - `/dev/v4l/by-id/usb-046d_Logitech_BRIO_6A53C801-video-index3`

Practical interpretation:

- `video-index0` is the node to treat as the primary front camera stream.
- Do not hardcode `/dev/video0` if you want stable startup after USB reordering.
- BRIO exposes multiple V4L2 nodes. That is normal. Do not assume all four are interchangeable.

Observed format support on Jetson:

- Raw `YUYV`: up to `1920x1080`
- Raw `NV12`: up to `1920x1080`
- `MJPEG`: up to `3840x2160` and `4096x2160`

Observed control surface on Jetson via `v4l2-ctl -L`:

- brightness, contrast, saturation
- white balance auto/manual
- gain, sharpness, backlight compensation
- auto exposure / manual exposure
- pan, tilt, focus, zoom

That matches the expected "standard UVC camera with V4L2 controls" model.

## 2. Official docs / packages / tools that actually matter

### Logitech official docs

1. Logitech support says BRIO supports `ChromeOS, Linux`:
   [What operating systems support BRIO/BRIO 4K?](https://support.logi.com/hc/en-in/articles/360023363493-What-operating-systems-support-BRIO-BRIO-4K)

2. Logitech support says BRIO works with any application supporting the USB Video Class standard:
   [With which applications is BRIO/BRIO 4K compatible?](https://support.logi.com/hc/en-us/articles/360023363013-With-which-applications-is-BRIO-BRIO-4k-compatible)

3. Logitech product page gives the meaningful capture targets:
   - 4K at 30 fps
   - 1080p at up to 60 fps
   - 720p at up to 90 fps
   [BRIO product page](https://www.logitech.com/en-us/products/webcams/brio-4k-hdr-webcam.html)

4. Logitech datasheet confirms:
   - works with Windows/macOS/ChromeOS and "most platforms and operating systems"
   - Tune controls zoom/color/focus/firmware
   - Tune-dependent features are a desktop-app path, not a robot-runtime requirement
   [BRIO datasheet PDF](https://www.logitech.com/content/dam/logitech/en/video-collaboration/pdf/brio-datasheet.pdf)

5. Logitech support says firmware update tooling is provided for Windows and Mac, not Linux:
   [How do I upgrade my BRIO firmware and what platforms do you support?](https://support.logi.com/hc/en-001/articles/360023196794-How-do-I-upgrade-my-BRIO-firmware-and-what-platforms-do-you-support)

### What those official docs mean for Linux / Jetson

The Linux story is simple:

- BRIO is meant to be used as a standards-based UVC webcam.
- There is no Linux-specific Logitech runtime package that Smart3D needs in order to capture video.
- The Linux runtime tools that matter are generic:
  - `v4l-utils`
  - `ffmpeg`
  - `gstreamer1.0-*`

For robot bring-up, the correct instinct is:

- do not hunt for a special Logitech Linux SDK first
- first prove UVC/V4L2 capture
- then choose transport: direct, ROS, or UDP

### ROS packages relevant to Jetson replication

These are the relevant primary-source ROS drivers/tools:

1. `v4l2_camera`
   [docs.ros.org v4l2_camera](https://docs.ros.org/en/humble/p/v4l2_camera/__README.html)

   Key points:

- it is a ROS 2 V4L2 driver
- it exposes camera controls as ROS parameters
- it supports `image_transport`
- its docs explicitly list `Logitech BRIO 4K Pro` as proven to work with `uvcvideo`

2. `usb_cam`
   [docs.ros.org usb_cam](https://docs.ros.org/en/humble/p/usb_cam/)

   Key points:

- it is a ROS 2 driver for V4L USB cameras
- binary package is available on Humble
- it supports `image_transport`
- its documented format path is friendly to MJPEG-based webcams, which matters for BRIO high-res modes

3. `gscam`
   [ROS Index gscam](https://index.ros.org/p/gscam/)

   Key points:

- it is a ROS camera driver that uses GStreamer
- useful when you want the exact same GStreamer pipeline locally and inside ROS
- especially relevant on Jetson when you want to lean into the existing GStreamer stack

### Jetson package reality on this machine

Already installed on the Jetson:

- `v4l-utils`
- `ffmpeg`
- `gstreamer1.0-tools`
- `gstreamer1.0-plugins-good`
- `gstreamer1.0-plugins-bad`
- `ros-humble-image-transport`
- `ros-humble-image-tools`

Available via apt but not yet installed:

- `ros-humble-v4l2-camera`
- `ros-humble-usb-cam`
- `ros-humble-image-transport-plugins`
- `ros-humble-camera-info-manager`
- `ros-humble-gscam`

Suggested install set for the BRIO track on Jetson:

```bash
sudo apt update
sudo apt install -y \
  v4l-utils \
  ffmpeg \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  ros-humble-v4l2-camera \
  ros-humble-usb-cam \
  ros-humble-image-transport \
  ros-humble-image-transport-plugins \
  ros-humble-camera-info-manager \
  ros-humble-image-tools \
  ros-humble-rqt-image-view \
  ros-humble-gscam
```

## 3. Pi baseline behavior

### What I found in the local trees

Canonical tree:

- [SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md) explicitly calls the Logitech camera chain optional and says it is not part of the accepted minimum baseline.
- [SMART3D_JETSON_ORIN_NX16_MIGRATION_FROM_PI5.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_JETSON_ORIN_NX16_MIGRATION_FROM_PI5.md) warns that older trees will confuse camera-stack assumptions.
- [smart3d_campus.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh) does not stage or orchestrate a BRIO camera chain.
- [start_full_native.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh) does not stage a BRIO camera chain.

Historical tree:

- [robotcar_logitech_udp.sh](/Users/felixfang/Documents/Playground/smart3d-campus/robotcar_logitech_udp.sh) is the only actual Logitech/BRIO camera runtime script I found in the Smart3D trees.

Deploy tree:

- [start_full.sh](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/start_full.sh) has no camera launch path.
- [restore_full_stack_desktop.sh](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/linux_desktop/restore_full_stack_desktop.sh) references `camera-info-manager` and an Orbbec camera stack, but that is a different hardware track, not BRIO.

### What the Pi actually used

The accepted Pi baseline did not require the BRIO camera.

The historical Pi camera behavior was:

- capture from `/dev/video0`
- use `ffmpeg` on the robot computer
- encode low-latency H.264
- send MPEG-TS over UDP to the i9
- use `ffplay` on the i9 to view it

That exact historical implementation lives in:

- [robotcar_logitech_udp.sh](/Users/felixfang/Documents/Playground/smart3d-campus/robotcar_logitech_udp.sh)

Important:

- this was a side channel for operator video, not a required part of the mapping/planning chain
- current handoff docs explicitly say "camera off unless specifically needed"
- recent field notes suggest live video may worsen lag or at least make debugging noisier

So the correct statement is:

- BRIO was researched and used historically
- BRIO is not part of the accepted Pi baseline contract that must pass `goal-test`

## 4. Jetson replication paths

## Path A: Native Linux V4L2 capture

Use this first. It is the cleanest proof that BRIO itself is healthy.

Recommended device:

- `/dev/v4l/by-id/usb-046d_Logitech_BRIO_6A53C801-video-index0`

Why:

- stable across reboots and USB reorder events
- avoids surprise renumbering of `/dev/video0`

Exact capture sanity check:

```bash
v4l2-ctl -d /dev/v4l/by-id/usb-046d_Logitech_BRIO_6A53C801-video-index0 --list-formats-ext
```

Exact single-frame validation:

```bash
ffmpeg -hide_banner -loglevel error \
  -f v4l2 \
  -input_format mjpeg \
  -video_size 1280x720 \
  -framerate 30 \
  -i /dev/v4l/by-id/usb-046d_Logitech_BRIO_6A53C801-video-index0 \
  -frames:v 1 /tmp/brio_test.jpg

file /tmp/brio_test.jpg
```

I ran the equivalent test on the Jetson and it produced a valid `1280x720` JPEG.

When to use Path A:

- hardware bring-up
- regression test after cable or USB topology changes
- proving the camera itself works before blaming ROS or network transport

## Path B: Native GStreamer path

This is the most Jetson-friendly non-ROS path because Jetson already ships a strong GStreamer stack.

Exact device inventory:

```bash
gst-device-monitor-1.0 Video/Source
```

Exact no-output pipeline validation:

```bash
gst-launch-1.0 -q \
  v4l2src device=/dev/v4l/by-id/usb-046d_Logitech_BRIO_6A53C801-video-index0 num-buffers=30 \
  ! image/jpeg,width=1280,height=720,framerate=30/1 \
  ! jpegparse \
  ! fakesink
```

I ran the equivalent test on the Jetson and it completed successfully.

Inference:

- If the camera is eventually kept in the "full" Jetson build, GStreamer is the natural place to do hardware-aware optimization later.
- Do not start there. Start with V4L2 proof first.

## Path C: ROS image pipeline

There are three realistic ROS choices here.

### C1. `v4l2_camera` for the clean V4L2 path

Why it is attractive:

- official docs list `Logitech BRIO 4K Pro` as proven to work
- exposes camera controls as ROS parameters
- integrates with `image_transport`

Important constraint from the package docs:

- documented input pixel formats are `YUYV`, `UYVY`, and `GREY`
- that makes it clean for raw modes, but not obviously the best path if your real target is BRIO MJPEG 4K

Exact install:

```bash
sudo apt install -y ros-humble-v4l2-camera ros-humble-image-transport-plugins ros-humble-camera-info-manager
```

Exact run command:

```bash
source /opt/ros/humble/setup.bash

ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:="/dev/v4l/by-id/usb-046d_Logitech_BRIO_6A53C801-video-index0" \
  -p pixel_format:="YUYV" \
  -p image_size:="[1280,720]"
```

Exact validation:

```bash
ros2 topic list | grep image
ros2 topic hz /image_raw
ros2 run rqt_image_view rqt_image_view
```

Recommended use:

- if you want a conservative ROS camera node
- if `1280x720` or `1920x1080` raw is enough
- if exposing standard V4L2 controls as ROS params is valuable

### C2. `usb_cam` if you want BRIO's MJPEG modes

Why it is attractive:

- better fit when the webcam's practical high-res path is MJPEG
- Humble binary package is available
- supports `image_transport`

Exact install:

```bash
sudo apt install -y ros-humble-usb-cam ros-humble-image-transport-plugins ros-humble-camera-info-manager
```

Exact quickstart from official docs:

```bash
source /opt/ros/humble/setup.bash
ros2 run usb_cam usb_cam_node_exe
```

Exact format discovery from official docs:

```bash
source /opt/ros/humble/setup.bash
ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:="test"
```

Practical recommendation:

- use `usb_cam` if you want to chase `MJPEG` modes like `1920x1080@60` or `3840x2160@30`
- use a params file once the format is chosen

### C3. `gscam` if you want the same GStreamer pipeline inside ROS

Why it is attractive:

- ROS wrapper around GStreamer
- good fit if you later want one pipeline definition to feed ROS and non-ROS consumers

Exact install:

```bash
sudo apt install -y ros-humble-gscam
```

Recommended use:

- only after plain GStreamer capture is already proven
- especially useful if Jetson-specific encoding/transport becomes important

## Path D: Historical UDP bridge parity with Pi

If the goal is "replicate what the old Pi side-video chain did", then this is the historical design:

- robot side: `ffmpeg`
- desktop side: `ffplay`
- transport: MPEG-TS over UDP
- default port: `5001`

That exact historical implementation is:

- [robotcar_logitech_udp.sh](/Users/felixfang/Documents/Playground/smart3d-campus/robotcar_logitech_udp.sh)

Historical sender shape, adapted to use the stable BRIO path on Jetson:

```bash
ffmpeg -hide_banner -loglevel error \
  -fflags nobuffer -flags low_delay \
  -f v4l2 -thread_queue_size 2 \
  -input_format yuyv422 \
  -video_size 640x480 \
  -framerate 30 \
  -i /dev/v4l/by-id/usb-046d_Logitech_BRIO_6A53C801-video-index0 \
  -an \
  -c:v libx264 -preset ultrafast -tune zerolatency -bf 0 -g 15 -keyint_min 15 \
  -b:v 1500k -maxrate 1500k -bufsize 300k -pix_fmt yuv420p \
  -flush_packets 1 -muxdelay 0 -muxpreload 0 \
  -f mpegts "udp://<I9_IP>:5001?pkt_size=1316&buffer_size=65535"
```

Historical i9 viewer shape:

```bash
ffplay -fflags nobuffer+discardcorrupt \
  -flags low_delay \
  -framedrop \
  -analyzeduration 0 \
  -probesize 16384 \
  -sync ext \
  -window_title robotcar-front \
  -f mpegts "udp://0.0.0.0:5001?fifo_size=8192&overrun_nonfatal=1&buffer_size=65535"
```

Important caution:

- this is historical parity, not a canonical requirement
- keep this detached from `goal-test`
- only bring it back after core mapping / planning / chassis health is already stable

## 5. Exact validation commands

Run these in order.

### Stage 0: identity

```bash
lsusb | grep -i Logitech
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id
```

Expected good signs:

- `046d:085e Logitech, Inc. BRIO Ultra HD Webcam`
- one BRIO device with multiple `/dev/video*` nodes
- stable `/dev/v4l/by-id/usb-046d_Logitech_BRIO_...` links

### Stage 1: format audit

```bash
v4l2-ctl -d /dev/v4l/by-id/usb-046d_Logitech_BRIO_6A53C801-video-index0 --list-formats-ext
ffmpeg -hide_banner -f v4l2 -list_formats all -i /dev/v4l/by-id/usb-046d_Logitech_BRIO_6A53C801-video-index0
v4l2-ctl -d /dev/v4l/by-id/usb-046d_Logitech_BRIO_6A53C801-video-index0 -L
```

Expected good signs:

- `MJPEG` modes up to 4K
- usable raw modes
- normal UVC controls

### Stage 2: one-frame capture

```bash
rm -f /tmp/brio_test.jpg
ffmpeg -hide_banner -loglevel error \
  -f v4l2 \
  -input_format mjpeg \
  -video_size 1280x720 \
  -framerate 30 \
  -i /dev/v4l/by-id/usb-046d_Logitech_BRIO_6A53C801-video-index0 \
  -frames:v 1 /tmp/brio_test.jpg

file /tmp/brio_test.jpg
```

Expected good sign:

- `JPEG image data ... 1280x720`

### Stage 3: GStreamer live pull

```bash
gst-device-monitor-1.0 Video/Source

gst-launch-1.0 -q \
  v4l2src device=/dev/v4l/by-id/usb-046d_Logitech_BRIO_6A53C801-video-index0 num-buffers=30 \
  ! image/jpeg,width=1280,height=720,framerate=30/1 \
  ! jpegparse \
  ! fakesink
```

Expected good sign:

- device appears as `Logitech BRIO`
- pipeline exits cleanly without caps negotiation failure

### Stage 4: ROS node validation

For `v4l2_camera`:

```bash
source /opt/ros/humble/setup.bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:="/dev/v4l/by-id/usb-046d_Logitech_BRIO_6A53C801-video-index0" \
  -p pixel_format:="YUYV" \
  -p image_size:="[1280,720]"
```

In another shell:

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /image_raw
ros2 run rqt_image_view rqt_image_view
```

For `usb_cam`:

```bash
source /opt/ros/humble/setup.bash
ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:="test"
```

Then choose a real params file once the reported device formats match what you want.

### Stage 5: optional UDP viewer test

Sender on robot:

```bash
ffmpeg -hide_banner -loglevel error \
  -fflags nobuffer -flags low_delay \
  -f v4l2 -thread_queue_size 2 \
  -input_format yuyv422 \
  -video_size 640x480 \
  -framerate 30 \
  -i /dev/v4l/by-id/usb-046d_Logitech_BRIO_6A53C801-video-index0 \
  -an \
  -c:v libx264 -preset ultrafast -tune zerolatency -bf 0 -g 15 -keyint_min 15 \
  -b:v 1500k -maxrate 1500k -bufsize 300k -pix_fmt yuv420p \
  -flush_packets 1 -muxdelay 0 -muxpreload 0 \
  -f mpegts "udp://<I9_IP>:5001?pkt_size=1316&buffer_size=65535"
```

Viewer on i9:

```bash
ffplay -fflags nobuffer+discardcorrupt \
  -flags low_delay \
  -framedrop \
  -analyzeduration 0 \
  -probesize 16384 \
  -sync ext \
  -window_title robotcar-front \
  -f mpegts "udp://0.0.0.0:5001?fifo_size=8192&overrun_nonfatal=1&buffer_size=65535"
```

## 6. Recommendation: phase 1 or phase 2?

Recommendation: keep BRIO in phase 2.

Reasoning:

1. The canonical local handoff explicitly says the Logitech UDP camera chain is optional and not part of the accepted minimum baseline in [SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md).
2. The accepted Pi runtime contract in [SMART3D_JETSON_ORIN_NX16_MIGRATION_FROM_PI5.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/SMART3D_JETSON_ORIN_NX16_MIGRATION_FROM_PI5.md) is about `/scan`, `/tf`, `/map`, `/car1/pose`, `/map_campus`, and `/cmd_vel`, not camera topics.
3. The actual Pi reference launcher [start_full.sh](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/start_full.sh) does not start a camera stack.
4. Historical notes already warned that live video can correlate with worse lag or noisier debugging.

So:

- Phase 1:
  - enumerate BRIO
  - prove V4L2 capture on Jetson
  - optionally prove a ROS image publisher locally
- Phase 2:
  - optional operator video
  - optional ROS camera topic integration
  - optional UDP bridge to i9
  - optional logging / recording / perception consumers

If someone insists on "full hardware completeness", the safest wording is:

- BRIO should be installed and documented now
- BRIO should not be part of the required `goal-test` gate

## 7. My recommendation for the next session

Do this in this order:

1. Keep the current navigation stack as the authority:
   - [start_full_native.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh)
   - [deploy_runtime_to_jetson.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/deploy_runtime_to_jetson.sh)

2. Add BRIO only as a separate feature flag or standalone launcher, not inside the core SLAM/planner bring-up.

3. Use `/dev/v4l/by-id/...video-index0`, never raw `/dev/video0`, for the camera path.

4. Start with:
   - native V4L2 proof
   - then `v4l2_camera` or `usb_cam`
   - only then optional UDP bridge

5. If the end goal is just operator preview, historical UDP parity is enough.

6. If the end goal is ROS-native perception, prefer a ROS driver first and only add UDP if a human viewer is still needed.

## Summary

I found that BRIO is real hardware on the Jetson and works normally as a UVC/V4L2 webcam, but it is not part of the accepted Pi Smart3D baseline. The only local Smart3D camera implementation is the old optional UDP script in [robotcar_logitech_udp.sh](/Users/felixfang/Documents/Playground/smart3d-campus/robotcar_logitech_udp.sh). The correct Jetson replication strategy is to treat BRIO as a phase 2 module: prove V4L2 first, then choose either `v4l2_camera`, `usb_cam`, or the historical ffmpeg-to-UDP bridge depending on whether the need is ROS perception or human preview.
