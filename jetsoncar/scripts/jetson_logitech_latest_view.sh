#!/usr/bin/env bash
set -euo pipefail

I9_HOST="${I9_HOST:-felix@alienware-ssh.mofangpi.net}"
I9_KEY="${I9_KEY:-$HOME/.ssh/rpi5_robot}"
I9_SSH_PROXY_COMMAND="${I9_SSH_PROXY_COMMAND-/opt/homebrew/bin/cloudflared access ssh --hostname %h}"
I9_IP="${I9_IP:-}"
I9_NET_IFACE="${I9_NET_IFACE:-wlp3s0}"

JETSON_HOST="${JETSON_HOST:-jetson@10.66.121.109}"
JETSON_SSH_PROXY_COMMAND="${JETSON_SSH_PROXY_COMMAND-ssh -i $HOME/.ssh/rpi5_robot -o StrictHostKeyChecking=no -o UserKnownHostsFile=/tmp/codex_known_hosts_i9 -o ProxyCommand=\"/opt/homebrew/bin/cloudflared access ssh --hostname alienware-ssh.mofangpi.net\" felix@alienware-ssh.mofangpi.net -W %h:%p}"

PORT="${PORT:-5001}"
JETSON_VIDEO_DEV="${JETSON_VIDEO_DEV:-/dev/v4l/by-id/usb-046d_Logitech_BRIO_6A53C801-video-index0}"
VIDEO_SIZE="${VIDEO_SIZE:-1280x720}"
FPS="${FPS:-30}"
BITRATE="${BITRATE:-2500k}"
BUFSIZE="${BUFSIZE:-256k}"
KEYINT="${KEYINT:-15}"

run_i9() {
  local -a args
  args=(
    -i "$I9_KEY"
    -o StrictHostKeyChecking=no
  )
  if [ -n "${I9_SSH_PROXY_COMMAND:-}" ]; then
    args+=(-o ProxyCommand="$I9_SSH_PROXY_COMMAND")
  fi
  ssh "${args[@]}" "$I9_HOST" "$@"
}

run_jetson() {
  local -a args
  args=(
    -o StrictHostKeyChecking=no
  )
  if [ -n "${JETSON_SSH_PROXY_COMMAND:-}" ]; then
    args+=(-o ProxyCommand="$JETSON_SSH_PROXY_COMMAND")
  fi
  ssh "${args[@]}" "$JETSON_HOST" "$@"
}

refresh_i9_ip() {
  local detected
  detected="$(run_i9 "bash -lc \"ip -4 -o addr show ${I9_NET_IFACE} | awk '{print \\\$4}' | cut -d/ -f1 | head -n1\"")"
  detected="$(printf '%s' "$detected" | tr -d '\r\n')"
  if [ -n "$detected" ]; then
    I9_IP="$detected"
  fi
}

stop_ros_camera() {
  run_jetson "bash -lc 'pids=\$(pgrep -f \"jetson-brio-ros-camera|[v]4l2_camera_node|[v]4l2_camera \" || true); for pid in \$pids; do [ \"\$pid\" != \"\$\$\" ] && [ \"\$pid\" != \"\$PPID\" ] && kill \"\$pid\" 2>/dev/null || true; done'"
}

start_ros_camera() {
  stop_ros_camera
  run_jetson "bash -lc 'set +u; source /opt/ros/humble/setup.bash; set -u; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///tmp/cyclone-campus-jetson.xml; nohup bash -lc \"exec -a jetson-brio-ros-camera ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=${JETSON_VIDEO_DEV} -p output_encoding:=rgb8\" >/tmp/v4l2_camera.log 2>&1 < /dev/null &'"
}

stop_udp() {
  run_i9 "bash -lc 'pids=\$(pgrep -f \"jetson-brio-udp-view|[f]fplay .*jetson-brio-i9\" || true); for pid in \$pids; do [ \"\$pid\" != \"\$\$\" ] && [ \"\$pid\" != \"\$PPID\" ] && kill \"\$pid\" 2>/dev/null || true; done'"
  run_jetson "bash -lc 'pids=\$(pgrep -f \"jetson-brio-udp-sender|[f]fmpeg .*udp://.*:${PORT}\" || true); for pid in \$pids; do [ \"\$pid\" != \"\$\$\" ] && [ \"\$pid\" != \"\$PPID\" ] && kill \"\$pid\" 2>/dev/null || true; done'"
}

stop_local() {
  run_jetson "bash -lc 'pids=\$(pgrep -f \"jetson-brio-local-view|[f]fplay .*jetson-brio-local\" || true); for pid in \$pids; do [ \"\$pid\" != \"\$\$\" ] && [ \"\$pid\" != \"\$PPID\" ] && kill \"\$pid\" 2>/dev/null || true; done'"
}

start_udp_sender() {
  refresh_i9_ip
  stop_ros_camera
  run_jetson "bash -lc 'nohup bash -lc \"exec -a jetson-brio-udp-sender ffmpeg -hide_banner -loglevel error -fflags nobuffer -flags low_delay -f v4l2 -thread_queue_size 1 -input_format mjpeg -video_size ${VIDEO_SIZE} -framerate ${FPS} -i ${JETSON_VIDEO_DEV} -an -c:v libx264 -preset ultrafast -tune zerolatency -bf 0 -g ${KEYINT} -keyint_min ${KEYINT} -b:v ${BITRATE} -maxrate ${BITRATE} -bufsize ${BUFSIZE} -pix_fmt yuv420p -flush_packets 1 -muxdelay 0 -muxpreload 0 -f mpegts udp://${I9_IP}:${PORT}?pkt_size=1316\\&buffer_size=65535\" >/tmp/jetson_brio_udp_sender.log 2>&1 < /dev/null &'"
}

start_udp_viewer() {
  run_i9 "bash -lc 'XAUTH=/run/user/1000/gdm/Xauthority; [ -f \"\$XAUTH\" ] || XAUTH=/home/felix/.Xauthority; nohup bash -lc \"export DISPLAY=:1; export XAUTHORITY=\$XAUTH; exec -a jetson-brio-udp-view ffplay -fflags nobuffer+discardcorrupt -flags low_delay -framedrop -analyzeduration 0 -probesize 8192 -sync ext -window_title jetson-brio-i9 -f mpegts udp://0.0.0.0:${PORT}?fifo_size=1024\\&overrun_nonfatal=1\\&buffer_size=65535\" >/tmp/jetson_brio_i9_ffplay.log 2>&1 < /dev/null &'"
}

start_local_view() {
  stop_ros_camera
  run_jetson "bash -lc 'XAUTH=/run/user/1000/gdm/Xauthority; [ -f \"\$XAUTH\" ] || XAUTH=/home/jetson/.Xauthority; nohup bash -lc \"export DISPLAY=:0; export XAUTHORITY=\$XAUTH; exec -a jetson-brio-local-view ffplay -fflags nobuffer+discardcorrupt -flags low_delay -framedrop -window_title jetson-brio-local -f v4l2 -input_format mjpeg -video_size ${VIDEO_SIZE} -framerate ${FPS} -i ${JETSON_VIDEO_DEV}\" >/tmp/jetson_brio_local.log 2>&1 < /dev/null &'"
}

status() {
  echo "== i9 =="
  run_i9 "bash -lc 'ps -ef | grep -E \"jetson-brio-udp-view|[f]fplay .*jetson-brio-i9\" | grep -v grep || true; ss -tunlp | grep -E \":${PORT}\\b\" || true; tail -n 20 /tmp/jetson_brio_i9_ffplay.log 2>/dev/null || true'"
  echo "== jetson =="
  run_jetson "bash -lc 'ps -ef | grep -E \"jetson-brio-udp-sender|jetson-brio-local-view|jetson-brio-ros-camera|[f]fmpeg .*:${PORT}|[f]fplay .*jetson-brio-local|[v]4l2_camera\" | grep -v grep || true; tail -n 20 /tmp/jetson_brio_udp_sender.log 2>/dev/null || true; tail -n 20 /tmp/jetson_brio_local.log 2>/dev/null || true'"
}

usage() {
  cat <<EOF
Usage: $(basename "$0") <udp|local|stop|status|ros-camera>

udp         Start low-latency Jetson BRIO sender and i9 viewer.
local       Start low-latency BRIO preview on Jetson display :0.
stop        Stop UDP and local preview processes.
status      Show sender/viewer status and recent logs.
ros-camera  Restore ROS v4l2_camera BRIO publisher on Jetson.

Examples:
  Direct LAN:
    I9_HOST='felix@10.66.112.64' I9_SSH_PROXY_COMMAND='' \\
    JETSON_HOST='jetson@10.66.121.109' JETSON_SSH_PROXY_COMMAND='' \\
    $(basename "$0") udp
EOF
}

case "${1:-udp}" in
  udp)
    stop_udp
    stop_local
    start_udp_viewer
    sleep 1
    start_udp_sender
    sleep 2
    status
    ;;
  local)
    stop_udp
    stop_local
    start_local_view
    sleep 2
    status
    ;;
  stop)
    stop_udp
    stop_local
    ;;
  status)
    status
    ;;
  ros-camera)
    stop_udp
    stop_local
    start_ros_camera
    sleep 2
    status
    ;;
  *)
    usage
    exit 1
    ;;
esac
