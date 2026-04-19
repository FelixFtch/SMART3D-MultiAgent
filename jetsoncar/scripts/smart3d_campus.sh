#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TMP_DIR="${TMPDIR:-/tmp}/smart3d-campus.$$"
DEPLOY_DIR="${SMART3D_DEPLOY_DIR:-$HOME/Documents/private-repos/SMART3D-MultiAgent/deploy}"

I9_HOST="${I9_HOST:-felix@10.66.112.64}"
I9_KEY="${I9_KEY:-$HOME/.ssh/rpi5_robot}"
I9_SSH_PROXY_COMMAND="${I9_SSH_PROXY_COMMAND:-}"
PI_HOST="${PI_HOST:-robotcar@10.66.38.108}"
PI_PASS="${PI_PASS:-234123}"
ROBOT_SPEED="${ROBOT_SPEED:-0.24}"

I9_IP="${I9_IP:-10.66.112.64}"
PI_IP="${PI_IP:-10.66.38.108}"

I9_PLANNER_XML_REMOTE="/tmp/cyclone-campus-planner.xml"
I9_RVIZ_XML_REMOTE="/tmp/cyclone-campus-rviz.xml"
I9_VALIDATE_XML_REMOTE="/tmp/cyclone-campus-validate.xml"
I9_WATCH_XML_REMOTE="/tmp/cyclone-campus-watch.xml"
PI_XML_HOST_REMOTE="/tmp/cyclone-campus-pi.xml"
PI_XML_CONTAINER_REMOTE="/tmp/cyclone-campus-pi.xml"
I9_PLANNER_LOCAL="$DEPLOY_DIR/i9_planner.py"
I9_PLANNER_REMOTE="/home/felix/SMART3D/i9_planner.py"
PI5_CLIENT_LOCAL="$DEPLOY_DIR/pi5_client.py"
PI5_CLIENT_HOST_REMOTE="/tmp/pi5_client.py"
PI5_CLIENT_CONTAINER_REMOTE="/root/pi5_client.py"
if [ -f "$ROOT_DIR/../rviz/smart_real_robot.rviz" ]; then
  RVIZ_CONFIG_LOCAL="$ROOT_DIR/../rviz/smart_real_robot.rviz"
else
  RVIZ_CONFIG_LOCAL="$ROOT_DIR/smart_real_robot.rviz"
fi
if [ -f "$ROOT_DIR/../pi/campus_map_bridge.py" ]; then
  PI_MAP_BRIDGE_LOCAL="$ROOT_DIR/../pi/campus_map_bridge.py"
else
  PI_MAP_BRIDGE_LOCAL="$ROOT_DIR/campus_map_bridge.py"
fi
if [ -f "$ROOT_DIR/../pi/pi_ros_watchdog.py" ]; then
  PI_ROS_WATCHDOG_LOCAL="$ROOT_DIR/../pi/pi_ros_watchdog.py"
else
  PI_ROS_WATCHDOG_LOCAL="$ROOT_DIR/pi_ros_watchdog.py"
fi
if [ -f "$ROOT_DIR/../pi/pi_host_watchdog.sh" ]; then
  PI_HOST_WATCHDOG_LOCAL="$ROOT_DIR/../pi/pi_host_watchdog.sh"
else
  PI_HOST_WATCHDOG_LOCAL="$ROOT_DIR/pi_host_watchdog.sh"
fi
if [ -f "$ROOT_DIR/../pi/mk_m10p_dev.sh" ]; then
  PI_M10P_HELPER_LOCAL="$ROOT_DIR/../pi/mk_m10p_dev.sh"
else
  PI_M10P_HELPER_LOCAL="$ROOT_DIR/mk_m10p_dev.sh"
fi
if [ -f "$ROOT_DIR/../pi/mk_ddsm_dev.sh" ]; then
  PI_DDSM_HELPER_LOCAL="$ROOT_DIR/../pi/mk_ddsm_dev.sh"
else
  PI_DDSM_HELPER_LOCAL="$ROOT_DIR/mk_ddsm_dev.sh"
fi
if [ -f "$ROOT_DIR/../i9/i9_ros_watchdog.py" ]; then
  I9_ROS_WATCHDOG_LOCAL="$ROOT_DIR/../i9/i9_ros_watchdog.py"
else
  I9_ROS_WATCHDOG_LOCAL="$ROOT_DIR/i9_ros_watchdog.py"
fi
if [ -f "$ROOT_DIR/../i9/i9_host_watchdog.sh" ]; then
  I9_HOST_WATCHDOG_LOCAL="$ROOT_DIR/../i9/i9_host_watchdog.sh"
else
  I9_HOST_WATCHDOG_LOCAL="$ROOT_DIR/i9_host_watchdog.sh"
fi
I9_RVIZ_CONFIG_REMOTE="/home/felix/smart_rviz_bridge/smart_real_robot.rviz"
PI_MAP_BRIDGE_HOST_REMOTE="/tmp/campus_map_bridge.py"
PI_MAP_BRIDGE_CONTAINER_REMOTE="/root/campus_map_bridge.py"
PI_ROS_WATCHDOG_HOST_REMOTE="/tmp/pi_ros_watchdog.py"
PI_ROS_WATCHDOG_CONTAINER_REMOTE="/root/pi_ros_watchdog.py"
PI_HOST_WATCHDOG_REMOTE="/tmp/pi_host_watchdog.sh"
PI_M10P_HELPER_HOST_REMOTE="/tmp/mk_m10p_dev.sh"
PI_M10P_HELPER_CONTAINER_REMOTE="/root/mk_m10p_dev.sh"
PI_DDSM_HELPER_HOST_REMOTE="/tmp/mk_ddsm_dev.sh"
PI_DDSM_HELPER_CONTAINER_REMOTE="/root/mk_ddsm_dev.sh"
I9_ROS_WATCHDOG_REMOTE="/tmp/i9_ros_watchdog.py"
I9_HOST_WATCHDOG_REMOTE="/tmp/i9_host_watchdog.sh"

log() {
  printf '[smart3d-campus] %s\n' "$*"
}

retry() {
  local attempts="$1"
  local delay="$2"
  shift 2

  local try rc
  for try in $(seq 1 "$attempts"); do
    if "$@"; then
      return 0
    fi
    rc=$?
    if [ "$try" -lt "$attempts" ]; then
      sleep "$delay"
    fi
  done
  return "${rc:-1}"
}

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    printf 'missing command: %s\n' "$1" >&2
    exit 1
  }
}

run_i9() {
  local ssh_opts=(-i "$I9_KEY" -o StrictHostKeyChecking=no)
  if [ -n "${I9_SSH_PROXY_COMMAND}" ]; then
    ssh_opts+=(-o "ProxyCommand=${I9_SSH_PROXY_COMMAND}")
  fi
  ssh "${ssh_opts[@]}" "$I9_HOST" "$@"
}

run_i9_detached() {
  local ssh_opts=(-f -i "$I9_KEY" -o StrictHostKeyChecking=no)
  if [ -n "${I9_SSH_PROXY_COMMAND}" ]; then
    ssh_opts+=(-o "ProxyCommand=${I9_SSH_PROXY_COMMAND}")
  fi
  ssh "${ssh_opts[@]}" "$I9_HOST" "$@"
}

run_pi() {
  retry 4 2 sshpass -p "$PI_PASS" ssh \
    -o StrictHostKeyChecking=no \
    -o PreferredAuthentications=password \
    -o PubkeyAuthentication=no \
    -o ConnectTimeout=8 \
    "$PI_HOST" "$@"
}

copy_i9() {
  local scp_opts=(-i "$I9_KEY" -o StrictHostKeyChecking=no)
  if [ -n "${I9_SSH_PROXY_COMMAND}" ]; then
    scp_opts+=(-o "ProxyCommand=${I9_SSH_PROXY_COMMAND}")
  fi
  scp "${scp_opts[@]}" "$1" "$I9_HOST:$2" >/dev/null
}

copy_pi() {
  retry 4 2 sshpass -p "$PI_PASS" scp -O \
    -o StrictHostKeyChecking=no \
    -o PreferredAuthentications=password \
    -o PubkeyAuthentication=no \
    -o ConnectTimeout=8 \
    "$1" "$PI_HOST:$2" >/dev/null
}

cleanup_tmp() {
  rm -rf "$TMP_DIR"
}

make_tmp() {
  rm -rf "$TMP_DIR"
  mkdir -p "$TMP_DIR"
}

detect_i9_ip() {
  run_i9 "bash -lc \"ip -4 -o addr show wlp3s0 | awk '{print \\\$4}' | cut -d/ -f1 | head -n1\""
}

detect_pi_ip() {
  run_pi "bash -lc \"ip -4 -o addr show wlan0 | awk '{print \\\$4}' | cut -d/ -f1 | head -n1\""
}

refresh_peer_ips() {
  local detected_i9 detected_pi

  detected_i9="$(detect_i9_ip | tr -d '\r\n')"
  detected_pi="$(detect_pi_ip | tr -d '\r\n')"

  if [ -z "$detected_i9" ] || [ -z "$detected_pi" ]; then
    printf 'failed to detect campus IPs (i9=%s, pi=%s)\n' "$detected_i9" "$detected_pi" >&2
    exit 1
  fi

  I9_IP="$detected_i9"
  PI_IP="$detected_pi"
  log "detected campus IPs: i9=$I9_IP pi=$PI_IP"
}

write_xmls() {
  make_tmp

  cat >"$TMP_DIR/i9-planner.xml" <<EOF
<CycloneDDS>
  <Domain id="42">
    <General>
      <AllowMulticast>false</AllowMulticast>
      <Interfaces>
        <NetworkInterface name="wlp3s0"/>
      </Interfaces>
    </General>
    <Discovery>
      <ParticipantIndex>31</ParticipantIndex>
      <MaxAutoParticipantIndex>40</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="$I9_IP"/>
        <Peer address="$PI_IP"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

  cat >"$TMP_DIR/i9-rviz.xml" <<EOF
<CycloneDDS>
  <Domain id="42">
    <General>
      <AllowMulticast>false</AllowMulticast>
      <Interfaces>
        <NetworkInterface name="wlp3s0"/>
      </Interfaces>
    </General>
    <Discovery>
      <ParticipantIndex>32</ParticipantIndex>
      <MaxAutoParticipantIndex>40</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="$I9_IP"/>
        <Peer address="$PI_IP"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

  cat >"$TMP_DIR/i9-validate.xml" <<EOF
<CycloneDDS>
  <Domain id="42">
    <General>
      <AllowMulticast>false</AllowMulticast>
      <Interfaces>
        <NetworkInterface name="wlp3s0"/>
      </Interfaces>
    </General>
    <Discovery>
      <ParticipantIndex>33</ParticipantIndex>
      <MaxAutoParticipantIndex>40</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="$I9_IP"/>
        <Peer address="$PI_IP"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

  cat >"$TMP_DIR/i9-watch.xml" <<EOF
<CycloneDDS>
  <Domain id="42">
    <General>
      <AllowMulticast>false</AllowMulticast>
      <Interfaces>
        <NetworkInterface name="wlp3s0"/>
      </Interfaces>
    </General>
    <Discovery>
      <ParticipantIndex>34</ParticipantIndex>
      <MaxAutoParticipantIndex>40</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="$I9_IP"/>
        <Peer address="$PI_IP"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

  cat >"$TMP_DIR/pi.xml" <<EOF
<CycloneDDS>
  <Domain id="42">
    <General>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>40</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="$I9_IP"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

  copy_i9 "$TMP_DIR/i9-planner.xml" "$I9_PLANNER_XML_REMOTE"
  copy_i9 "$TMP_DIR/i9-rviz.xml" "$I9_RVIZ_XML_REMOTE"
  copy_i9 "$TMP_DIR/i9-validate.xml" "$I9_VALIDATE_XML_REMOTE"
  copy_i9 "$TMP_DIR/i9-watch.xml" "$I9_WATCH_XML_REMOTE"
  copy_i9 "$I9_PLANNER_LOCAL" "$I9_PLANNER_REMOTE"
  copy_i9 "$RVIZ_CONFIG_LOCAL" "$I9_RVIZ_CONFIG_REMOTE"
  copy_i9 "$I9_ROS_WATCHDOG_LOCAL" "$I9_ROS_WATCHDOG_REMOTE"
  copy_i9 "$I9_HOST_WATCHDOG_LOCAL" "$I9_HOST_WATCHDOG_REMOTE"
  copy_pi "$PI5_CLIENT_LOCAL" "$PI5_CLIENT_HOST_REMOTE"
  copy_pi "$PI_MAP_BRIDGE_LOCAL" "$PI_MAP_BRIDGE_HOST_REMOTE"
  copy_pi "$PI_ROS_WATCHDOG_LOCAL" "$PI_ROS_WATCHDOG_HOST_REMOTE"
  copy_pi "$PI_HOST_WATCHDOG_LOCAL" "$PI_HOST_WATCHDOG_REMOTE"
  copy_pi "$PI_M10P_HELPER_LOCAL" "$PI_M10P_HELPER_HOST_REMOTE"
  copy_pi "$PI_DDSM_HELPER_LOCAL" "$PI_DDSM_HELPER_HOST_REMOTE"
  copy_pi "$TMP_DIR/pi.xml" "$PI_XML_HOST_REMOTE"
}

start_watchdogs() {
  log "starting i9 + Pi watchdogs"
  run_i9 "bash -lc 'chmod +x $I9_HOST_WATCHDOG_REMOTE $I9_ROS_WATCHDOG_REMOTE; pkill -f \"bash $I9_HOST_WATCHDOG_REMOTE$\" 2>/dev/null || true; nohup bash $I9_HOST_WATCHDOG_REMOTE >/tmp/i9_host_watchdog.stdout 2>&1 </dev/null &'"
  sleep 1
  run_i9 "bash -lc 'pkill -f \"python3 $I9_ROS_WATCHDOG_REMOTE$\" 2>/dev/null || true; source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://$I9_WATCH_XML_REMOTE; setsid -f python3 $I9_ROS_WATCHDOG_REMOTE >/tmp/i9_ros_watchdog.log 2>&1 < /dev/null'"
  run_pi "chmod +x $PI_HOST_WATCHDOG_REMOTE $PI_ROS_WATCHDOG_HOST_REMOTE"
  run_pi "pkill -f '[p]i_host_watchdog.sh' 2>/dev/null || true"
  run_pi "docker exec ros2-humble bash -lc 'pkill -f \"[p]i_ros_watchdog.py\" 2>/dev/null || true'"
  run_pi "docker cp $PI_ROS_WATCHDOG_HOST_REMOTE ros2-humble:$PI_ROS_WATCHDOG_CONTAINER_REMOTE >/dev/null"
  run_pi "nohup bash $PI_HOST_WATCHDOG_REMOTE >/tmp/pi_host_watchdog.stdout 2>&1 </dev/null &"
  run_pi "docker exec -d ros2-humble bash -lc 'export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://$PI_XML_CONTAINER_REMOTE; source /opt/ros/humble/setup.bash; python3 $PI_ROS_WATCHDOG_CONTAINER_REMOTE >/tmp/pi_ros_watchdog.log 2>&1'"
  sleep 2
}

watch_status() {
  log "i9 watchdog status"
  run_i9 "bash -lc 'echo ====I9_HOST_PROCS====; ps -ef | grep -E \"[i]9_host_watchdog.sh\" || true; echo ====I9_HOST_EVENTS====; grep -E \"EVENT|HEARTBEAT|i9_host_watchdog start\" /tmp/i9_host_watchdog.log 2>/dev/null | tail -n 20 || true; echo ====I9_HOST_STDOUT====; tail -n 20 /tmp/i9_host_watchdog.stdout 2>/dev/null || true; echo ====I9_ROS_PROCS====; ps -ef | grep -E \"[i]9_ros_watchdog.py\" || true; echo ====I9_ROS_EVENTS====; grep -E \"EVENT|HEARTBEAT|i9 ROS watchdog started\" /tmp/i9_ros_watchdog.log 2>/dev/null | tail -n 30 || true'"
  log "Pi watchdog status"
  run_pi "bash -lc 'echo ====HOST_PROCS====; ps -ef | grep -E \"[p]i_host_watchdog.sh\" || true; echo ====HOST_EVENTS====; grep -E \"EVENT|HEARTBEAT|pi_host_watchdog start\" /tmp/pi_host_watchdog.log 2>/dev/null | tail -n 20 || true; echo ====HOST_STDOUT====; tail -n 20 /tmp/pi_host_watchdog.stdout 2>/dev/null || true'"
  run_pi "docker exec ros2-humble bash -lc 'echo ====ROS_WATCHDOG_PROCS====; ps -ef | grep -E \"[p]i_ros_watchdog.py\" || true; echo ====ROS_WATCHDOG_EVENTS====; grep -E \"EVENT|HEARTBEAT|Pi ROS watchdog started\" /tmp/pi_ros_watchdog.log 2>/dev/null | tail -n 30 || true'"
}

prepare() {
  need_cmd ssh
  need_cmd scp
  need_cmd sshpass
  refresh_peer_ips
  write_xmls

  log "prepared remote XML files"
  run_i9 "sed -n '1,80p' $I9_VALIDATE_XML_REMOTE; printf '\n'; sed -n '1,80p' $I9_WATCH_XML_REMOTE"
  run_pi "sed -n '1,80p' $PI_XML_HOST_REMOTE"
}

start_pi_passive() {
  log "restarting Pi container and starting passive campus stack"
  run_pi "docker restart ros2-humble >/dev/null"
  sleep 6
  run_pi "docker cp $PI_XML_HOST_REMOTE ros2-humble:$PI_XML_CONTAINER_REMOTE >/dev/null"
  run_pi "docker cp $PI5_CLIENT_HOST_REMOTE ros2-humble:$PI5_CLIENT_CONTAINER_REMOTE >/dev/null"
  run_pi "docker cp $PI_MAP_BRIDGE_HOST_REMOTE ros2-humble:$PI_MAP_BRIDGE_CONTAINER_REMOTE >/dev/null"
  run_pi "docker cp $PI_M10P_HELPER_HOST_REMOTE ros2-humble:$PI_M10P_HELPER_CONTAINER_REMOTE >/dev/null"
  run_pi "docker cp $PI_DDSM_HELPER_HOST_REMOTE ros2-humble:$PI_DDSM_HELPER_CONTAINER_REMOTE >/dev/null"
  run_pi "docker exec ros2-humble bash -lc 'chmod +x $PI_M10P_HELPER_CONTAINER_REMOTE'"
  run_pi "docker exec ros2-humble bash -lc 'chmod +x $PI_DDSM_HELPER_CONTAINER_REMOTE'"

  run_pi "docker exec ros2-humble bash -lc 'pkill -f \"[s]tart_full.sh\" 2>/dev/null || true'"
  run_pi "docker exec ros2-humble bash -lc 'pkill -f \"[c]ampus_map_bridge.py\" 2>/dev/null || true'"
  run_pi "docker exec ros2-humble bash -lc '$PI_M10P_HELPER_CONTAINER_REMOTE >/tmp/mk_m10p_dev.log 2>&1 || true; /root/mk_imu_dev.sh >/tmp/mk_imu_dev.log 2>&1 || true; $PI_DDSM_HELPER_CONTAINER_REMOTE >/tmp/mk_ddsm_dev.log 2>&1 || true'"
  run_pi "docker exec -d ros2-humble bash -lc 'export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://$PI_XML_CONTAINER_REMOTE; /root/start_full.sh >/tmp/start_full_campus.log 2>&1'"
  sleep 18
  run_pi "docker exec -d ros2-humble bash -lc 'export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://$PI_XML_CONTAINER_REMOTE; source /opt/ros/humble/setup.bash; python3 $PI_MAP_BRIDGE_CONTAINER_REMOTE >/tmp/campus_map_bridge.log 2>&1'"
  sleep 3
  run_pi "docker exec -d ros2-humble bash -lc 'export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://$PI_XML_CONTAINER_REMOTE; source /opt/ros/humble/setup.bash; urdf=\$(cat /root/robotcar_description/robotcar.urdf); ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=\"\$urdf\" >/tmp/robot_state_publisher.log 2>&1'"
  sleep 2
  run_pi "docker exec ros2-humble bash -lc 'pkill -f \"[p]i5_client.py\" 2>/dev/null || true'"
  run_pi "docker exec -d ros2-humble bash -lc 'export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://$PI_XML_CONTAINER_REMOTE; source /opt/ros/humble/setup.bash; python3 /root/pi5_client.py --ros-args -p car_ns:=car1 -p robot_speed:=$ROBOT_SPEED -p campus_mode:=true >/tmp/pi5_client_campus.log 2>&1'"
  sleep 6

  run_pi "docker exec ros2-humble bash -lc 'export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://$PI_XML_CONTAINER_REMOTE; export ROS2CLI_NO_DAEMON=1; source /opt/ros/humble/setup.bash; ok=0; for n in \$(seq 1 20); do topics=\$(ros2 topic list 2>/dev/null || true); if printf \"%s\n\" \"\$topics\" | grep -qx /scan && printf \"%s\n\" \"\$topics\" | grep -qx /map_campus && printf \"%s\n\" \"\$topics\" | grep -qx /car1/pose; then if timeout 5 ros2 topic echo /scan --once --qos-reliability best_effort >/tmp/scan_probe.log 2>&1 && timeout 5 ros2 topic echo /map_campus --once --qos-durability transient_local --qos-reliability best_effort >/tmp/map_probe.log 2>&1 && timeout 5 ros2 topic echo /car1/pose --once --qos-reliability best_effort >/tmp/pose_probe.log 2>&1; then ok=1; break; fi; fi; sleep 1; done; if [ \"\$ok\" = 1 ]; then exit 0; fi; echo ====START_FULL_LOG====; tail -n 80 /tmp/start_full_campus.log 2>/dev/null || true; echo ====LIDAR_LOG====; tail -n 120 /tmp/lidar.log 2>/dev/null || true; echo ====MAP_BRIDGE_LOG====; tail -n 80 /tmp/campus_map_bridge.log 2>/dev/null || true; echo ====PI5_CLIENT_LOG====; tail -n 80 /tmp/pi5_client_campus.log 2>/dev/null || true; echo ====SCAN_PROBE====; tail -n 40 /tmp/scan_probe.log 2>/dev/null || true; echo ====MAP_PROBE====; tail -n 40 /tmp/map_probe.log 2>/dev/null || true; echo ====POSE_PROBE====; tail -n 40 /tmp/pose_probe.log 2>/dev/null || true; exit 1'"
  start_watchdogs

  log "Pi detached processes"
  run_pi "docker exec ros2-humble bash -lc \"ps -ef | grep -E '[s]tart_full.sh|[p]i5_client.py|[c]ampus_map_bridge.py|[c]artographer_node|[l]slidar_driver_node|[i]mu_qos_bridge.py|[d]dsm115_odom.py|[p]i_ros_watchdog.py' | grep -v grep || true\""
}

validate_passive() {
  log "validating campus data-plane from i9"
  run_i9 "bash -lc 'set -eo pipefail; source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://$I9_VALIDATE_XML_REMOTE; export ROS2CLI_NO_DAEMON=1; echo ====TOPICS====; ros2 topic list 2>/dev/null | grep -E \"^/scan$|^/map_campus$|^/car1/pose$\"; echo ====SCAN====; timeout 10 ros2 topic echo /scan --once --qos-reliability best_effort | sed -n \"1,20p\"; echo ====MAP_CAMPUS====; timeout 10 ros2 topic echo /map_campus --once --qos-durability transient_local --qos-reliability best_effort | sed -n \"1,20p\"; echo ====POSE====; timeout 10 ros2 topic echo /car1/pose --once --qos-reliability best_effort | sed -n \"1,20p\"'"
}

start_active() {
  log "starting cmd_vel driver on Pi"
  run_pi "docker exec ros2-humble bash -lc '$PI_DDSM_HELPER_CONTAINER_REMOTE >/tmp/mk_ddsm_dev.log 2>&1 || true; ls -l /dev/ttyUSB0 /dev/ttyUSB1 2>/dev/null || true'"
  run_pi "docker exec ros2-humble bash -lc 'pkill -f \"[c]md_vel_driver.py\" 2>/dev/null || true'"
  run_pi "docker exec -d ros2-humble bash -lc 'export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://$PI_XML_CONTAINER_REMOTE; source /opt/ros/humble/setup.bash; python3 /root/cmd_vel_driver.py >/tmp/cmd_vel_driver_campus.log 2>&1'"
  sleep 4

  log "starting i9 planner"
  run_i9 "pkill -f '[i]9_planner.py' 2>/dev/null || true"
  run_i9 "bash -lc 'cd ~/SMART3D; source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://$I9_PLANNER_XML_REMOTE; setsid -f python3 i9_planner.py --ros-args -p car_names:=car1 -p robot_speed:=$ROBOT_SPEED -p campus_mode:=true -p map_topic:=/map_campus >/tmp/i9_planner_campus.log 2>&1 < /dev/null'"
  sleep 5

  log "starting RViz on i9 display :1"
  run_i9 "pkill -f '[r]viz2 -d .*smart_real_robot.rviz' 2>/dev/null || true"
  run_i9 "bash -lc 'XAUTH=/run/user/1000/gdm/Xauthority; [ -f \"\$XAUTH\" ] || XAUTH=/home/felix/.Xauthority; source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://$I9_RVIZ_XML_REMOTE; export DISPLAY=:1; export XAUTHORITY=\$XAUTH; setsid -f rviz2 -d $I9_RVIZ_CONFIG_REMOTE >/tmp/rviz_campus.log 2>&1 < /dev/null'"
  sleep 5

  log "active processes"
  run_pi "docker exec ros2-humble bash -lc \"ps -ef | grep -E '[c]md_vel_driver.py' | grep -v grep || true; tail -n 10 /tmp/cmd_vel_driver_campus.log 2>/dev/null || true\""
  run_i9 "ps -ef | grep -E '[i]9_planner.py|[r]viz2' | grep -v grep || true"
}

move_test() {
  log "sending tiny /cmd_vel pulse and sampling odom"
  run_pi "docker exec ros2-humble bash -lc 'export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://$PI_XML_CONTAINER_REMOTE; source /opt/ros/humble/setup.bash; timeout 1 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\" >/tmp/cmdvel_pulse.log 2>&1 || true; ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\" >/tmp/cmdvel_zero.log 2>&1 || true; tail -n 10 /tmp/odom.log 2>/dev/null || true'"
}

goal_test() {
  local goal_x="${1:-0.37}"
  local goal_y="${2:--0.16}"

  log "arming Pi-side /car1/path capture"
  run_pi "docker exec -d ros2-humble bash -lc 'rm -f /tmp/goal_test_path.log; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://$PI_XML_CONTAINER_REMOTE; export ROS2CLI_NO_DAEMON=1; source /opt/ros/humble/setup.bash; timeout 20 ros2 topic echo /car1/path --once --qos-durability transient_local --qos-reliability best_effort >/tmp/goal_test_path.log 2>&1'"
  sleep 2

  log "publishing /goal_pose = (${goal_x}, ${goal_y}) from i9"
  run_i9 "bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://$I9_VALIDATE_XML_REMOTE; export ROS2CLI_NO_DAEMON=1; ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \"{header: {frame_id: map}, pose: {position: {x: ${goal_x}, y: ${goal_y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}\"'"

  run_pi "docker exec ros2-humble bash -lc 'for n in \$(seq 1 20); do [ -s /tmp/goal_test_path.log ] && exit 0; sleep 1; done; exit 1'"

  log "Pi saw /car1/path"
  run_pi "docker exec ros2-humble bash -lc 'sed -n \"1,80p\" /tmp/goal_test_path.log; printf \"\\n====PI5_CLIENT====\\n\"; tail -n 20 /tmp/pi5_client_campus.log 2>/dev/null || true'"
  log "planner tail"
  run_i9 "tail -n 20 /tmp/i9_planner_campus.log"
}

stop_all() {
  log "stopping campus stack"
  run_i9 "pkill -f '[i]9_host_watchdog.sh' 2>/dev/null || true; pkill -f '[i]9_ros_watchdog.py' 2>/dev/null || true; pkill -f '[i]9_planner.py' 2>/dev/null || true; pkill -f '[r]viz2 -d .*smart_real_robot.rviz' 2>/dev/null || true"
  run_pi "pkill -f '[p]i_host_watchdog.sh' 2>/dev/null || true"
  run_pi "docker exec ros2-humble bash -lc 'pkill -f \"[p]i_ros_watchdog.py\" 2>/dev/null || true; pkill -f \"[p]i5_client.py\" 2>/dev/null || true; pkill -f \"[c]md_vel_driver.py\" 2>/dev/null || true; pkill -f \"[c]artographer_node\" 2>/dev/null || true; pkill -f \"[l]slidar_driver_node\" 2>/dev/null || true; pkill -f \"[t]alker\" 2>/dev/null || true; pkill -f \"[i]mu_qos_bridge.py\" 2>/dev/null || true; pkill -f \"[d]dsm115_odom.py\" 2>/dev/null || true; pkill -f \"[s]tatic_transform_publisher\" 2>/dev/null || true'"
}

status() {
  log "i9"
  run_i9 "bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://$I9_VALIDATE_XML_REMOTE; export ROS2CLI_NO_DAEMON=1; echo ====TOPICS====; ros2 topic list 2>/dev/null || true; echo ====NODES====; ros2 node list 2>/dev/null || true; echo ====PROCS====; ps -ef | grep -E \"[i]9_planner.py|[r]viz2|[i]9_host_watchdog.sh|[i]9_ros_watchdog.py\" | grep -v grep || true; echo ====I9_WATCHDOG====; tail -n 10 /tmp/i9_host_watchdog.log 2>/dev/null || true; tail -n 10 /tmp/i9_ros_watchdog.log 2>/dev/null || true; echo ====XMLS====; ls $I9_PLANNER_XML_REMOTE $I9_RVIZ_XML_REMOTE $I9_VALIDATE_XML_REMOTE $I9_WATCH_XML_REMOTE 2>/dev/null || true'"
  log "Pi"
  run_pi "bash -lc 'echo ====HOST_WATCHDOG====; ps -ef | grep -E \"[p]i_host_watchdog.sh\" || true; tail -n 10 /tmp/pi_host_watchdog.log 2>/dev/null || true'"
  run_pi "docker exec ros2-humble bash -lc \"export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://$PI_XML_CONTAINER_REMOTE; export ROS2CLI_NO_DAEMON=1; source /opt/ros/humble/setup.bash; echo ====TOPICS====; ros2 topic list 2>/dev/null || true; echo ====NODES====; ros2 node list 2>/dev/null || true; echo ====PROCS====; ps -ef | grep -E '[s]tart_full.sh|[p]i5_client.py|[c]md_vel_driver.py|[c]artographer_node|[l]slidar_driver_node|[i]mu_qos_bridge.py|[d]dsm115_odom.py|[p]i_ros_watchdog.py' | grep -v grep || true; echo ====ROS_WATCHDOG====; tail -n 10 /tmp/pi_ros_watchdog.log 2>/dev/null || true\""
}

usage() {
  cat <<EOF
Usage: $(basename "$0") <command>

Optional environment overrides:
  I9_HOST=felix@<current_i9_campus_ip>
  I9_SSH_PROXY_COMMAND='/opt/homebrew/bin/cloudflared access ssh --hostname %h'
  PI_HOST=robotcar@<current_pi_campus_ip>

Commands:
  prepare         Write campus XMLs to i9 and Pi
  start-passive   Restart Pi container and start detached sensors + pi5_client
  validate        Passive i9 validation of /scan /tf /map /car1/pose
  start-active    Start cmd_vel_driver on Pi and planner + RViz on i9
  move-test       Send a tiny /cmd_vel pulse through the live chain
  goal-test [x y] Publish /goal_pose on i9 and prove Pi received /car1/path
  watch-start     Start i9 + Pi host/ROS watchdogs without restarting stack
  watch-status    Show i9 + Pi watchdog processes and recent watchdog logs
  full-active     prepare + start-passive + validate + start-active + move-test
  stop            Stop planner/RViz and Pi client/driver/sensors
  status          Show relevant running processes
EOF
}

main() {
  trap cleanup_tmp EXIT

  case "${1:-}" in
    prepare)
      prepare
      ;;
    start-passive)
      prepare
      start_pi_passive
      ;;
    validate)
      validate_passive
      ;;
    start-active)
      start_active
      ;;
    move-test)
      move_test
      ;;
    goal-test)
      goal_test "${2:-}" "${3:-}"
      ;;
    watch-start)
      prepare
      start_watchdogs
      ;;
    watch-status)
      watch_status
      ;;
    full-active)
      prepare
      start_pi_passive
      validate_passive
      start_active
      move_test
      ;;
    stop)
      stop_all
      ;;
    status)
      status
      ;;
    *)
      usage
      exit 1
      ;;
  esac
}

main "$@"
