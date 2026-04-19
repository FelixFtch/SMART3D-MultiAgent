#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SMART3D_ROOT="$(cd "${ROOT_DIR}/.." && pwd)"
TMP_DIR="${TMPDIR:-/tmp}/smart3d-jetson-campus.$$"

SMART3D_DEPLOY_DIR="${SMART3D_DEPLOY_DIR:-$HOME/Documents/private-repos/SMART3D-MultiAgent/deploy}"

I9_HOST="${I9_HOST:-felix@10.66.112.64}"
I9_KEY="${I9_KEY:-$HOME/.ssh/rpi5_robot}"
I9_SSH_PROXY_COMMAND="${I9_SSH_PROXY_COMMAND:-}"
JETSON_HOST="${JETSON_HOST:-jetson@10.66.121.109}"
JETSON_SSH_PROXY_COMMAND="${JETSON_SSH_PROXY_COMMAND:-}"

I9_IP="${I9_IP:-10.66.112.64}"
JETSON_IP="${JETSON_IP:-10.66.121.109}"

ROBOT_SPEED="${ROBOT_SPEED:-0.24}"
PI5_EMERGENCY_STOP_DIST="${PI5_EMERGENCY_STOP_DIST:-0.05}"
I9_MAP_CHANGE_INTERVAL_SEC="${I9_MAP_CHANGE_INTERVAL_SEC:-0.8}"
I9_MAP_CHANGE_THRESHOLD="${I9_MAP_CHANGE_THRESHOLD:-0.03}"
I9_MAP_CHANGE_MIN_CELLS="${I9_MAP_CHANGE_MIN_CELLS:-30}"
START_PI5_REFERENCE_WHEEL_ODOM="${START_PI5_REFERENCE_WHEEL_ODOM:-0}"
KEEP_PI5_REFERENCE_WHEEL_ODOM="${KEEP_PI5_REFERENCE_WHEEL_ODOM:-0}"
MOVE_TEST_SPEED="${MOVE_TEST_SPEED:-0.03}"
JETSON_BACKEND="${JETSON_BACKEND:-cartographer}"
JETSON_RUNTIME_PROFILE="${JETSON_RUNTIME_PROFILE:-pi5_reference}"
JETSON_PI5_REFERENCE_CARTO_CONFIG_BASENAME="${JETSON_PI5_REFERENCE_CARTO_CONFIG_BASENAME:-robotcar_2d.lua}"
JETSON_BRIO_DEVICE="${JETSON_BRIO_DEVICE:-/dev/v4l/by-id/usb-046d_Logitech_BRIO_6A53C801-video-index0}"
JETSON_STP23L_DEVICE="${JETSON_STP23L_DEVICE:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6C069316-if00}"
CMD_VEL_DRIVER_WHEEL_TF="${CMD_VEL_DRIVER_WHEEL_TF:-0}"
CMD_VEL_DRIVER_ODOM_TOPIC="${CMD_VEL_DRIVER_ODOM_TOPIC:-/odom}"
CMD_VEL_DRIVER_ODOM_FRAME="${CMD_VEL_DRIVER_ODOM_FRAME:-odom}"
CMD_VEL_DRIVER_BASE_FRAME="${CMD_VEL_DRIVER_BASE_FRAME:-base_link}"
CMD_VEL_DRIVER_MAX_RPM="${CMD_VEL_DRIVER_MAX_RPM:-50}"
CMD_VEL_DRIVER_USE_COMMANDED_FALLBACK="${CMD_VEL_DRIVER_USE_COMMANDED_FALLBACK:-false}"
JETSON_ACTIVE_CARTO_CONFIG_BASENAME="${JETSON_ACTIVE_CARTO_CONFIG_BASENAME:-robotcar_2d_odom.lua}"
ACTIVE_START_STP23L_BRAKE_GUARD="${ACTIVE_START_STP23L_BRAKE_GUARD:-0}"

JETSON_CTL="${SMART3D_ROOT}/jetson/jetson_runtime_ctl.sh"
JETSON_COMPARE="${SMART3D_ROOT}/jetson/compare_slam_backends.sh"
JETSON_DEPLOY="${SMART3D_ROOT}/jetson/deploy_runtime_to_jetson.sh"

I9_PLANNER_XML_REMOTE="/tmp/cyclone-campus-planner-jetson.xml"
I9_RVIZ_XML_REMOTE="/tmp/cyclone-campus-rviz-jetson.xml"
I9_VALIDATE_XML_REMOTE="/tmp/cyclone-campus-validate-jetson.xml"
I9_WATCH_XML_REMOTE="/tmp/cyclone-campus-watch-jetson.xml"
I9_PLANNER_LOCAL="${SMART3D_DEPLOY_DIR}/i9_planner.py"
I9_PLANNER_REMOTE="/home/felix/SMART3D/i9_planner.py"
if [ -f "${SMART3D_ROOT}/rviz/smart_real_robot.rviz" ]; then
  RVIZ_CONFIG_LOCAL="${SMART3D_ROOT}/rviz/smart_real_robot.rviz"
else
  RVIZ_CONFIG_LOCAL="${ROOT_DIR}/smart_real_robot.rviz"
fi
I9_RVIZ_CONFIG_REMOTE="/home/felix/smart_rviz_bridge/smart_real_robot.rviz"

log() {
  printf '[smart3d-jetson] %s\n' "$*"
}

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    printf 'missing command: %s\n' "$1" >&2
    exit 1
  }
}

run_i9() {
  local ssh_opts=(-i "${I9_KEY}" -o StrictHostKeyChecking=no)
  if [ -n "${I9_SSH_PROXY_COMMAND}" ]; then
    ssh_opts+=(-o "ProxyCommand=${I9_SSH_PROXY_COMMAND}")
  fi
  ssh "${ssh_opts[@]}" "${I9_HOST}" "$@"
}

copy_i9() {
  local scp_opts=(-i "${I9_KEY}" -o StrictHostKeyChecking=no)
  if [ -n "${I9_SSH_PROXY_COMMAND}" ]; then
    scp_opts+=(-o "ProxyCommand=${I9_SSH_PROXY_COMMAND}")
  fi
  scp "${scp_opts[@]}" "$1" "${I9_HOST}:$2" >/dev/null
}

run_jetson() {
  local ssh_opts=(-o StrictHostKeyChecking=no)
  if [ -n "${JETSON_SSH_PROXY_COMMAND}" ]; then
    ssh_opts+=(-o "ProxyCommand=${JETSON_SSH_PROXY_COMMAND}")
  fi
  ssh "${ssh_opts[@]}" "${JETSON_HOST}" "$@"
}

cleanup_tmp() {
  rm -rf "${TMP_DIR}"
}

make_tmp() {
  rm -rf "${TMP_DIR}"
  mkdir -p "${TMP_DIR}"
}

detect_i9_ip() {
  run_i9 "bash -lc \"ip -4 -o addr show wlp3s0 | awk '{print \\\$4}' | cut -d/ -f1 | head -n1\""
}

detect_jetson_ip() {
  run_jetson "bash -lc \"ip -4 -o addr show wlan0 | awk '{print \\\$4}' | cut -d/ -f1 | head -n1\""
}

refresh_peer_ips() {
  local detected_i9 detected_jetson

  detected_i9="$(detect_i9_ip | tr -d '\r\n')"
  detected_jetson="$(detect_jetson_ip | tr -d '\r\n')"

  if [ -z "${detected_i9}" ] || [ -z "${detected_jetson}" ]; then
    printf 'failed to detect campus IPs (i9=%s, jetson=%s)\n' "${detected_i9}" "${detected_jetson}" >&2
    exit 1
  fi

  I9_IP="${detected_i9}"
  JETSON_IP="${detected_jetson}"
  log "detected campus IPs: i9=${I9_IP} jetson=${JETSON_IP}"
}

write_i9_xmls() {
  make_tmp

  cat >"${TMP_DIR}/i9-planner-jetson.xml" <<EOF
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
        <Peer address="${I9_IP}"/>
        <Peer address="${JETSON_IP}"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

  cat >"${TMP_DIR}/i9-rviz-jetson.xml" <<EOF
<CycloneDDS>
  <Domain id="42">
    <General>
      <AllowMulticast>false</AllowMulticast>
      <Interfaces>
        <NetworkInterface name="wlp3s0"/>
      </Interfaces>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>40</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="${I9_IP}"/>
        <Peer address="${JETSON_IP}"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

  cat >"${TMP_DIR}/i9-validate-jetson.xml" <<EOF
<CycloneDDS>
  <Domain id="42">
    <General>
      <AllowMulticast>false</AllowMulticast>
      <Interfaces>
        <NetworkInterface name="wlp3s0"/>
      </Interfaces>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>40</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="${I9_IP}"/>
        <Peer address="${JETSON_IP}"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

  cat >"${TMP_DIR}/i9-watch-jetson.xml" <<EOF
<CycloneDDS>
  <Domain id="42">
    <General>
      <AllowMulticast>false</AllowMulticast>
      <Interfaces>
        <NetworkInterface name="wlp3s0"/>
      </Interfaces>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>40</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="${I9_IP}"/>
        <Peer address="${JETSON_IP}"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

  copy_i9 "${TMP_DIR}/i9-planner-jetson.xml" "${I9_PLANNER_XML_REMOTE}"
  copy_i9 "${TMP_DIR}/i9-rviz-jetson.xml" "${I9_RVIZ_XML_REMOTE}"
  copy_i9 "${TMP_DIR}/i9-validate-jetson.xml" "${I9_VALIDATE_XML_REMOTE}"
  copy_i9 "${TMP_DIR}/i9-watch-jetson.xml" "${I9_WATCH_XML_REMOTE}"
  copy_i9 "${I9_PLANNER_LOCAL}" "${I9_PLANNER_REMOTE}"
  copy_i9 "${RVIZ_CONFIG_LOCAL}" "${I9_RVIZ_CONFIG_REMOTE}"
}

prepare() {
  need_cmd ssh
  need_cmd scp
  need_cmd python3

  refresh_peer_ips
  write_i9_xmls

  log "deploying latest Jetson runtime"
  JETSON_HOST="${JETSON_HOST}" \
  JETSON_SSH_PROXY_COMMAND="${JETSON_SSH_PROXY_COMMAND}" \
  bash "${JETSON_DEPLOY}"

  log "prepared i9 XML files"
  run_i9 "sed -n '1,80p' ${I9_VALIDATE_XML_REMOTE}; printf '\n'; sed -n '1,80p' ${I9_RVIZ_XML_REMOTE}"
}

start_passive() {
  prepare
  log "stopping stale Jetson runtime first"
  JETSON_HOST="${JETSON_HOST}" \
  JETSON_SSH_PROXY_COMMAND="${JETSON_SSH_PROXY_COMMAND}" \
  bash "${JETSON_CTL}" stop || true
  sleep 3

  if [ "${JETSON_RUNTIME_PROFILE}" = "pi5_reference" ]; then
    log "starting Pi5 reference passive chain on Jetson"
    JETSON_HOST="${JETSON_HOST}" \
    JETSON_SSH_PROXY_COMMAND="${JETSON_SSH_PROXY_COMMAND}" \
    I9_IP="${I9_IP}" \
    START_WATCHDOG="${START_WATCHDOG:-1}" \
    START_PI5_REFERENCE_WHEEL_ODOM="${START_PI5_REFERENCE_WHEEL_ODOM}" \
    SMART3D_CARTO_CONFIG_BASENAME="${JETSON_PI5_REFERENCE_CARTO_CONFIG_BASENAME}" \
    SMART3D_SLAM_SCAN_TOPIC="${SMART3D_SLAM_SCAN_TOPIC:-/scan_slam}" \
    CAR_NS=car1 \
    ROBOT_SPEED="${ROBOT_SPEED}" \
    bash "${JETSON_CTL}" start-pi5-passive
    return 0
  fi

  case "${JETSON_BACKEND}" in
    cartographer)
      log "starting passive Jetson runtime with cartographer (${JETSON_RUNTIME_PROFILE})"
      JETSON_HOST="${JETSON_HOST}" \
      JETSON_SSH_PROXY_COMMAND="${JETSON_SSH_PROXY_COMMAND}" \
      I9_IP="${I9_IP}" \
      START_ODOM="${START_ODOM:-0}" \
      START_ACTIVE=0 \
      START_WATCHDOG="${START_WATCHDOG:-1}" \
      START_STP23L="${START_STP23L:-1}" \
      START_STP23L_BRAKE_GUARD="${START_STP23L_BRAKE_GUARD:-1}" \
      SMART3D_CARTO_CONFIG_BASENAME="${SMART3D_CARTO_CONFIG_BASENAME:-robotcar_2d.lua}" \
      bash "${JETSON_CTL}" start-cartographer
      ;;
    slam_toolbox)
      log "starting passive Jetson runtime with slam_toolbox"
      JETSON_HOST="${JETSON_HOST}" \
      JETSON_SSH_PROXY_COMMAND="${JETSON_SSH_PROXY_COMMAND}" \
      I9_IP="${I9_IP}" \
      START_ODOM="${START_ODOM:-0}" \
      START_ODOM_STATIC_TF="${START_ODOM_STATIC_TF:-1}" \
      START_ODOM_TF_RELAY="${START_ODOM_TF_RELAY:-1}" \
      START_ACTIVE=0 \
      START_WATCHDOG="${START_WATCHDOG:-1}" \
      START_STP23L="${START_STP23L:-1}" \
      START_STP23L_BRAKE_GUARD="${START_STP23L_BRAKE_GUARD:-1}" \
      bash "${JETSON_CTL}" start-slam
      ;;
    *)
      printf 'unsupported backend: %s\n' "${JETSON_BACKEND}" >&2
      exit 1
      ;;
  esac
}

start_sensors() {
  prepare
  log "stopping stale Jetson runtime first"
  JETSON_HOST="${JETSON_HOST}" \
  JETSON_SSH_PROXY_COMMAND="${JETSON_SSH_PROXY_COMMAND}" \
  bash "${JETSON_CTL}" stop || true
  sleep 3

  log "starting Jetson sensors-only runtime"
  JETSON_HOST="${JETSON_HOST}" \
  JETSON_SSH_PROXY_COMMAND="${JETSON_SSH_PROXY_COMMAND}" \
  I9_IP="${I9_IP}" \
  START_ODOM="${START_ODOM:-0}" \
  START_ACTIVE=0 \
  START_WATCHDOG=0 \
  START_ODOM_STATIC_TF=0 \
  START_ODOM_TF_RELAY=0 \
  START_STP23L="${START_STP23L:-1}" \
  START_STP23L_BRAKE_GUARD="${START_STP23L_BRAKE_GUARD:-1}" \
  bash "${JETSON_CTL}" start-sensors
}

validate_passive() {
  log "healthcheck from Jetson side"
  JETSON_HOST="${JETSON_HOST}" \
  JETSON_SSH_PROXY_COMMAND="${JETSON_SSH_PROXY_COMMAND}" \
  I9_IP="${I9_IP}" \
  bash "${JETSON_CTL}" healthcheck

  log "validating Jetson topics from i9"
  if [ "${JETSON_RUNTIME_PROFILE}" = "pi5_reference" ]; then
    run_i9 "bash -lc 'set -eo pipefail; source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://${I9_VALIDATE_XML_REMOTE}; export ROS2CLI_NO_DAEMON=1; echo ====TOPICS====; ros2 topic list 2>/dev/null | grep -E \"^/scan$|^/map_campus$\"; echo ====SCAN====; timeout 10 ros2 topic echo /scan --once --qos-reliability best_effort | sed -n \"1,20p\"; echo ====ODOM_SOFT====; timeout 5 ros2 topic echo /odom --once 2>&1 | sed -n \"1,20p\" || true; echo ====MAP_CAMPUS====; timeout 10 ros2 topic echo /map_campus --once --qos-durability transient_local --qos-reliability best_effort | sed -n \"1,20p\"; echo ====POSE_SOFT====; timeout 5 ros2 topic echo /car1/pose --once --qos-reliability best_effort 2>&1 | sed -n \"1,20p\" || true; echo ====FRONT_RANGE_SOFT====; timeout 5 ros2 topic echo /front_range --once --qos-reliability best_effort 2>&1 | sed -n \"1,20p\" || true; echo ====CAMERA_INFO_SOFT====; timeout 5 ros2 topic echo /camera_info --once 2>&1 | sed -n \"1,20p\" || true'"
  else
    run_i9 "bash -lc 'set -eo pipefail; source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://${I9_VALIDATE_XML_REMOTE}; export ROS2CLI_NO_DAEMON=1; echo ====TOPICS====; ros2 topic list 2>/dev/null | grep -E \"^/scan$|^/odom$|^/map_campus$|^/car1/pose$|^/front_range$\"; echo ====SCAN====; timeout 10 ros2 topic echo /scan --once --qos-reliability best_effort | sed -n \"1,20p\"; echo ====ODOM====; timeout 10 ros2 topic echo /odom --once | sed -n \"1,20p\"; echo ====MAP_CAMPUS====; timeout 10 ros2 topic echo /map_campus --once --qos-durability transient_local --qos-reliability best_effort | sed -n \"1,20p\"; echo ====POSE====; timeout 10 ros2 topic echo /car1/pose --once --qos-reliability best_effort | sed -n \"1,20p\"; echo ====FRONT_RANGE====; timeout 10 ros2 topic echo /front_range --once --qos-reliability best_effort | sed -n \"1,20p\"; echo ====CAMERA_INFO_SOFT====; timeout 5 ros2 topic echo /camera_info --once 2>&1 | sed -n \"1,20p\" || true'"
  fi
}

start_camera() {
  log "starting BRIO camera on Jetson"
  run_jetson "bash -lc 'pkill -f \"[v]4l2_camera\" 2>/dev/null || true'"
  run_jetson "bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///tmp/cyclone-campus-jetson.xml; nohup ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=${JETSON_BRIO_DEVICE} -p output_encoding:=rgb8 >/tmp/v4l2_camera.log 2>&1 < /dev/null &'"
  sleep 4
}

camera_check() {
  start_camera

  log "validating BRIO topics on Jetson"
  run_jetson "bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///tmp/cyclone-campus-jetson.xml; export ROS2CLI_NO_DAEMON=1; echo ====CAMERA_INFO====; timeout 10 ros2 topic echo /camera_info --once | sed -n \"1,20p\"; echo ====IMAGE_RAW====; timeout 10 ros2 topic echo /image_raw --once | sed -n \"1,20p\"; echo ====CAMERA_LOG====; tail -n 20 /tmp/v4l2_camera.log 2>/dev/null || true'"

  log "validating BRIO topics from i9"
  run_i9 "bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://${I9_VALIDATE_XML_REMOTE}; export ROS2CLI_NO_DAEMON=1; echo ====I9_CAMERA_INFO====; timeout 10 ros2 topic echo /camera_info --once | sed -n \"1,20p\"'"
}

start_cmd_vel_driver() {
  log "starting cmd_vel driver on Jetson"
  run_jetson "bash -lc 'cd /home/jetson/SMART3D/jetson_runtime; ./mk_ddsm_dev.sh >/tmp/mk_ddsm_dev.log 2>&1 || true; pkill -f \"[c]md_vel_driver.py\" 2>/dev/null || true; pkill -f \"[d]dsm115_odom.py\" 2>/dev/null || true; pkill -f \"[o]dom_supervisor.sh\" 2>/dev/null || true; pkill -f \"[r]obotcar_wasdqe_test.py\" 2>/dev/null || true'"
  run_jetson "bash -lc 'cd /home/jetson/SMART3D/jetson_runtime; source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///tmp/cyclone-campus-jetson.xml; cmd=(python3 ./cmd_vel_driver.py --ros-args -p wheel_odom_topic:=${CMD_VEL_DRIVER_ODOM_TOPIC} -p wheel_odom_frame_id:=${CMD_VEL_DRIVER_ODOM_FRAME} -p base_frame_id:=${CMD_VEL_DRIVER_BASE_FRAME} -p max_rpm:=${CMD_VEL_DRIVER_MAX_RPM} -p use_commanded_odom_fallback:=${CMD_VEL_DRIVER_USE_COMMANDED_FALLBACK}); if [ \"${CMD_VEL_DRIVER_WHEEL_TF}\" = \"1\" ]; then cmd+=( -p publish_wheel_odom_tf:=true ); fi; nohup \"\${cmd[@]}\" >/tmp/cmd_vel_driver_campus.log 2>&1 < /dev/null &'"
  sleep 4
}

restart_stp23l_range() {
  log "restoring live STP23L range publisher on Jetson"
  run_jetson "bash -lc 'cd /home/jetson/SMART3D/jetson_runtime; ./mk_stp23l_dev.sh >/tmp/mk_stp23l_dev.log 2>&1 || true; pkill -f \"[s]tp23l_range.py\" 2>/dev/null || true'"
  run_jetson "bash -lc 'cd /home/jetson/SMART3D/jetson_runtime; source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///tmp/cyclone-campus-jetson.xml; nohup python3 ./stp23l_range.py --ros-args -p serial_port:=${JETSON_STP23L_DEVICE} -p baud_rate:=230400 -p frame_id:=stp23l_link -p min_range:=0.03 -p max_range:=7.5 -p emergency_stop_dist:=0.25 >/tmp/stp23l.log 2>&1 < /dev/null &'"
  sleep 3
}

start_active() {
  if [ "${JETSON_RUNTIME_PROFILE}" = "pi5_reference" ]; then
    log "starting Pi5 reference active chain on Jetson"
    run_jetson "bash -lc 'cd /home/jetson/SMART3D/jetson_runtime; \
      export CYCLONEDDS_URI=file:///tmp/cyclone-campus-jetson.xml; \
      export ROS_DOMAIN_ID=42; \
      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
      export START_PI5_REFERENCE_WHEEL_ODOM=${START_PI5_REFERENCE_WHEEL_ODOM}; \
      export KEEP_PI5_REFERENCE_WHEEL_ODOM=${KEEP_PI5_REFERENCE_WHEEL_ODOM}; \
      export SMART3D_CARTO_CONFIG_BASENAME=${JETSON_PI5_REFERENCE_CARTO_CONFIG_BASENAME}; \
      export SMART3D_SLAM_SCAN_TOPIC=${SMART3D_SLAM_SCAN_TOPIC:-/scan_slam}; \
      export CAR_NS=car1; \
      export ROBOT_SPEED=${ROBOT_SPEED}; \
      export PI5_EMERGENCY_STOP_DIST=${PI5_EMERGENCY_STOP_DIST}; \
      ./start_pi5_reference_active.sh'"
    sleep 4
  else
    log "restarting Jetson runtime in active single-owner mode (${JETSON_RUNTIME_PROFILE})"
    JETSON_HOST="${JETSON_HOST}" \
    JETSON_SSH_PROXY_COMMAND="${JETSON_SSH_PROXY_COMMAND}" \
    bash "${JETSON_CTL}" stop || true
    sleep 3

    JETSON_HOST="${JETSON_HOST}" \
    JETSON_SSH_PROXY_COMMAND="${JETSON_SSH_PROXY_COMMAND}" \
    I9_IP="${I9_IP}" \
    START_ODOM=1 \
    START_ACTIVE=1 \
    START_WATCHDOG="${START_WATCHDOG:-1}" \
    START_STP23L="${START_STP23L:-1}" \
    START_STP23L_BRAKE_GUARD="${ACTIVE_START_STP23L_BRAKE_GUARD}" \
    SMART3D_CARTO_CONFIG_BASENAME="${JETSON_ACTIVE_CARTO_CONFIG_BASENAME}" \
    CMD_VEL_DRIVER_WHEEL_TF="${CMD_VEL_DRIVER_WHEEL_TF}" \
    CMD_VEL_DRIVER_ODOM_TOPIC="${CMD_VEL_DRIVER_ODOM_TOPIC}" \
    CMD_VEL_DRIVER_ODOM_FRAME="${CMD_VEL_DRIVER_ODOM_FRAME}" \
    CMD_VEL_DRIVER_BASE_FRAME="${CMD_VEL_DRIVER_BASE_FRAME}" \
    CMD_VEL_DRIVER_MAX_RPM="${CMD_VEL_DRIVER_MAX_RPM}" \
    CMD_VEL_DRIVER_USE_COMMANDED_FALLBACK="${CMD_VEL_DRIVER_USE_COMMANDED_FALLBACK}" \
    bash "${JETSON_CTL}" start-cartographer
    sleep 5
  fi

  log "starting i9 planner"
  run_i9 "pkill -f '[i]9_planner.py' 2>/dev/null || true"
  run_i9 "bash -lc 'cd ~/SMART3D; source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://${I9_PLANNER_XML_REMOTE}; setsid -f python3 i9_planner.py --ros-args -p car_names:=car1 -p robot_speed:=${ROBOT_SPEED} -p campus_mode:=true -p map_topic:=/map_campus -p map_change_interval_sec:=${I9_MAP_CHANGE_INTERVAL_SEC} -p map_change_threshold:=${I9_MAP_CHANGE_THRESHOLD} -p map_change_min_cells:=${I9_MAP_CHANGE_MIN_CELLS} >/tmp/i9_planner_campus.log 2>&1 < /dev/null'"
  sleep 5

  log "starting RViz on i9 display :1"
  run_i9 "pkill -f '[r]viz2 -d .*smart_real_robot.rviz' 2>/dev/null || true"
  run_i9 "bash -lc 'XAUTH=/run/user/1000/gdm/Xauthority; [ -f \"\$XAUTH\" ] || XAUTH=/home/felix/.Xauthority; source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://${I9_RVIZ_XML_REMOTE}; export DISPLAY=:1; export XAUTHORITY=\$XAUTH; setsid -f rviz2 -d ${I9_RVIZ_CONFIG_REMOTE} >/tmp/rviz_campus.log 2>&1 < /dev/null'"
  sleep 5

  log "active process snapshot"
  run_jetson "bash -lc 'lsof /dev/ttyUSB0 || true; printf \"\\n====CMD_VEL====\\n\"; tail -n 20 /tmp/cmd_vel_driver_campus.log 2>/dev/null || true'"
  run_i9 "ps -ef | grep -E '[i]9_planner.py|[r]viz2' | grep -v grep || true"
}

move_test() {
  log "sending tiny /cmd_vel pulse on Jetson (${MOVE_TEST_SPEED} m/s)"
  run_jetson "bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///tmp/cyclone-campus-jetson.xml; timeout 1 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: ${MOVE_TEST_SPEED}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\" >/tmp/cmdvel_pulse.log 2>&1 || true; ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\" >/tmp/cmdvel_zero.log 2>&1 || true; printf \"====CMD_VEL_DRIVER====\\n\"; tail -n 20 /tmp/cmd_vel_driver_campus.log 2>/dev/null || true; printf \"\\n====ODOM_LOG====\\n\"; tail -n 20 /tmp/odom.log 2>/dev/null || true; printf \"\\n====ODOM_ONCE====\\n\"; timeout 5 ros2 topic echo /odom --once | sed -n \"1,30p\" || true'"
}

estop_test() {
  log "stopping live STP23L range publisher so fake close-range data is authoritative"
  run_jetson "bash -lc 'pkill -f \"[s]tp23l_range.py\" 2>/dev/null || true; pgrep -af cmd_vel_driver.py || true; lsof /dev/ttyUSB0 || true'"
  sleep 1

  log "publishing fake /front_range=0.04m for 12s to trigger hard-brake"
  run_jetson "bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///tmp/cyclone-campus-jetson.xml; timeout 12 ros2 topic pub -r 5 /front_range sensor_msgs/msg/Range \"{header: {frame_id: stp23l_link}, radiation_type: 1, field_of_view: 0.05, min_range: 0.03, max_range: 7.5, range: 0.04}\" >/tmp/stp23l_fake_low.log 2>&1 || true'"

  log "checking hard-brake activation and RS485 ownership"
  run_jetson "bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///tmp/cyclone-campus-jetson.xml; echo ====BRAKE_ACTIVE====; timeout 5 ros2 topic echo /stp23l_hard_brake_active --once | sed -n \"1,20p\" || true; echo ====GUARD_LOG====; tail -n 40 /tmp/stp23l_hard_brake_guard.log 2>/dev/null || true; echo ====CMD_VEL_PROCS====; pgrep -af cmd_vel_driver.py || true; echo ====TTYUSB0====; lsof /dev/ttyUSB0 || true'"

  log "publishing fake /front_range=0.20m to clear hard-brake"
  run_jetson "bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///tmp/cyclone-campus-jetson.xml; timeout 3 ros2 topic pub -r 5 /front_range sensor_msgs/msg/Range \"{header: {frame_id: stp23l_link}, radiation_type: 1, field_of_view: 0.05, min_range: 0.03, max_range: 7.5, range: 0.20}\" >/tmp/stp23l_fake_clear.log 2>&1 || true; sleep 2; echo ====BRAKE_CLEARED====; timeout 5 ros2 topic echo /stp23l_hard_brake_active --once | sed -n \"1,20p\" || true; echo ====GUARD_LOG====; tail -n 40 /tmp/stp23l_hard_brake_guard.log 2>/dev/null || true; echo ====TTYUSB0====; lsof /dev/ttyUSB0 || true'"

  restart_stp23l_range
  start_cmd_vel_driver

  log "verifying recovery after hard-brake test"
  run_jetson "bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///tmp/cyclone-campus-jetson.xml; echo ====FRONT_RANGE====; timeout 10 ros2 topic echo /front_range --once --qos-reliability best_effort | sed -n \"1,20p\" || true; echo ====CMD_VEL====; tail -n 20 /tmp/cmd_vel_driver_campus.log 2>/dev/null || true; echo ====TTYUSB0====; lsof /dev/ttyUSB0 || true'"
}

goal_test() {
  local goal_x="${1:-0.37}"
  local goal_y="${2:--0.16}"

  log "arming Jetson-side /car1/path capture"
  run_jetson "bash -lc 'rm -f /tmp/goal_test_path.log; nohup bash -lc \"source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///tmp/cyclone-campus-jetson.xml; export ROS2CLI_NO_DAEMON=1; timeout 20 ros2 topic echo /car1/path --once --qos-durability transient_local --qos-reliability best_effort >/tmp/goal_test_path.log 2>&1\" >/tmp/goal_test_arm.log 2>&1 < /dev/null &'"
  sleep 2

  log "publishing /goal_pose = (${goal_x}, ${goal_y}) from i9"
  run_i9 "bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://${I9_VALIDATE_XML_REMOTE}; export ROS2CLI_NO_DAEMON=1; ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \"{header: {frame_id: map}, pose: {position: {x: ${goal_x}, y: ${goal_y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}\"'"

  run_jetson "bash -lc 'for n in \$(seq 1 20); do [ -s /tmp/goal_test_path.log ] && exit 0; sleep 1; done; exit 1'"

  log "Jetson saw /car1/path"
  run_jetson "bash -lc 'sed -n \"1,80p\" /tmp/goal_test_path.log; printf \"\\n====PI5_CLIENT====\\n\"; tail -n 30 /tmp/pi5_client_campus.log 2>/dev/null || true'"
  log "planner tail"
  run_i9 "tail -n 30 /tmp/i9_planner_campus.log"
}

stop_all() {
  log "stopping Jetson + i9 planner/RViz"
  run_i9 "pkill -f '[i]9_planner.py' 2>/dev/null || true; pkill -f '[r]viz2 -d .*smart_real_robot.rviz' 2>/dev/null || true"
  JETSON_HOST="${JETSON_HOST}" \
  JETSON_SSH_PROXY_COMMAND="${JETSON_SSH_PROXY_COMMAND}" \
  bash "${JETSON_CTL}" stop || true
}

status() {
  log "i9 status"
  run_i9 "bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file://${I9_VALIDATE_XML_REMOTE}; export ROS2CLI_NO_DAEMON=1; echo ====TOPICS====; ros2 topic list 2>/dev/null || true; echo ====PROCS====; ps -ef | grep -E \"[i]9_planner.py|[r]viz2|[i]9_ros_watchdog.py|[i]9_host_watchdog.sh\" | grep -v grep || true; echo ====XMLS====; ls ${I9_PLANNER_XML_REMOTE} ${I9_RVIZ_XML_REMOTE} ${I9_VALIDATE_XML_REMOTE} ${I9_WATCH_XML_REMOTE} 2>/dev/null || true'"
  log "Jetson status"
  JETSON_HOST="${JETSON_HOST}" \
  JETSON_SSH_PROXY_COMMAND="${JETSON_SSH_PROXY_COMMAND}" \
  bash "${JETSON_CTL}" status
}

benchmark_backends() {
  local duration_sec="${1:-60}"
  log "running cartographer vs slam_toolbox compare (${duration_sec}s per backend)"
  refresh_peer_ips
  JETSON_HOST="${JETSON_HOST}" \
  JETSON_SSH_PROXY_COMMAND="${JETSON_SSH_PROXY_COMMAND}" \
  I9_IP="${I9_IP}" \
  JETSON_SUDO_PASSWORD="${JETSON_SUDO_PASSWORD:-}" \
  DURATION_SEC="${duration_sec}" \
  REBOOT_BETWEEN_RUNS="${REBOOT_BETWEEN_RUNS:-1}" \
  bash "${JETSON_COMPARE}"
}

usage() {
  cat <<EOF
Usage: $(basename "$0") <command>

Optional environment overrides:
  I9_HOST=felix@<current_i9_campus_ip>
  I9_SSH_PROXY_COMMAND='/opt/homebrew/bin/cloudflared access ssh --hostname %h'
  JETSON_HOST=jetson@<current_jetson_campus_ip>
  JETSON_SSH_PROXY_COMMAND='ssh -i ~/.ssh/rpi5_robot -o StrictHostKeyChecking=no -o UserKnownHostsFile=/tmp/codex_known_hosts_i9 -o ProxyCommand="/opt/homebrew/bin/cloudflared access ssh --hostname alienware-ssh.mofangpi.net" felix@alienware-ssh.mofangpi.net -W %h:%p'
  JETSON_BACKEND=cartographer|slam_toolbox
  JETSON_RUNTIME_PROFILE=pi5_reference|single_owner
  CMD_VEL_DRIVER_WHEEL_TF=0|1
  MOVE_TEST_SPEED=0.03

Commands:
  prepare                  Detect campus IPs, deploy Jetson runtime, write i9 XMLs
  start-passive            Prepare and start Jetson passive chain (default Pi5 reference)
  start-sensors            Prepare and start Jetson sensors-only runtime
  validate                 Validate Jetson topics from both Jetson and i9
  start-active             Start Jetson active chain and planner + RViz on i9
  move-test                Send a tiny /cmd_vel pulse on Jetson
  estop-test               Simulate sustained close obstacle and verify hard-brake + recovery
  goal-test [x y]          Publish /goal_pose on i9 and prove Jetson received /car1/path
  benchmark-backends [s]   Run cartographer vs slam_toolbox compare (default 60s each)
  camera-check             Start BRIO via v4l2_camera and validate /camera_info + /image_raw
  full-active              prepare + start-passive + validate + start-active + move-test
  stop                     Stop planner/RViz and Jetson runtime
  status                   Show relevant running processes
EOF
}

main() {
  trap cleanup_tmp EXIT

  case "${1:-}" in
    prepare)
      prepare
      ;;
    start-passive)
      start_passive
      ;;
    start-sensors)
      start_sensors
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
    estop-test)
      estop_test
      ;;
    goal-test)
      goal_test "${2:-}" "${3:-}"
      ;;
    benchmark-backends)
      benchmark_backends "${2:-60}"
      ;;
    camera-check)
      camera_check
      ;;
    full-active)
      start_passive
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
