#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
CAR_NS="${CAR_NS:-car1}"
ROBOT_SPEED="${ROBOT_SPEED:-0.24}"
START_WATCHDOG="${START_WATCHDOG:-1}"
START_PI5_CLIENT_IN_SUPPORT="${START_PI5_CLIENT_IN_SUPPORT:-0}"
LIDAR_WS="${SMART3D_LIDAR_WS:-$HOME/SMART3D/ws_lidar}"
LIDAR_PARAMS="${SMART3D_LIDAR_PARAMS:-${ROOT_DIR}/m10p_uart.yaml}"
WATCHDOG_MAP_TOPIC="${WATCHDOG_MAP_TOPIC:-/map_campus}"
WATCHDOG_POSE_GAP_WARN_SEC="${WATCHDOG_POSE_GAP_WARN_SEC:-999999.0}"

need_file() {
  local path="$1"
  if [ ! -f "${path}" ]; then
    echo "[pi5-reference-support] missing required file: ${path}" >&2
    exit 1
  fi
}

need_file "/opt/ros/${ROS_DISTRO}/setup.bash"
need_file "${ROOT_DIR}/campus_map_bridge.py"
need_file "${ROOT_DIR}/robotcar.urdf"
need_file "${ROOT_DIR}/pi_ros_watchdog.py"

if [ -n "${CYCLONEDDS_URI:-}" ]; then
  export CYCLONEDDS_URI
fi
export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

pkill -f "[c]ampus_map_bridge.py" 2>/dev/null || true
pkill -f "[r]obot_state_publisher" 2>/dev/null || true
pkill -f "[p]i_ros_watchdog.py" 2>/dev/null || true
if [ "${START_PI5_CLIENT_IN_SUPPORT}" = "1" ]; then
  pkill -f "[p]i5_client.py" 2>/dev/null || true
fi
sleep 1

echo "[support 1/4] Campus map bridge..."
nohup python3 "${ROOT_DIR}/campus_map_bridge.py" >/tmp/campus_map_bridge.log 2>&1 &
sleep 2

echo "[support 2/4] Robot state publisher..."
robot_description="$(tr '\n' ' ' < "${ROOT_DIR}/robotcar.urdf")"
nohup ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="${robot_description}" >/tmp/robot_state_publisher.log 2>&1 &
sleep 2

if [ "${START_PI5_CLIENT_IN_SUPPORT}" = "1" ]; then
  echo "[support 3/4] Pi5 client..."
  nohup python3 "${ROOT_DIR}/pi5_client.py" \
    --ros-args -p car_ns:="${CAR_NS}" -p robot_speed:="${ROBOT_SPEED}" -p campus_mode:=true \
    >/tmp/pi5_client_campus.log 2>&1 &
  sleep 4
else
  echo "[support 3/4] Pi5 client skipped in passive support (Pi5 reference semantics)"
fi

if [ "${START_WATCHDOG}" = "1" ]; then
  echo "[support 4/4] Pi ROS watchdog..."
  nohup env \
    SMART3D_RUNTIME_ROOT="${ROOT_DIR}" \
    SMART3D_LIDAR_WS="${LIDAR_WS}" \
    SMART3D_LIDAR_PARAMS="${LIDAR_PARAMS}" \
    SMART3D_CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///tmp/cyclone-campus-jetson.xml}" \
    python3 "${ROOT_DIR}/pi_ros_watchdog.py" \
      --ros-args \
      -p map_topic:="${WATCHDOG_MAP_TOPIC}" \
      -p pose_gap_warn_sec:="${WATCHDOG_POSE_GAP_WARN_SEC}" \
      >/tmp/pi_ros_watchdog.log 2>&1 &
  sleep 2
else
  echo "[support 4/4] Pi ROS watchdog skipped (START_WATCHDOG=${START_WATCHDOG})"
fi

echo "Pi5 reference campus support started."
