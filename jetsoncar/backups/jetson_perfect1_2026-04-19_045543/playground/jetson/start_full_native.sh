#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIDAR_WS="${SMART3D_LIDAR_WS:-$HOME/SMART3D/ws_lidar}"
IMU_WS="${SMART3D_IMU_WS:-$HOME/SMART3D/ws_imu}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
CAR_NS="${CAR_NS:-car1}"
ROBOT_SPEED="${ROBOT_SPEED:-0.24}"
SLAM_BACKEND="${SMART3D_SLAM_BACKEND:-cartographer}"
CARTO_CONFIG_BASENAME="${SMART3D_CARTO_CONFIG_BASENAME:-robotcar_2d.lua}"
START_ODOM="${START_ODOM:-1}"
START_ACTIVE="${START_ACTIVE:-0}"
START_WATCHDOG="${START_WATCHDOG:-1}"
START_ODOM_TF_RELAY="${START_ODOM_TF_RELAY:-1}"
START_ODOM_STATIC_TF="${START_ODOM_STATIC_TF:-1}"
IMU_PORT="${SMART3D_IMU_PORT:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B030655-if00}"
START_STP23L="${START_STP23L:-1}"
START_STP23L_BRAKE_GUARD="${START_STP23L_BRAKE_GUARD:-1}"
STP23L_PORT="${SMART3D_STP23L_PORT:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6C069316-if00}"
STP23L_TRIGGER_DIST_M="${SMART3D_STP23L_TRIGGER_DIST_M:-0.05}"
STP23L_HOLD_SEC="${SMART3D_STP23L_HOLD_SEC:-10.0}"
CMD_VEL_DRIVER_WHEEL_TF="${CMD_VEL_DRIVER_WHEEL_TF:-0}"
CMD_VEL_DRIVER_ODOM_TOPIC="${CMD_VEL_DRIVER_ODOM_TOPIC:-/odom}"
CMD_VEL_DRIVER_ODOM_FRAME="${CMD_VEL_DRIVER_ODOM_FRAME:-odom}"
CMD_VEL_DRIVER_BASE_FRAME="${CMD_VEL_DRIVER_BASE_FRAME:-base_link}"
CMD_VEL_DRIVER_MAX_RPM="${CMD_VEL_DRIVER_MAX_RPM:-50}"
CMD_VEL_DRIVER_USE_COMMANDED_FALLBACK="${CMD_VEL_DRIVER_USE_COMMANDED_FALLBACK:-false}"
LIDAR_SUPERVISOR="${SMART3D_LIDAR_SUPERVISOR:-${ROOT_DIR}/lidar_supervisor.sh}"
LIDAR_READY_TIMEOUT_SEC="${SMART3D_LIDAR_READY_TIMEOUT_SEC:-20.0}"
SLAM_TOOLBOX_LAUNCH="${SMART3D_SLAM_TOOLBOX_LAUNCH:-${ROOT_DIR}/slam_toolbox_launch.py}"
SLAM_TOOLBOX_PARAMS="${SMART3D_SLAM_TOOLBOX_PARAMS:-${ROOT_DIR}/mapper_params_online_async.yaml}"
SLAM_SCAN_TOPIC="${SMART3D_SLAM_SCAN_TOPIC:-/scan_slam}"
CARTO_USE_SCAN_NORMALIZER="${SMART3D_CARTO_USE_SCAN_NORMALIZER:-1}"

ODOM_REQUESTED="${START_ODOM}"
START_DDSM_ODOM="${START_ODOM}"
ACTIVE_ODOM_VIA_CMD=0
CMD_VEL_DRIVER_STARTED=0

if [ "${START_ACTIVE}" = "1" ]; then
  echo "[config] START_ACTIVE=1 keeps cmd_vel_driver.py as the sole /dev/ttyUSB0 owner."
  START_DDSM_ODOM=0
  START_ODOM_TF_RELAY=0
  if [ "${ODOM_REQUESTED}" = "1" ]; then
    echo "[config] active wheel odom will come from cmd_vel_driver.py real RS485 feedback."
    ACTIVE_ODOM_VIA_CMD=1
  fi
fi

if [ "${START_DDSM_ODOM}" = "1" ] && [ "${START_STP23L_BRAKE_GUARD}" = "1" ]; then
  echo "[config] START_ODOM=1 forces START_STP23L_BRAKE_GUARD=0 so ddsm115_odom.py stays the sole /dev/ttyUSB0 owner."
  START_STP23L_BRAKE_GUARD=0
fi

need_file() {
  local path="$1"
  if [ ! -f "$path" ]; then
    echo "[start_full_native] ERROR: missing required file: $path" >&2
    exit 1
  fi
}

kill_matching() {
  local pattern="$1"
  local pids
  pids="$(pgrep -f "$pattern" 2>/dev/null || true)"
  [ -z "$pids" ] && return 0
  # shellcheck disable=SC2086
  kill -9 $pids 2>/dev/null || true
}

wait_for_scan_ready() {
  local timeout_sec="${1:-20.0}"
  python3 - "${timeout_sec}" <<'PY'
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

timeout_sec = float(sys.argv[1])
seen = {"value": False}

rclpy.init()
node = rclpy.create_node("wait_for_scan_ready")
node.create_subscription(
    LaserScan,
    "/scan",
    lambda _msg: seen.__setitem__("value", True),
    QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
)

deadline = time.time() + timeout_sec
while time.time() < deadline and not seen["value"]:
    rclpy.spin_once(node, timeout_sec=0.2)

node.destroy_node()
rclpy.shutdown()
sys.exit(0 if seen["value"] else 1)
PY
}

start_cmd_vel_driver() {
  [ "${START_ACTIVE}" = "1" ] || return 0
  [ "${CMD_VEL_DRIVER_STARTED}" = "0" ] || return 0

  echo "[active] cmd_vel driver..."
  echo "[active] NOTE: cmd_vel_driver.py is the sole /dev/ttyUSB0 owner during active control."

  local -a cmd=(
    python3 "${ROOT_DIR}/cmd_vel_driver.py" --ros-args
    -p max_rpm:="${CMD_VEL_DRIVER_MAX_RPM}"
    -p base_frame_id:="${CMD_VEL_DRIVER_BASE_FRAME}"
    -p use_commanded_odom_fallback:="${CMD_VEL_DRIVER_USE_COMMANDED_FALLBACK}"
  )

  if [ "${ACTIVE_ODOM_VIA_CMD}" = "1" ]; then
    echo "[active] primary /odom topic: ${CMD_VEL_DRIVER_ODOM_TOPIC} (${CMD_VEL_DRIVER_ODOM_FRAME} -> ${CMD_VEL_DRIVER_BASE_FRAME})"
    cmd+=(
      -p wheel_odom_topic:="${CMD_VEL_DRIVER_ODOM_TOPIC}"
      -p wheel_odom_frame_id:="${CMD_VEL_DRIVER_ODOM_FRAME}"
    )
  fi

  if [ "${CMD_VEL_DRIVER_WHEEL_TF}" = "1" ]; then
    echo "[active] wheel odom TF enabled: ${CMD_VEL_DRIVER_ODOM_FRAME} -> ${CMD_VEL_DRIVER_BASE_FRAME}"
    cmd+=(-p publish_wheel_odom_tf:=true)
  fi

  nohup "${cmd[@]}" >/tmp/cmd_vel_driver_campus.log 2>&1 &
  CMD_VEL_DRIVER_STARTED=1
  sleep 2
}

if [ -n "${CYCLONEDDS_URI:-}" ]; then
  export CYCLONEDDS_URI
fi

export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION
export SMART3D_RUNTIME_ROOT="${ROOT_DIR}"
export SMART3D_LIDAR_WS="${LIDAR_WS}"
export SMART3D_LIDAR_PARAMS="${ROOT_DIR}/m10p_uart.yaml"

need_file "/opt/ros/${ROS_DISTRO}/setup.bash"
need_file "${ROOT_DIR}/mk_imu_dev.sh"
need_file "${LIDAR_SUPERVISOR}"
need_file "${ROOT_DIR}/mk_m10p_dev.sh"
need_file "${ROOT_DIR}/mk_ddsm_dev.sh"
need_file "${ROOT_DIR}/m10p_uart.yaml"
need_file "${ROOT_DIR}/imu_qos_bridge.py"
need_file "${ROOT_DIR}/ddsm115_odom.py"
need_file "${ROOT_DIR}/odom_supervisor.sh"
need_file "${ROOT_DIR}/odom_tf_relay.py"
need_file "${ROOT_DIR}/scan_metadata_normalizer.py"
need_file "${ROOT_DIR}/stp23l_range.py"
need_file "${ROOT_DIR}/stp23l_hard_brake_guard.py"
need_file "${ROOT_DIR}/campus_map_bridge.py"
need_file "${ROOT_DIR}/robotcar.urdf"
need_file "${ROOT_DIR}/pi5_client.py"
need_file "${ROOT_DIR}/pi_ros_watchdog.py"
need_file "${ROOT_DIR}/cmd_vel_driver.py"

case "${SLAM_BACKEND}" in
  cartographer)
    need_file "${ROOT_DIR}/cartographer_launch.py"
    need_file "${ROOT_DIR}/${CARTO_CONFIG_BASENAME}"
    ;;
  slam_toolbox)
    need_file "${SLAM_TOOLBOX_LAUNCH}"
    need_file "${SLAM_TOOLBOX_PARAMS}"
    ;;
  none)
    ;;
  *)
    echo "[start_full_native] ERROR: unsupported SMART3D_SLAM_BACKEND=${SLAM_BACKEND}" >&2
    exit 1
    ;;
esac

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[ -f "${LIDAR_WS}/install/setup.bash" ] && source "${LIDAR_WS}/install/setup.bash"
[ -f "${IMU_WS}/install/setup.bash" ] && source "${IMU_WS}/install/setup.bash"
set -u

pkill -f "[c]artographer_node" 2>/dev/null || true
pkill -f "[o]ccupancy_grid_node" 2>/dev/null || true
pkill -f "[l]idar_supervisor.sh" 2>/dev/null || true
pkill -f "[l]slidar_driver_node" 2>/dev/null || true
pkill -f "[h]ipnuc_imu" 2>/dev/null || true
kill_matching "ros2 run hipnuc_imu talker"
[ -x "${IMU_WS}/install/hipnuc_imu/lib/hipnuc_imu/talker" ] && \
  kill_matching "${IMU_WS}/install/hipnuc_imu/lib/hipnuc_imu/talker"
pkill -f "[i]mu_qos_bridge.py" 2>/dev/null || true
pkill -f "[o]dom_supervisor.sh" 2>/dev/null || true
pkill -f "[d]dsm115_odom.py" 2>/dev/null || true
pkill -f "[s]tp23l_range.py" 2>/dev/null || true
pkill -f "[s]tp23l_hard_brake_guard.py" 2>/dev/null || true
pkill -f "[o]dom_tf_relay.py" 2>/dev/null || true
pkill -f "[s]can_metadata_normalizer.py" 2>/dev/null || true
pkill -f "[s]lam_toolbox" 2>/dev/null || true
pkill -f "[s]tatic_transform_publisher .* base_link laser_link" 2>/dev/null || true
pkill -f "[s]tatic_transform_publisher .* odom base_link" 2>/dev/null || true
pkill -f "[c]ampus_map_bridge.py" 2>/dev/null || true
pkill -f "[r]obot_state_publisher" 2>/dev/null || true
pkill -f "[p]i5_client.py" 2>/dev/null || true
pkill -f "[c]md_vel_driver.py" 2>/dev/null || true
pkill -f "[r]obotcar_wasdqe_test.py" 2>/dev/null || true
pkill -f "[p]i_ros_watchdog.py" 2>/dev/null || true
sleep 2

rm -f /tmp/lidar.log \
      /tmp/imu_talker.log \
      /tmp/imu_bridge.log \
      /tmp/odom.log \
      /tmp/stp23l.log \
      /tmp/stp23l_hard_brake_guard.log \
      /tmp/odom_tf_relay.log \
      /tmp/scan_normalizer.log \
      /tmp/tf_odom_static.log \
      /tmp/cartographer.log \
      /tmp/slam_toolbox.log \
      /tmp/campus_map_bridge.log \
      /tmp/robot_state_publisher.log \
      /tmp/pi5_client_campus.log \
      /tmp/cmd_vel_driver_campus.log \
      /tmp/pi_ros_watchdog.log
rm -rf /dev/shm/fastrtps_* /dev/shm/*fastdds* /dev/shm/*SHM* 2>/dev/null || true

"${ROOT_DIR}/mk_imu_dev.sh" >/tmp/mk_imu_dev.log 2>&1 || true
"${ROOT_DIR}/mk_m10p_dev.sh" >/tmp/mk_m10p_dev.log 2>&1 || true
"${ROOT_DIR}/mk_ddsm_dev.sh" >/tmp/mk_ddsm_dev.log 2>&1 || true
"${ROOT_DIR}/mk_stp23l_dev.sh" >/tmp/mk_stp23l_dev.log 2>&1 || true

echo "[1/12] Static TF..."
nohup ros2 run tf2_ros static_transform_publisher \
  0 0 0.05 0 0 0 base_link laser_link >/tmp/tf_static.log 2>&1 &
sleep 2

echo "[2/12] LiDAR..."
nohup "${LIDAR_SUPERVISOR}" >/tmp/lidar.log 2>&1 &
sleep 3

echo "[3/12] IMU..."
nohup ros2 run hipnuc_imu talker --ros-args \
  -p serial_port:="${IMU_PORT}" \
  -p baud_rate:=115200 \
  -p imu_topic:=/IMU_data \
  -p frame_id:=base_link >/tmp/imu_talker.log 2>&1 &
sleep 2

echo "[4/12] IMU bridge..."
nohup python3 "${ROOT_DIR}/imu_qos_bridge.py" >/tmp/imu_bridge.log 2>&1 &
sleep 2

if [ "${START_STP23L}" = "1" ]; then
  echo "[5/12] STP23L range..."
  nohup python3 "${ROOT_DIR}/stp23l_range.py" --ros-args \
    -p serial_port:="${STP23L_PORT}" \
    -p baud_rate:=230400 \
    -p frame_id:=stp23l_link \
    -p min_range:=0.03 \
    -p max_range:=7.5 \
    -p emergency_stop_dist:=0.25 >/tmp/stp23l.log 2>&1 &
  sleep 2
else
  echo "[5/12] STP23L range skipped (START_STP23L=${START_STP23L})"
fi

if [ "${START_STP23L_BRAKE_GUARD}" = "1" ]; then
  echo "[6/12] STP23L hard-brake guard..."
  nohup python3 "${ROOT_DIR}/stp23l_hard_brake_guard.py" --ros-args \
    -p range_topic:=/front_range \
    -p trigger_distance_m:="${STP23L_TRIGGER_DIST_M}" \
    -p hold_time_sec:="${STP23L_HOLD_SEC}" \
    -p clear_distance_m:=0.08 \
    -p clear_time_sec:=1.0 \
    -p serial_port:=/dev/ttyUSB0 \
    -p baud_rate:=115200 \
    -p publish_zero_cmd_vel:=true \
    -p stop_rs485_owners_on_trigger:=true \
    >/tmp/stp23l_hard_brake_guard.log 2>&1 &
  sleep 2
else
  echo "[6/12] STP23L hard-brake guard skipped (START_STP23L_BRAKE_GUARD=${START_STP23L_BRAKE_GUARD})"
fi

if [ "${START_DDSM_ODOM}" = "1" ]; then
  echo "[7/12] Wheel odom..."
  nohup "${ROOT_DIR}/odom_supervisor.sh" >/tmp/odom.log 2>&1 &
  sleep 2
else
  if [ "${ACTIVE_ODOM_VIA_CMD}" = "1" ]; then
    echo "[7/12] Raw wheel odom skipped (active cmd_vel_driver will publish /odom)"
  else
    echo "[7/12] Wheel odom skipped (START_ODOM=${ODOM_REQUESTED})"
  fi
fi

echo "[scan] Waiting for /scan readiness..."
if ! wait_for_scan_ready "${LIDAR_READY_TIMEOUT_SEC}"; then
  echo "[start_full_native] ERROR: /scan did not become ready within ${LIDAR_READY_TIMEOUT_SEC}s" >&2
  tail -n 80 /tmp/lidar.log 2>/dev/null || true
  exit 1
fi

if [ "${START_ACTIVE}" = "1" ]; then
  echo "[active] Pre-SLAM cmd_vel owner startup..."
  start_cmd_vel_driver
fi

if [ "${SLAM_BACKEND}" = "cartographer" ]; then
  carto_scan_topic="/scan"
  if [ "${CARTO_USE_SCAN_NORMALIZER}" = "1" ]; then
    echo "[scan] Scan normalizer (${SLAM_SCAN_TOPIC})..."
    nohup python3 "${ROOT_DIR}/scan_metadata_normalizer.py" --ros-args \
      -p input_topic:=/scan \
      -p output_topic:="${SLAM_SCAN_TOPIC}" \
      -p adjust_angle_increment:=false \
      -p prefer_stamp_delta_for_scan_time:=true \
      -p derive_time_increment_from_scan_time:=true \
      -p fallback_scan_hz:=12.0 >/tmp/scan_normalizer.log 2>&1 &
    sleep 2
    carto_scan_topic="${SLAM_SCAN_TOPIC}"
  else
    echo "[scan] Cartographer using raw /scan (normalizer disabled)"
  fi
  echo "[8/12] Cartographer (${CARTO_CONFIG_BASENAME})..."
  nohup ros2 launch "${ROOT_DIR}/cartographer_launch.py" \
    config_dir:="${ROOT_DIR}" \
    config_basename:="${CARTO_CONFIG_BASENAME}" \
    scan_topic:="${carto_scan_topic}" >/tmp/cartographer.log 2>&1 &
  sleep 8
elif [ "${SLAM_BACKEND}" = "slam_toolbox" ]; then
  if [ "${START_ODOM}" = "1" ] && [ "${START_ODOM_TF_RELAY}" = "1" ]; then
    echo "[odom_tf] Odom TF relay..."
    nohup python3 "${ROOT_DIR}/odom_tf_relay.py" >/tmp/odom_tf_relay.log 2>&1 &
    sleep 2
  elif [ "${START_ODOM}" != "1" ] && [ "${START_ODOM_STATIC_TF}" = "1" ]; then
    echo "[odom_tf] Static odom TF..."
    nohup ros2 run tf2_ros static_transform_publisher \
      0 0 0 0 0 0 odom base_link >/tmp/tf_odom_static.log 2>&1 &
    sleep 2
  fi
  echo "[scan] Scan normalizer (${SLAM_SCAN_TOPIC})..."
  nohup python3 "${ROOT_DIR}/scan_metadata_normalizer.py" --ros-args \
    -p input_topic:=/scan \
    -p output_topic:="${SLAM_SCAN_TOPIC}" \
    -p adjust_angle_increment:=true \
    -p prefer_stamp_delta_for_scan_time:=true \
    -p derive_time_increment_from_scan_time:=true >/tmp/scan_normalizer.log 2>&1 &
  sleep 2
  echo "[8/12] slam_toolbox..."
  nohup ros2 launch "${SLAM_TOOLBOX_LAUNCH}" \
    params_file:="${SLAM_TOOLBOX_PARAMS}" >/tmp/slam_toolbox.log 2>&1 &
  sleep 8
else
  echo "[8/12] SLAM skipped (SMART3D_SLAM_BACKEND=${SLAM_BACKEND})"
fi

echo "[10/12] Campus map bridge..."
nohup python3 "${ROOT_DIR}/campus_map_bridge.py" >/tmp/campus_map_bridge.log 2>&1 &
sleep 2

echo "[11/12] Robot state publisher..."
robot_description="$(tr '\n' ' ' < "${ROOT_DIR}/robotcar.urdf")"
nohup ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="${robot_description}" >/tmp/robot_state_publisher.log 2>&1 &
sleep 2

echo "[12/12] Pi5 client..."
nohup python3 "${ROOT_DIR}/pi5_client.py" \
  --ros-args -p car_ns:="${CAR_NS}" -p robot_speed:="${ROBOT_SPEED}" -p campus_mode:=true \
  >/tmp/pi5_client_campus.log 2>&1 &
sleep 4

if [ "${START_WATCHDOG}" = "1" ]; then
  echo "[watchdog] Pi ROS watchdog..."
  nohup python3 "${ROOT_DIR}/pi_ros_watchdog.py" >/tmp/pi_ros_watchdog.log 2>&1 &
  sleep 2
fi

if [ "${START_ACTIVE}" = "1" ] && [ "${CMD_VEL_DRIVER_STARTED}" != "1" ]; then
  start_cmd_vel_driver
fi

echo "==== Topic Probe (5s) ===="
python3 - <<PY
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan

CAR_NS = "${CAR_NS}"

rclpy.init()
node = rclpy.create_node("start_full_native_probe")
counts = {"scan": 0, "imu": 0, "map": 0, "pose": 0}

node.create_subscription(
    LaserScan,
    "/scan",
    lambda _msg: counts.__setitem__("scan", counts["scan"] + 1),
    QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
)
node.create_subscription(
    Imu,
    "/IMU_data",
    lambda _msg: counts.__setitem__("imu", counts["imu"] + 1),
    QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
)
node.create_subscription(
    OccupancyGrid,
    "/map",
    lambda _msg: counts.__setitem__("map", counts["map"] + 1),
    QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    ),
)
node.create_subscription(
    PoseStamped,
    f"/{CAR_NS}/pose",
    lambda _msg: counts.__setitem__("pose", counts["pose"] + 1),
    QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
)

deadline = time.time() + 5.0
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)

print(f"/scan msgs={counts['scan']}")
print(f"/IMU_data msgs={counts['imu']}")
print(f"/map msgs={counts['map']}")
print(f"/{CAR_NS}/pose msgs={counts['pose']}")

node.destroy_node()
rclpy.shutdown()
PY

echo "Jetson full runtime started."
