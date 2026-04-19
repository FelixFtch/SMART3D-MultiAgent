#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIDAR_WS="${SMART3D_LIDAR_WS:-$HOME/SMART3D/ws_lidar}"
IMU_WS="${SMART3D_IMU_WS:-$HOME/SMART3D/ws_imu}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
IMU_PORT="${SMART3D_IMU_PORT:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B030655-if00}"
LIDAR_PARAMS="${SMART3D_LIDAR_PARAMS:-${ROOT_DIR}/m10p_uart.yaml}"
IMU_BRIDGE="${SMART3D_IMU_BRIDGE:-${ROOT_DIR}/imu_qos_bridge.py}"
ODOM_NODE="${SMART3D_ODOM_NODE:-${ROOT_DIR}/ddsm115_odom.py}"
CARTO_LAUNCH="${SMART3D_CARTO_LAUNCH:-${ROOT_DIR}/cartographer_launch.py}"
SCAN_NORMALIZER="${SMART3D_SCAN_NORMALIZER:-${ROOT_DIR}/scan_metadata_normalizer.py}"
START_PI5_REFERENCE_WHEEL_ODOM="${START_PI5_REFERENCE_WHEEL_ODOM:-0}"
CARTO_CONFIG="${SMART3D_CARTO_CONFIG_BASENAME:-robotcar_2d.lua}"
SLAM_SCAN_TOPIC="${SMART3D_SLAM_SCAN_TOPIC:-/scan_slam}"

need_file() {
  local path="$1"
  if [ ! -f "${path}" ]; then
    echo "[pi5-reference-passive] missing required file: ${path}" >&2
    exit 1
  fi
}

need_file "/opt/ros/${ROS_DISTRO}/setup.bash"
need_file "${ROOT_DIR}/mk_imu_dev.sh"
need_file "${ROOT_DIR}/mk_m10p_dev.sh"
need_file "${ROOT_DIR}/mk_ddsm_dev.sh"
need_file "${LIDAR_PARAMS}"
need_file "${IMU_BRIDGE}"
need_file "${CARTO_LAUNCH}"
need_file "${SCAN_NORMALIZER}"
need_file "${ROOT_DIR}/${CARTO_CONFIG}"
if [ "${START_PI5_REFERENCE_WHEEL_ODOM}" = "1" ]; then
  need_file "${ODOM_NODE}"
fi

if [ -n "${CYCLONEDDS_URI:-}" ]; then
  export CYCLONEDDS_URI
fi
export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[ -f "${LIDAR_WS}/install/setup.bash" ] && source "${LIDAR_WS}/install/setup.bash"
[ -f "${IMU_WS}/install/setup.bash" ] && source "${IMU_WS}/install/setup.bash"
set -u

pkill -f "[c]artographer_node" 2>/dev/null || true
pkill -f "[o]ccupancy_grid_node" 2>/dev/null || true
pkill -f "[l]slidar_driver_node" 2>/dev/null || true
pkill -f "[h]ipnuc_imu" 2>/dev/null || true
pkill -f "[i]mu_qos_bridge.py" 2>/dev/null || true
pkill -f "[d]dsm115_odom.py" 2>/dev/null || true
pkill -f "[s]can_metadata_normalizer.py" 2>/dev/null || true
pkill -f "[s]tatic_transform_publisher .* base_link laser_link" 2>/dev/null || true
sleep 2

rm -f /tmp/lidar.log \
      /tmp/imu_talker.log \
      /tmp/imu_bridge.log \
      /tmp/odom.log \
      /tmp/scan_normalizer.log \
      /tmp/cartographer.log \
      /tmp/tf_static.log
rm -rf /dev/shm/fastrtps_* /dev/shm/*fastdds* /dev/shm/*SHM* 2>/dev/null || true

"${ROOT_DIR}/mk_imu_dev.sh" >/tmp/mk_imu_dev.log 2>&1 || true
"${ROOT_DIR}/mk_m10p_dev.sh" >/tmp/mk_m10p_dev.log 2>&1 || true
"${ROOT_DIR}/mk_ddsm_dev.sh" >/tmp/mk_ddsm_dev.log 2>&1 || true

echo "[1/6] Static TF..."
nohup ros2 run tf2_ros static_transform_publisher \
  0 0 0.05 0 0 0 base_link laser_link >/tmp/tf_static.log 2>&1 &
sleep 2

echo "[2/6] LiDAR..."
nohup ros2 run lslidar_driver lslidar_driver_node --ros-args \
  --params-file "${LIDAR_PARAMS}" >/tmp/lidar.log 2>&1 &
sleep 3

echo "[3/6] IMU..."
nohup ros2 run hipnuc_imu talker --ros-args \
  -p serial_port:="${IMU_PORT}" \
  -p baud_rate:=115200 \
  -p imu_topic:=/IMU_data \
  -p frame_id:=base_link >/tmp/imu_talker.log 2>&1 &
sleep 2

echo "[4/6] Bridge..."
nohup python3 "${IMU_BRIDGE}" >/tmp/imu_bridge.log 2>&1 &
sleep 3

if [ "${START_PI5_REFERENCE_WHEEL_ODOM}" = "1" ]; then
  echo "[5/6] Wheel Odom..."
  nohup python3 "${ODOM_NODE}" >/tmp/odom.log 2>&1 &
  sleep 2
else
  echo "[5/6] Wheel Odom skipped (Jetson stable restore path)"
fi

echo "[6/7] Scan normalizer (${SLAM_SCAN_TOPIC})..."
nohup python3 "${SCAN_NORMALIZER}" --ros-args \
  -p input_topic:=/scan \
  -p output_topic:="${SLAM_SCAN_TOPIC}" \
  -p adjust_angle_increment:=false \
  -p prefer_stamp_delta_for_scan_time:=true \
  -p derive_time_increment_from_scan_time:=true \
  -p fallback_scan_hz:=12.0 >/tmp/scan_normalizer.log 2>&1 &
sleep 2

echo "[7/7] Cartographer (LiDAR+IMU+Odom)..."
nohup ros2 launch "${CARTO_LAUNCH}" \
  config_dir:="${ROOT_DIR}" \
  config_basename:="${CARTO_CONFIG}" \
  scan_topic:="${SLAM_SCAN_TOPIC}" >/tmp/cartographer.log 2>&1 &
sleep 8

echo "=== SCAN ==="
timeout 3 ros2 topic hz /scan 2>&1 | head -2 || true
echo "=== TF ==="
timeout 3 ros2 topic hz /tf 2>&1 | head -2 || true
echo "=== ODOM ==="
tail -n 3 /tmp/odom.log 2>/dev/null || true
echo "=== SCAN NORMALIZER ==="
tail -n 3 /tmp/scan_normalizer.log 2>/dev/null || true
echo "=== BRIDGE ==="
tail -n 2 /tmp/imu_bridge.log 2>/dev/null || true
echo "=== CARTO ==="
tail -n 3 /tmp/cartographer.log 2>/dev/null || true
echo "Pi5 reference passive stack started."
