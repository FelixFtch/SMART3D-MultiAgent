#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
SCAN_ODOM_WS="${SMART3D_SCAN_ODOM_WS:-$HOME/SMART3D/ws_scan_odom}"
RF2O_PARAMS_FILE="${RF2O_EKF_RF2O_PARAMS_FILE:-${ROOT_DIR}/rf2o_odom_for_ekf_params.yaml}"
EKF_PARAMS_FILE="${RF2O_EKF_PARAMS_FILE:-${ROOT_DIR}/ekf_rf2o_imu.yaml}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[ -f "${SCAN_ODOM_WS}/install/setup.bash" ] && source "${SCAN_ODOM_WS}/install/setup.bash"
set -u

pkill -f '[/]rf2o_laser_odometry_node' 2>/dev/null || true
pkill -f '[/]laser_scan_matcher' 2>/dev/null || true
pkill -f '[/]robot_localization/.*/ekf_node' 2>/dev/null || true
pkill -f '[e]kf_node --ros-args --params-file .*/ekf_rf2o_imu.yaml' 2>/dev/null || true
sleep 1

nohup ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args \
  --params-file "${RF2O_PARAMS_FILE}" >/tmp/rf2o_raw_for_ekf.log 2>&1 < /dev/null &

sleep 3

nohup ros2 run robot_localization ekf_node \
  --ros-args \
  --params-file "${EKF_PARAMS_FILE}" \
  -r odometry/filtered:=/odom >/tmp/rf2o_ekf.log 2>&1 < /dev/null &

sleep 4
pgrep -af 'rf2o_laser_odometry|rf2o_laser_odometry_node|ekf_node' || true
echo '==== RF2O RAW ===='
tail -n 20 /tmp/rf2o_raw_for_ekf.log 2>/dev/null || true
echo '==== EKF ===='
tail -n 40 /tmp/rf2o_ekf.log 2>/dev/null || true
