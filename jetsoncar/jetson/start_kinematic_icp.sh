#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
KINEMATIC_ICP_WS="${SMART3D_KINEMATIC_ICP_WS:-$HOME/SMART3D/ws_kinematic_icp}"
KINEMATIC_ICP_CONFIG="${KINEMATIC_ICP_CONFIG:-${KINEMATIC_ICP_WS}/src/kinematic-icp/ros/config/kinematic_icp_ros.yaml}"
LIDAR_TOPIC="${KINEMATIC_ICP_LIDAR_TOPIC:-/scan}"
LIDAR_ODOMETRY_TOPIC="${KINEMATIC_ICP_LIDAR_ODOMETRY_TOPIC:-/lidar_odometry_kinematic}"
LIDAR_ODOM_FRAME="${KINEMATIC_ICP_LIDAR_ODOM_FRAME:-odom_kinematic}"
WHEEL_ODOM_FRAME="${KINEMATIC_ICP_WHEEL_ODOM_FRAME:-odom_wheel}"
BASE_FRAME="${KINEMATIC_ICP_BASE_FRAME:-base_link}"
USE_2D_LIDAR="${KINEMATIC_ICP_USE_2D_LIDAR:-true}"
PUBLISH_ODOM_TF="${KINEMATIC_ICP_PUBLISH_ODOM_TF:-false}"
INVERT_ODOM_TF="${KINEMATIC_ICP_INVERT_ODOM_TF:-true}"
TF_TIMEOUT="${KINEMATIC_ICP_TF_TIMEOUT:-0.2}"
STOP_RIVAL_ODOM_PUBLISHERS="${KINEMATIC_ICP_STOP_RIVAL_ODOM_PUBLISHERS:-true}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[ -f "${KINEMATIC_ICP_WS}/install/setup.bash" ] && source "${KINEMATIC_ICP_WS}/install/setup.bash"
set -u

if [ ! -f "${KINEMATIC_ICP_CONFIG}" ]; then
  echo "missing config: ${KINEMATIC_ICP_CONFIG}" >&2
  exit 1
fi

if [ "${STOP_RIVAL_ODOM_PUBLISHERS}" = "true" ]; then
  pkill -f '[r]f2o_laser_odometry_node' 2>/dev/null || true
  pkill -f '[C]LaserOdometry2DNode' 2>/dev/null || true
  pkill -f '[e]kf_node' 2>/dev/null || true
fi

pkill -f '[k]inematic_icp_online_node' 2>/dev/null || true
sleep 1

nohup ros2 run kinematic_icp kinematic_icp_online_node \
  --ros-args \
  -r lidar_odometry:="${LIDAR_ODOMETRY_TOPIC}" \
  --params-file "${KINEMATIC_ICP_CONFIG}" \
  -p lidar_topic:="${LIDAR_TOPIC}" \
  -p use_2d_lidar:="${USE_2D_LIDAR}" \
  -p wheel_odom_frame:="${WHEEL_ODOM_FRAME}" \
  -p base_frame:="${BASE_FRAME}" \
  -p lidar_odom_frame:="${LIDAR_ODOM_FRAME}" \
  -p publish_odom_tf:="${PUBLISH_ODOM_TF}" \
  -p invert_odom_tf:="${INVERT_ODOM_TF}" \
  -p tf_timeout:="${TF_TIMEOUT}" \
  >/tmp/kinematic_icp.log 2>&1 < /dev/null &

sleep 4
pgrep -af 'kinematic_icp_online_node' || true
echo '==== KINEMATIC-ICP ===='
tail -n 40 /tmp/kinematic_icp.log 2>/dev/null || true
