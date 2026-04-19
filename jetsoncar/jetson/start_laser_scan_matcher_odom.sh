#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
SCAN_ODOM_WS="${SMART3D_SCAN_ODOM_WS:-$HOME/SMART3D/ws_scan_odom}"
PARAMS_FILE="${LASER_SCAN_MATCHER_PARAMS_FILE:-${ROOT_DIR}/laser_scan_matcher_params.yaml}"
PUBLISH_TF="${LASER_SCAN_MATCHER_PUBLISH_TF:-true}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[ -f "${SCAN_ODOM_WS}/install/setup.bash" ] && source "${SCAN_ODOM_WS}/install/setup.bash"
set -u

pkill -f '[/]rf2o_laser_odometry_node' 2>/dev/null || true
pkill -f '[/]laser_scan_matcher' 2>/dev/null || true
sleep 1

nohup ros2 run ros2_laser_scan_matcher laser_scan_matcher \
  --ros-args \
  --params-file "${PARAMS_FILE}" \
  -r scan:=/scan \
  -p publish_tf:="${PUBLISH_TF}" >/tmp/laser_scan_matcher_odom.log 2>&1 < /dev/null &

sleep 4
pgrep -af 'laser_scan_matcher' || true
tail -n 30 /tmp/laser_scan_matcher_odom.log 2>/dev/null || true
