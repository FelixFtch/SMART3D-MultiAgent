#!/usr/bin/env bash
set -euo pipefail

RUNTIME_DIR="${SMART3D_JETSON_RUNTIME_DIR:-/home/jetson/SMART3D/jetson_runtime}"
DURATION="${DURATION:-20}"

set +u
source /opt/ros/humble/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///tmp/cyclone-campus-jetson.xml}"

echo "[stationary-debug] runtime=${RUNTIME_DIR}"
echo "[stationary-debug] duration=${DURATION}s"
echo "[stationary-debug] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

echo "==== CPU snapshot ===="
ps -eo pid,pcpu,pmem,comm,args --sort=-pcpu | sed -n '1,12p'
echo

exec python3 "${RUNTIME_DIR}/stationary_slam_debug.py" --duration "${DURATION}" "$@"
