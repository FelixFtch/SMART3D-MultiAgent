#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIDAR_WS="${SMART3D_LIDAR_WS:-$HOME/SMART3D/ws_lidar}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
PARAMS_FILE="${SMART3D_LIDAR_PARAMS:-${ROOT_DIR}/m10p_uart.yaml}"
STARTUP_DELAY_SEC="${SMART3D_LIDAR_STARTUP_DELAY_SEC:-6.0}"
RESPAWN_DELAY_SEC="${SMART3D_LIDAR_RESPAWN_DELAY_SEC:-3.0}"
MAX_RESTARTS="${SMART3D_LIDAR_MAX_RESTARTS:-0}"

if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  echo "[lidar_supervisor] missing /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 1
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[ -f "${LIDAR_WS}/install/setup.bash" ] && source "${LIDAR_WS}/install/setup.bash"
set -u

attempt=0
while true; do
  attempt=$((attempt + 1))
  echo "[lidar_supervisor] attempt=${attempt} params=${PARAMS_FILE}"
  "${ROOT_DIR}/mk_m10p_dev.sh" >/tmp/mk_m10p_dev.log 2>&1 || true

  if [ "${attempt}" -eq 1 ] && [ "${STARTUP_DELAY_SEC}" != "0" ] && [ "${STARTUP_DELAY_SEC}" != "0.0" ]; then
    echo "[lidar_supervisor] startup delay ${STARTUP_DELAY_SEC}s"
    sleep "${STARTUP_DELAY_SEC}"
  fi

  set +e
  ros2 run lslidar_driver lslidar_driver_node --ros-args --params-file "${PARAMS_FILE}"
  rc=$?
  set -e

  echo "[lidar_supervisor] lslidar_driver_node exited rc=${rc}"
  if [ "${MAX_RESTARTS}" != "0" ] && [ "${attempt}" -ge "${MAX_RESTARTS}" ]; then
    exit "${rc}"
  fi

  if [ "${RESPAWN_DELAY_SEC}" != "0" ] && [ "${RESPAWN_DELAY_SEC}" != "0.0" ]; then
    echo "[lidar_supervisor] respawn delay ${RESPAWN_DELAY_SEC}s"
    sleep "${RESPAWN_DELAY_SEC}"
  fi
done
