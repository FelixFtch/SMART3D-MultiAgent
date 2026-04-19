#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
RESPAWN_DELAY_SEC="${SMART3D_ODOM_RESPAWN_DELAY_SEC:-2.0}"
MAX_RESTARTS="${SMART3D_ODOM_MAX_RESTARTS:-0}"

if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  echo "[odom_supervisor] missing /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 1
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

attempt=0
while true; do
  attempt=$((attempt + 1))
  echo "[odom_supervisor] attempt=${attempt}"
  "${ROOT_DIR}/mk_ddsm_dev.sh" >/tmp/mk_ddsm_dev.log 2>&1 || true

  set +e
  python3 "${ROOT_DIR}/ddsm115_odom.py"
  rc=$?
  set -e

  echo "[odom_supervisor] ddsm115_odom.py exited rc=${rc}"
  if [ "${MAX_RESTARTS}" != "0" ] && [ "${attempt}" -ge "${MAX_RESTARTS}" ]; then
    exit "${rc}"
  fi

  if [ "${RESPAWN_DELAY_SEC}" != "0" ] && [ "${RESPAWN_DELAY_SEC}" != "0.0" ]; then
    echo "[odom_supervisor] respawn delay ${RESPAWN_DELAY_SEC}s"
    sleep "${RESPAWN_DELAY_SEC}"
  fi
done
