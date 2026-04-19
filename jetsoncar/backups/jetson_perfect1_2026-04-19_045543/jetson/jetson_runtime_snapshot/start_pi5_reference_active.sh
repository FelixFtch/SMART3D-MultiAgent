#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
CAR_NS="${CAR_NS:-car1}"
ROBOT_SPEED="${ROBOT_SPEED:-0.24}"
PI5_EMERGENCY_STOP_DIST="${PI5_EMERGENCY_STOP_DIST:-0.05}"
KEEP_PI5_REFERENCE_WHEEL_ODOM="${KEEP_PI5_REFERENCE_WHEEL_ODOM:-0}"

need_file() {
  local path="$1"
  if [ ! -f "${path}" ]; then
    echo "[pi5-reference-active] missing required file: ${path}" >&2
    exit 1
  fi
}

need_file "/opt/ros/${ROS_DISTRO}/setup.bash"
need_file "${ROOT_DIR}/mk_ddsm_dev.sh"
need_file "${ROOT_DIR}/cmd_vel_driver_pi5_reference.py"
need_file "${ROOT_DIR}/pi5_client.py"

if [ -n "${CYCLONEDDS_URI:-}" ]; then
  export CYCLONEDDS_URI
fi
export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

"${ROOT_DIR}/mk_ddsm_dev.sh" >/tmp/mk_ddsm_dev.log 2>&1 || true
pkill -f "[c]md_vel_driver.py" 2>/dev/null || true
pkill -f "[c]md_vel_driver_pi5_reference.py" 2>/dev/null || true
pkill -f "[c]md_vel_driver_single_owner.py" 2>/dev/null || true
if [ "${KEEP_PI5_REFERENCE_WHEEL_ODOM}" != "1" ]; then
  pkill -f "[d]dsm115_odom.py" 2>/dev/null || true
fi
pkill -f "[r]obotcar_wasdqe_test.py" 2>/dev/null || true
pkill -f "[p]i5_client.py" 2>/dev/null || true
sleep 1

echo "[active 1/2] Pi5 reference cmd_vel driver..."
nohup python3 "${ROOT_DIR}/cmd_vel_driver_pi5_reference.py" >/tmp/cmd_vel_driver_campus.log 2>&1 &
sleep 2

echo "[active 2/2] Pi5 client..."
nohup python3 "${ROOT_DIR}/pi5_client.py" \
  --ros-args \
  -p car_ns:="${CAR_NS}" \
  -p robot_speed:="${ROBOT_SPEED}" \
  -p emergency_stop_dist:="${PI5_EMERGENCY_STOP_DIST}" \
  -p campus_mode:=true \
  >/tmp/pi5_client_campus.log 2>&1 &
sleep 4

echo "Pi5 reference active driver started."
