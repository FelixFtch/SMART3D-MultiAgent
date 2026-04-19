#!/usr/bin/env bash
set -euo pipefail

RUNTIME_DIR="${SMART3D_JETSON_RUNTIME_DIR:-/home/jetson/SMART3D/jetson_runtime}"
SERIAL_PORT="${SMART3D_DDSM_PORT:-/dev/ttyUSB0}"
BASE_RPM="${BASE_RPM:-18}"
TURN_RPM="${TURN_RPM:-14}"
ARC_RPM="${ARC_RPM:-18}"
ARC_INNER_SCALE="${ARC_INNER_SCALE:-0.35}"
DEADMAN_SEC="${DEADMAN_SEC:-0.8}"
MAX_RPM_LIMIT="${MAX_RPM_LIMIT:-80}"

if [ ! -f "${RUNTIME_DIR}/robotcar_wasdqe_test.py" ]; then
  echo "[wasd-test] missing ${RUNTIME_DIR}/robotcar_wasdqe_test.py" >&2
  exit 1
fi

if [ ! -e "${SERIAL_PORT}" ]; then
  echo "[wasd-test] missing serial port ${SERIAL_PORT}" >&2
  exit 1
fi

pkill -f '[c]md_vel_driver.py' 2>/dev/null || true
pkill -f '[d]dsm115_odom.py' 2>/dev/null || true
pkill -f '[s]tp23l_hard_brake_guard.py' 2>/dev/null || true
sleep 1

echo "[wasd-test] direct motor test on ${SERIAL_PORT}"
echo "[wasd-test] base=${BASE_RPM} turn=${TURN_RPM} arc=${ARC_RPM} deadman=${DEADMAN_SEC}s"
echo "[wasd-test] controls: w/s/a/d q/e x space z r [ ] p ESC"

exec python3 "${RUNTIME_DIR}/robotcar_wasdqe_test.py" \
  --port "${SERIAL_PORT}" \
  --base-rpm "${BASE_RPM}" \
  --turn-rpm "${TURN_RPM}" \
  --arc-rpm "${ARC_RPM}" \
  --arc-inner-scale "${ARC_INNER_SCALE}" \
  --deadman-sec "${DEADMAN_SEC}" \
  --max-rpm-limit "${MAX_RPM_LIMIT}" \
  "$@"
