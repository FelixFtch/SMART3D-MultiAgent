#!/usr/bin/env bash
set -euo pipefail

JETSON_HOST="${JETSON_HOST:-jetson@10.66.121.109}"
JETSON_PASSWORD="${JETSON_PASSWORD:-jetson}"
JETSON_RUNTIME_DIR="${JETSON_RUNTIME_DIR:-/home/jetson/SMART3D/jetson_runtime}"

BASE_RPM="${BASE_RPM:-18}"
TURN_RPM="${TURN_RPM:-14}"
ARC_RPM="${ARC_RPM:-18}"
ARC_INNER_SCALE="${ARC_INNER_SCALE:-0.35}"
DEADMAN_SEC="${DEADMAN_SEC:-0.8}"
MAX_RPM_LIMIT="${MAX_RPM_LIMIT:-80}"

if ! command -v sshpass >/dev/null 2>&1; then
  echo "missing sshpass" >&2
  exit 1
fi

echo "[mac-wasd] connecting to ${JETSON_HOST}"
echo "[mac-wasd] base=${BASE_RPM} turn=${TURN_RPM} arc=${ARC_RPM} deadman=${DEADMAN_SEC}s"
echo "[mac-wasd] ESC quits, x/space stops, z hard-brakes"

exec sshpass -p "${JETSON_PASSWORD}" ssh -tt \
  -o StrictHostKeyChecking=no \
  "${JETSON_HOST}" \
  "bash -lc 'cd \"${JETSON_RUNTIME_DIR}\" && \
    BASE_RPM=\"${BASE_RPM}\" \
    TURN_RPM=\"${TURN_RPM}\" \
    ARC_RPM=\"${ARC_RPM}\" \
    ARC_INNER_SCALE=\"${ARC_INNER_SCALE}\" \
    DEADMAN_SEC=\"${DEADMAN_SEC}\" \
    MAX_RPM_LIMIT=\"${MAX_RPM_LIMIT}\" \
    ./run_robotcar_wasdqe_test.sh'"
