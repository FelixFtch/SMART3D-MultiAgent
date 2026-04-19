#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PI_HOST="${PI_HOST:-robotcar@10.66.38.108}"
PI_PASS="${PI_PASS:-234123}"
LOCAL_WASD="${LOCAL_WASD:-$ROOT_DIR/../../../robotics_helpers/robotcar_wasdqe_test.py}"
REMOTE_WASD_HOST="/tmp/robotcar_wasdqe_test.py"
REMOTE_WASD_CONTAINER="/root/robotcar_wasdqe_test.py"

BASE_RPM="${BASE_RPM:-330}"
TURN_RPM="${TURN_RPM:-240}"
ARC_RPM="${ARC_RPM:-300}"
DEADMAN_SEC="${DEADMAN_SEC:-0.35}"
MAX_RPM_LIMIT="${MAX_RPM_LIMIT:-330}"

log() {
  printf '[robotcar-wasd-max] %s\n' "$*"
}

run_pi() {
  sshpass -p "$PI_PASS" ssh \
    -o StrictHostKeyChecking=no \
    -o PreferredAuthentications=password \
    -o PubkeyAuthentication=no \
    -o ConnectTimeout=8 \
    "$PI_HOST" "$@"
}

copy_pi() {
  sshpass -p "$PI_PASS" scp -O \
    -o StrictHostKeyChecking=no \
    -o PreferredAuthentications=password \
    -o PubkeyAuthentication=no \
    -o ConnectTimeout=8 \
    "$1" "$PI_HOST:$2" >/dev/null
}

if [ ! -f "$LOCAL_WASD" ]; then
  echo "missing WASD script: $LOCAL_WASD" >&2
  exit 1
fi

log "copying WASD controller to Pi"
copy_pi "$LOCAL_WASD" "$REMOTE_WASD_HOST"

log "ensuring container is up"
run_pi "docker start ros2-humble >/dev/null 2>&1 || true"

log "stopping other RS485 owners"
run_pi "docker exec ros2-humble bash -lc 'pkill -f \"[c]md_vel_driver.py\" || true; pkill -f \"[p]i5_client.py\" || true'"

log "installing controller into container"
run_pi "docker cp $REMOTE_WASD_HOST ros2-humble:$REMOTE_WASD_CONTAINER && docker exec ros2-humble chmod +x $REMOTE_WASD_CONTAINER"

log "launching interactive high-speed WASD controller"
log "emergency: x/space stop, z hard brake, Ctrl-C exit+brake"
run_pi "docker exec -it ros2-humble python3 $REMOTE_WASD_CONTAINER --port /dev/ttyUSB0 --base-rpm $BASE_RPM --turn-rpm $TURN_RPM --arc-rpm $ARC_RPM --deadman-sec $DEADMAN_SEC --max-rpm-limit $MAX_RPM_LIMIT"
