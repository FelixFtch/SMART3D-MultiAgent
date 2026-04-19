#!/usr/bin/env bash
set -euo pipefail

TARGET_SER="5B0B030655"
FOUND=""

find_serial_for_dev() {
  local dev="$1"
  local sys cur
  sys="$(readlink -f "/sys/class/tty/$(basename "$dev")/device" || true)"
  cur="$sys"
  for _ in $(seq 1 12); do
    if [ -f "$cur/serial" ]; then
      cat "$cur/serial" 2>/dev/null || true
      return 0
    fi
    cur="$(dirname "$cur")"
  done
  return 1
}

for dev in /dev/ttyACM*; do
  [ -e "$dev" ] || continue
  ser="$(find_serial_for_dev "$dev" || true)"
  if [ "$ser" = "$TARGET_SER" ]; then
    FOUND="$dev"
    break
  fi
done

if [ -z "$FOUND" ]; then
  echo "[mk_imu_dev] ERROR: cannot find IMU serial=$TARGET_SER under /dev/ttyACM*" >&2
  ls -l /dev/ttyACM* 2>/dev/null || true
  exit 1
fi

rm -f /dev/imu
ln -sf "$FOUND" /dev/imu
echo "[mk_imu_dev] /dev/imu -> $FOUND"
ls -l /dev/imu 2>/dev/null || true
