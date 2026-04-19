#!/usr/bin/env bash
set -euo pipefail

PRIMARY_SER="5A6D010263"
FALLBACK_SER="5A6C069316"
IMU_SER="5B0B030655"
FOUND=""
FOUND_SER=""

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
  if [ "$ser" = "$PRIMARY_SER" ] || [ "$ser" = "$FALLBACK_SER" ]; then
    FOUND="$dev"
    FOUND_SER="$ser"
    break
  fi
done

if [ -z "$FOUND" ]; then
  for dev in /dev/ttyACM*; do
    [ -e "$dev" ] || continue
    ser="$(find_serial_for_dev "$dev" || true)"
    if [ -n "$ser" ] && [ "$ser" != "$IMU_SER" ]; then
      FOUND="$dev"
      FOUND_SER="$ser"
      echo "[mk_m10p_dev] WARNING: expected serial $PRIMARY_SER not found, using fallback serial $ser on $dev"
      break
    fi
  done
fi

if [ -z "$FOUND" ]; then
  echo "[mk_m10p_dev] ERROR: cannot find M10P under /dev/ttyACM*" >&2
  ls -l /dev/ttyACM* 2>/dev/null || true
  exit 1
fi

ln -sf "$FOUND" /dev/m10p
echo "[mk_m10p_dev] /dev/m10p -> $FOUND (serial=${FOUND_SER:-unknown})"
ls -l /dev/m10p
