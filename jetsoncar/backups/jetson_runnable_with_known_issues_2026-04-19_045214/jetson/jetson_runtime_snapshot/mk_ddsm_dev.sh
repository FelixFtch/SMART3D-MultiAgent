#!/usr/bin/env bash
set -euo pipefail

TARGET_SER="BG01LYTM"
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

for dev in /dev/ttyUSB*; do
  [ -e "$dev" ] || continue
  ser="$(find_serial_for_dev "$dev" || true)"
  if [ "$ser" = "$TARGET_SER" ]; then
    FOUND="$dev"
    break
  fi
done

if [ -z "$FOUND" ]; then
  echo "[mk_ddsm_dev] ERROR: cannot find FTDI serial=$TARGET_SER under /dev/ttyUSB*" >&2
  ls -l /dev/ttyUSB* 2>/dev/null || true
  exit 1
fi

if [ "$FOUND" = "/dev/ttyUSB0" ]; then
  # Avoid creating a self-referential symlink (/dev/ttyUSB0 -> /dev/ttyUSB0).
  if [ -L /dev/ttyUSB0 ] || [ ! -c /dev/ttyUSB0 ]; then
    rm -f /dev/ttyUSB0
    mknod /dev/ttyUSB0 c 188 0 2>/dev/null || true
    chmod 666 /dev/ttyUSB0 2>/dev/null || true
  fi
  echo "[mk_ddsm_dev] using native /dev/ttyUSB0 for serial=$TARGET_SER"
else
  rm -f /dev/ttyUSB0
  ln -sf "$FOUND" /dev/ttyUSB0
  echo "[mk_ddsm_dev] /dev/ttyUSB0 -> $FOUND"
fi

ls -l /dev/ttyUSB0 2>/dev/null || true
