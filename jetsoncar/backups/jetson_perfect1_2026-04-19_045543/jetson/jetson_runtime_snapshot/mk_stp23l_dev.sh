#!/bin/bash
# Create stable /dev/stp23l symlink for STP-23L sensor
# Serial number: 5A6C069316 (from host /dev/serial/by-id)
TARGET_SERIAL="5A6C069316"
for dev in /dev/ttyACM*; do
    [ -e "$dev" ] || continue
    serial=$(cat /sys/class/tty/$(basename $dev)/device/../serial 2>/dev/null || true)
    if [ "$serial" = "$TARGET_SERIAL" ]; then
        ln -sf "$dev" /dev/stp23l
        echo "[mk_stp23l_dev] /dev/stp23l -> $dev"
        ls -la /dev/stp23l
        exit 0
    fi
done
echo "[mk_stp23l_dev] ERROR: STP-23L (serial $TARGET_SERIAL) not found!"
exit 1
