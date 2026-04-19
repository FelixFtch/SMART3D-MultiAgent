#!/usr/bin/env python3
import glob
import os
import time
import serial

def crc8_maxim(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
            crc &= 0xFF
    return crc

def speed_frame(mid, hi=0x00, lo=0x00, brake=0xFF):
    body = [mid, 0x64, hi, lo, 0x00, 0x00, 0x00, brake, 0x00]
    return bytes(body + [crc8_maxim(body)])

def mode_frame(mid, mode=0x02):
    return bytes([mid, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, mode])

ports = [p for p in ['/dev/ttyUSB0','/dev/ttyUSB1'] if os.path.exists(p)]
if not ports:
    ports = glob.glob('/dev/ttyUSB*')
print('PORTS', ports)
for port in ports:
    try:
        ser = serial.Serial(port, 115200, timeout=0.02)
    except Exception as e:
        print('OPEN_FAIL', port, e)
        continue
    try:
        t_end = time.time() + 20.0
        loops = 0
        while time.time() < t_end:
            for mid in range(1, 9):
                ser.write(mode_frame(mid, 0x02))
            for _ in range(2):
                for mid in range(1, 9):
                    ser.write(speed_frame(mid, 0x00, 0x00, 0x00))
                for mid in range(1, 9):
                    ser.write(speed_frame(mid, 0x00, 0x00, 0xFF))
            ser.flush()
            loops += 1
            time.sleep(0.03)
        print('DONE', port, loops)
    finally:
        ser.close()
