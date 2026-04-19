#!/usr/bin/env python3
import sys
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


def frame(motor_id, brake):
    body = [motor_id, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, brake, 0x00]
    return bytes(body + [crc8_maxim(body)])


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    ser = serial.Serial(port, 115200, timeout=0.05)
    try:
        zero_frames = [frame(i, 0x00) for i in range(1, 5)]
        brake_frames = [frame(i, 0xFF) for i in range(1, 5)]
        for _ in range(60):
            for pkt in zero_frames:
                ser.write(pkt)
            time.sleep(0.01)
            for pkt in brake_frames:
                ser.write(pkt)
            ser.flush()
            time.sleep(0.02)
        print("BRAKES_SENT")
        for pkt in brake_frames:
            print(pkt.hex())
    finally:
        ser.close()


if __name__ == "__main__":
    main()
