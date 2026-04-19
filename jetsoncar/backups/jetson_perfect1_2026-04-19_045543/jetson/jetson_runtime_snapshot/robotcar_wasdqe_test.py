#!/usr/bin/env python3
"""Direct keyboard motor test for the robotcar Pi 5 base.

Designed to run inside the ros2-humble container:
  docker exec -it ros2-humble python3 /root/robotcar_wasdqe_test.py

Controls:
  w / s : forward / backward
  a / d : in-place left / right turn
  q / e : forward-left / forward-right arc
  x or space: stop
  1 2 3 4 : spin one motor forward (IDs 1..4)
  ! @ # $ : spin one motor backward (IDs 1..4)
  r : probe motor IDs 1..4
  [ / ] : decrease / increase RPM
  p : print current settings
  z : hard brake all 4 motors
  ESC / Ctrl-C: quit and stop

Safety:
  If you stop pressing keys for --deadman-sec, the script auto-brakes.
"""

from __future__ import annotations

import argparse
import math
import select
import serial
import struct
import sys
import termios
import time
import tty
from typing import Dict, Iterable, Tuple


LEFT_IDS = (1, 3)
RIGHT_IDS = (2, 4)


def crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc


def set_rpm_frame(motor_id: int, rpm: int, brake: bool = False) -> bytes:
    brake_byte = 0xFF if brake else 0x00
    body = bytes([motor_id, 0x64]) + struct.pack(">h", int(rpm)) + bytes([0, 0, 0, brake_byte, 0])
    return body + bytes([crc8(body)])


def query_frame(motor_id: int) -> bytes:
    body = bytes([motor_id, 0x74, 0, 0, 0, 0, 0, 0, 0])
    return body + bytes([crc8(body)])


class KeyReader:
    def __enter__(self) -> "KeyReader":
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def read_key(self, timeout: float = 0.1) -> str | None:
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if not rlist:
            return None
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            return "ESC"
        return ch


class MotorTester:
    def __init__(
        self,
        port: str,
        baud: int,
        base_rpm: int,
        turn_rpm: int,
        arc_rpm: int,
        arc_inner_scale: float,
        deadman_sec: float,
        max_rpm_limit: int,
    ) -> None:
        self.port = port
        self.base_rpm = base_rpm
        self.turn_rpm = turn_rpm
        self.arc_rpm = arc_rpm
        self.arc_inner_scale = arc_inner_scale
        self.deadman_sec = deadman_sec
        self.max_rpm_limit = max_rpm_limit
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.current: Dict[int, int] = {1: 0, 2: 0, 3: 0, 4: 0}
        self.last_command_ts = time.monotonic()
        self.deadman_engaged = False

    def write_motor(self, motor_id: int, rpm: int, brake: bool = False) -> None:
        self.current[motor_id] = int(rpm)
        self.ser.write(set_rpm_frame(motor_id, rpm, brake=brake))

    def stop(self) -> None:
        for motor_id in (1, 2, 3, 4):
            self.write_motor(motor_id, 0)
        self.last_command_ts = time.monotonic()
        self.deadman_engaged = False

    def brake(self, repeats: int = 3) -> None:
        for _ in range(repeats):
            for motor_id in (1, 2, 3, 4):
                self.write_motor(motor_id, 0, brake=True)
            time.sleep(0.02)
        self.last_command_ts = time.monotonic()
        self.deadman_engaged = False

    def drive_wheel_sides(self, left_rpm: int, right_rpm: int) -> None:
        # Match the current cmd_vel_driver mapping:
        # left IDs use left_rpm directly, right IDs are sign-flipped.
        for motor_id in LEFT_IDS:
            self.write_motor(motor_id, left_rpm)
        for motor_id in RIGHT_IDS:
            self.write_motor(motor_id, -right_rpm)
        self.last_command_ts = time.monotonic()
        self.deadman_engaged = False

    def twist(self, linear_x: float, angular_z: float, wheel_radius: float = 0.05, track_width: float = 0.4) -> Tuple[int, int]:
        v_left = linear_x - angular_z * track_width / 2.0
        v_right = linear_x + angular_z * track_width / 2.0
        rpm_left = int(v_left / (2.0 * math.pi * wheel_radius) * 60.0)
        rpm_right = int(v_right / (2.0 * math.pi * wheel_radius) * 60.0)
        return rpm_left, rpm_right

    def apply_chassis_command(self, key: str) -> str:
        if key == "w":
            self.drive_wheel_sides(self.base_rpm, self.base_rpm)
            return f"forward  L={self.base_rpm} R={self.base_rpm}"
        if key == "s":
            self.drive_wheel_sides(-self.base_rpm, -self.base_rpm)
            return f"backward L={-self.base_rpm} R={-self.base_rpm}"
        if key == "a":
            left, right = self.twist(0.0, +self.turn_rpm * 2.0 * math.pi * 0.05 / (60.0 * 0.2))
            self.drive_wheel_sides(left, right)
            return f"turn-left L={left} R={right}"
        if key == "d":
            left, right = self.twist(0.0, -self.turn_rpm * 2.0 * math.pi * 0.05 / (60.0 * 0.2))
            self.drive_wheel_sides(left, right)
            return f"turn-right L={left} R={right}"
        if key == "q":
            inner = int(round(self.arc_rpm * self.arc_inner_scale))
            self.drive_wheel_sides(inner, self.arc_rpm)
            return f"arc-left L={inner} R={self.arc_rpm}"
        if key == "e":
            inner = int(round(self.arc_rpm * self.arc_inner_scale))
            self.drive_wheel_sides(self.arc_rpm, inner)
            return f"arc-right L={self.arc_rpm} R={inner}"
        if key in ("x", " "):
            self.stop()
            return "stop"
        if key == "z":
            self.brake()
            return "hard brake"
        return ""

    def single_motor(self, key: str) -> str:
        mapping = {
            "1": (1, +self.base_rpm),
            "2": (2, +self.base_rpm),
            "3": (3, +self.base_rpm),
            "4": (4, +self.base_rpm),
            "!": (1, -self.base_rpm),
            "@": (2, -self.base_rpm),
            "#": (3, -self.base_rpm),
            "$": (4, -self.base_rpm),
        }
        if key not in mapping:
            return ""
        self.stop()
        motor_id, rpm = mapping[key]
        self.write_motor(motor_id, rpm)
        self.last_command_ts = time.monotonic()
        self.deadman_engaged = False
        return f"single motor ID {motor_id} -> {rpm} rpm"

    def probe(self) -> str:
        lines = []
        for motor_id in (1, 2, 3, 4):
            vals = []
            ok = 0
            for _ in range(5):
                self.ser.reset_input_buffer()
                self.ser.write(query_frame(motor_id))
                resp = self.ser.read(10)
                if len(resp) == 10 and resp[0] == motor_id and crc8(resp[:9]) == resp[9]:
                    vals.append(struct.unpack_from(">h", resp, 4)[0])
                    ok += 1
                time.sleep(0.03)
            lines.append(f"ID {motor_id}: ok={ok}/5 vals={vals}")
        return "\n".join(lines)

    def maybe_deadman_stop(self) -> str:
        if all(rpm == 0 for rpm in self.current.values()):
            return ""
        idle = time.monotonic() - self.last_command_ts
        if idle < self.deadman_sec:
            return ""
        if not self.deadman_engaged:
            self.brake()
            self.deadman_engaged = True
            return f"deadman brake after {idle:.2f}s"
        return ""


def print_help(tester: MotorTester) -> None:
    print("")
    print("robotcar_wasdqe_test")
    print(
        f"port={tester.port} base_rpm={tester.base_rpm} turn_rpm={tester.turn_rpm} "
        f"arc_rpm={tester.arc_rpm} deadman={tester.deadman_sec:.2f}s "
        f"max_limit={tester.max_rpm_limit}"
    )
    print("w/s forward-back | a/d turn | q/e arc | x/space stop | z hard brake")
    print("1 2 3 4 one motor forward | ! @ # $ one motor backward")
    print("r probe motor replies | [ ] adjust base rpm | p print status | ESC quit")
    print("")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--base-rpm", type=int, default=35)
    parser.add_argument("--turn-rpm", type=int, default=28)
    parser.add_argument("--arc-rpm", type=int, default=35)
    parser.add_argument("--arc-inner-scale", type=float, default=0.35)
    # SSH/terminal key-repeat can be bursty; a slightly longer deadman feels
    # much more natural while still braking quickly if input stops.
    parser.add_argument("--deadman-sec", type=float, default=0.8)
    parser.add_argument("--max-rpm-limit", type=int, default=330)
    args = parser.parse_args()

    tester = MotorTester(
        port=args.port,
        baud=args.baud,
        base_rpm=args.base_rpm,
        turn_rpm=args.turn_rpm,
        arc_rpm=args.arc_rpm,
        arc_inner_scale=args.arc_inner_scale,
        deadman_sec=args.deadman_sec,
        max_rpm_limit=args.max_rpm_limit,
    )
    print_help(tester)

    try:
        with KeyReader() as reader:
            while True:
                key = reader.read_key(timeout=0.1)
                if key is None:
                    msg = tester.maybe_deadman_stop()
                    if msg:
                        print(msg)
                    continue
                if key in ("ESC", "\x03"):
                    break
                if key == "[":
                    tester.base_rpm = max(5, tester.base_rpm - 5)
                    tester.turn_rpm = max(5, tester.turn_rpm - 5)
                    tester.arc_rpm = max(5, tester.arc_rpm - 5)
                    print_help(tester)
                    continue
                if key == "]":
                    tester.base_rpm = min(tester.max_rpm_limit, tester.base_rpm + 10)
                    tester.turn_rpm = min(tester.max_rpm_limit, tester.turn_rpm + 10)
                    tester.arc_rpm = min(tester.max_rpm_limit, tester.arc_rpm + 10)
                    print_help(tester)
                    continue
                if key == "p":
                    print_help(tester)
                    continue
                if key == "r":
                    print(tester.probe())
                    continue
                msg = tester.apply_chassis_command(key)
                if not msg:
                    msg = tester.single_motor(key)
                if msg:
                    print(msg)
    finally:
        tester.brake()
        tester.ser.close()
        print("stopped")


if __name__ == "__main__":
    main()
