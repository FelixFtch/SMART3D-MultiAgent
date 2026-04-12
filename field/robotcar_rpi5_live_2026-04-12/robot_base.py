import sys
import select
import termios
import tty
import serial
import time

COM_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
MAX_RPM = 80

def calculate_crc8_maxim(data_bytes):
    crc = 0x00
    for byte in data_bytes:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc

def generate_motor_cmd(motor_id, rpm):
    rpm = max(-330, min(330, int(rpm)))
    rpm_val = (1 << 16) + rpm if rpm < 0 else rpm
    frame = [motor_id, 0x64, (rpm_val >> 8) & 0xFF, rpm_val & 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00]
    frame.append(calculate_crc8_maxim(frame))
    return bytes(frame)

def get_key(settings, timeout=0.05):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    print(f"[INFO] Connecting to {COM_PORT}...")
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
        print("[INFO] Connection established.\n")
        print("=== Linux Terminal Chassis Control ===")
        print("    Q    W    E   |  Q/E: Arc Turns")
        print("    A    S    D   |  A/D: Pivot Turns")
        print("--------------------------------------")
        print(" Smooth Start/Stop Watchdog Enabled.")
        print(" Press Ctrl+C to Exit.")
        print("======================================\n")
    except Exception as e:
        print(f"[ERROR] Serial port failed: {e}")
        return

    settings = termios.tcgetattr(sys.stdin)
    
    linear_x = 0.0
    angular_z = 0.0
    last_cmd_time = time.time()
    
    # 🌟 核心修改：将超时容忍度提高到 0.65 秒，完美覆盖操作系统的打字死区
    TIMEOUT_TOLERANCE = 0.65  

    try:
        while True:
            key = get_key(settings, timeout=0.05)
            
            if key == '\x03':  # Ctrl+C
                break

            if key in ['w', 's', 'a', 'd', 'q', 'e']:
                last_cmd_time = time.time()
                
                if key == 'w':
                    linear_x = 1.0; angular_z = 0.0
                elif key == 's':
                    linear_x = -1.0; angular_z = 0.0
                elif key == 'a':
                    linear_x = 0.0; angular_z = -1.0
                elif key == 'd':
                    linear_x = 0.0; angular_z = 1.0
                elif key == 'q':
                    linear_x = 1.0; angular_z = -0.5
                elif key == 'e':
                    linear_x = 1.0; angular_z = 0.5
            else:
                if time.time() - last_cmd_time > TIMEOUT_TOLERANCE:
                    linear_x = 0.0
                    angular_z = 0.0

            left_target = linear_x + angular_z
            right_target = linear_x - angular_z

            max_val = max(abs(left_target), abs(right_target))
            if max_val > 1.0:
                left_target /= max_val
                right_target /= max_val

            motor_rpms = {
                1: left_target * MAX_RPM,
                3: left_target * MAX_RPM,
                2: -right_target * MAX_RPM,
                4: -right_target * MAX_RPM
            }

            batch_cmd = b''
            for m_id, rpm in motor_rpms.items():
                batch_cmd += generate_motor_cmd(m_id, rpm)
            
            ser.write(batch_cmd)
            ser.flush()

    except Exception as e:
        print(f"\n[ERROR] {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\n[INFO] Executing emergency stop...")
        try:
            batch_cmd = b''
            for m_id in [1, 2, 3, 4]:
                batch_cmd += generate_motor_cmd(m_id, 0)
            ser.write(batch_cmd)
            ser.flush()
            ser.close()
            print("[INFO] Chassis safely powered down.")
        except:
            pass

if __name__ == "__main__":
    main()
