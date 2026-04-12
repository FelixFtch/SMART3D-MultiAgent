import subprocess
import glob

def get_serial_devices_info():
    # 扫描常见的 USB 转串口和 ACM 虚拟串口
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    
    print("=" * 40)
    print("🔍 正在扫描串口设备 (ACM/USB)...")
    print("=" * 40)
    
    if not ports:
        print("❌ 没有找到任何 /dev/ttyACM* 或 /dev/ttyUSB* 设备。")
    else:
        for port in ports:
            try:
                # 调用系统底层 udevadm 获取设备硬件信息
                cmd = f"udevadm info -q property -n {port}"
                result = subprocess.check_output(cmd, shell=True, text=True)
                
                vendor = "未知品牌"
                model = "未知型号"
                
                for line in result.split('\n'):
                    # 提取品牌和型号信息
                    if line.startswith('ID_VENDOR='):
                        vendor = line.split('=')[1]
                    elif line.startswith('ID_MODEL='):
                        model = line.split('=')[1]
                
                print(f"🔌 端口路径 : {port}")
                print(f"🏢 品牌厂商 : {vendor}")
                print(f"🏷️ 设备型号 : {model}")
                print("-" * 40)
                
            except subprocess.CalledProcessError:
                print(f"⚠️ 无法读取 {port} 的详细信息，可能是权限不足。")

    print("\n📦 当前系统所有 USB 总线设备一览 (lsusb):")
    # 作为补充，直接打印出全局的 lsusb 信息
    subprocess.run(['lsusb'])

if __name__ == '__main__':
    get_serial_devices_info()
