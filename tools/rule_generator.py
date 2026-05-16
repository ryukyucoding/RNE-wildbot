import serial
import glob
import time
import json
import subprocess
import re

# 查找 /dev/ttyUSB* 和 /dev/ttyACM* 之下的所有設備
def find_serial_ports():
    return glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')

# 使用 udevadm 獲取設備的所有 KERNELS=="x-x" 信息
def get_kernel_info(serial_port):
    try:
        # 執行 udevadm 命令以獲取 KERNELS 信息
        output = subprocess.check_output(["udevadm", "info", "-a", "-n", serial_port])
        kernel_info = []
        for line in output.decode().splitlines():
            # 使用正則表達式過濾出形如 KERNELS=="x-x" 的行
            if re.search(r'KERNELS=="\d+-\d+"', line):
                kernel_info.append(line.strip())
        return kernel_info
    except Exception as e:
        print(f"Error getting KERNELS info for {serial_port}: {e}")
    return None

# 生成 USB udev 規則
def generate_udev_rule(custom_id, kernel_info, serial_port):
    rule_file = f"/etc/udev/rules.d/99-{custom_id}.rules"
    try:
        with open(rule_file, 'w') as f:
            # 根據 CUSTOM_ID 和 kernel_info 生成 udev 規則
            for kernel in kernel_info:
                kernel_number = kernel.split('==')[1].strip('"')
                rule = f'SUBSYSTEM=="tty", KERNELS=="{kernel_number}", SYMLINK+="{custom_id}", MODE="0666"\n'
                f.write(rule)
        print(f"USB rule added for {serial_port} with CUSTOM_ID: {custom_id}")
    except Exception as e:
        print(f"Error writing to udev rule file: {e}")

# 重載 udev 規則並觸發
def reload_udev_rules():
    try:
        subprocess.check_call(["sudo", "udevadm", "control", "--reload-rules"])
        subprocess.check_call(["sudo", "udevadm", "trigger"])
        print("udev rules reloaded and triggered successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Error reloading udev rules: {e}")

# 發送 JSON 指令並接收 CUSTOM_ID
def send_command_and_receive_custom_id(serial_port, baud_rate=115200, timeout=5):
    try:
        # 打開串口
        ser = serial.Serial(serial_port, baudrate=baud_rate, timeout=timeout)
        
        # 等待設備準備好
        time.sleep(2)
        
        # 發送 JSON 指令
        command = json.dumps({"command": "I"}) + "\n"
        ser.write(command.encode())
        
        # 等待接收 Arduino 傳送回的 CUSTOM_ID
        response = ser.readline().decode().strip()
        ser.close()

        # 如果收到回應，顯示並打印 kernel info
        if response:
            kernel_info = get_kernel_info(serial_port)
            if kernel_info:
                print(f"Received from {serial_port}: {response}")
                print("Matching KERNELS info:")
                for kernel in kernel_info:
                    print(kernel)
                # 生成 udev 規則
                generate_udev_rule(response, kernel_info, serial_port)
                reload_udev_rules()
            else:
                print(f"Received from {serial_port}: {response}, but no matching KERNELS info found.")
        else:
            print(f"No response from {serial_port}")
    except Exception as e:
        print(f"Error communicating with {serial_port}: {e}")

if __name__ == "__main__":
    # 找到所有 ttyUSB* 和 ttyACM* 設備
    serial_ports = find_serial_ports()

    # 如果有找到設備，發送指令並接收回應
    if serial_ports:
        for port in serial_ports:
            print(f"Communicating with {port}...")
            send_command_and_receive_custom_id(port)
    else:
        print("No serial devices found.")
