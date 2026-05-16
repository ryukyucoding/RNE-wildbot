# USB Device Management Tools

This repository contains tools for identifying and managing USB devices on your main system.

## 🚀 Getting Started

Clone this repository to your system where you need to connect and identify specific USB devices:

```bash
git clone https://github.com/your-username/tools.git
cd tools
```

## 🛠 Tools
### 1. create_usb_rule.sh
Description

This script creates a persistent udev rule for a specified USB device, allowing the system to recognize and assign a fixed name to it.

Usage
```
sudo ./create_usb_rule.sh /dev/ttyUSB0 usb_custom_name
```

Replace /dev/ttyUSB0 with your device path and usb_custom_name with the desired symlink name.

### 2. extract_usb_info.sh

#### Description
Quickly extracts and lists key attributes of specific USB devices.

#### Usage
```bash
./extract_usb_info.sh /dev/ttyUSB0
```
Replace `/dev/ttyUSB0` with the device you want to query.

#### Output
- `idVendor`: Device's vendor ID
- `idProduct`: Device's product ID
- `manufacturer`: Device manufacturer name
- `serial`: Device's serial number

### 3. device_rule_generate.sh

#### Description
Generates specific udev rules based on USB device characteristics, allowing the main system to recognize and assign fixed names to these USB devices.

#### Usage
1. Modify the `device_rule_generate.sh` script:
   - Add USB device characteristics (obtained from `extract_usb_info.sh`) to the designated area.
2. Execute the script with sudo:
   ```bash
   sudo ./device_rule_generate.sh
   ```
   This will assign fixed names to the USB devices and take effect immediately.

#### Example Rules
```bash
# LiDAR device rule
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{manufacturer}=="Silicon Labs", SYMLINK+="usb_ydlidar", MODE="0666"

# Arduino device rule
SUBSYSTEM=="tty", ATTRS{idVendor}=="1d6b", ATTRS{idProduct}=="0002", SYMLINK+="usb_rear_wheel", MODE="0666"
```

#### Result
![Console Output](https://github.com/alianlbj23/tools/blob/main/pic/console.png?raw=true)

### 4. rule_generator.py

#### Description
This Python script automates the process of generating udev rules for USB devices. It communicates with connected devices, retrieves their custom IDs, and creates appropriate udev rules.

#### Command Flow: rule_generator.py to Arduino/ESP32
![Command Flow](https://github.com/alianlbj23/tools/blob/main/pic/rule_generate.drawio.png?raw=true)

#### Process Flow for Flashing CUSTOM_ID and Generating USB Device Rule
![Process Flow](https://github.com/alianlbj23/tools/blob/main/pic/Use_flowchart.drawio.png?raw=true)
#### Prerequisites
- Python 3.x
- `pyserial` library (install with `pip install pyserial`)
- Arduino or ESP32 devices with proper firmware (see Arduino/ESP32 Code Requirements below)

#### Arduino/ESP32 Code Requirements
For this script to work correctly, the Arduino or ESP32 devices must include their own CUSTOM_ID and be able to respond to JSON commands. Add the following to your Arduino/ESP32 code:

```cpp
// Replace with your device's unique identifier
#define CUSTOM_ID "usb_rear_wheel"

// In your main loop or command processing function:
if (doc.containsKey("command") && doc["command"] == "I") {
    Serial.println(CUSTOM_ID);
}
```

This code allows the device to respond with its CUSTOM_ID when queried by the rule_generator.py script. The CUSTOM_ID will be used as the device's rule name in the generated udev rules.

#### Usage
1. Ensure you have the necessary permissions to write to `/etc/udev/rules.d/` and execute udev commands.
2. Make sure your Arduino/ESP32 devices are programmed with the required code (as described above).
3. Connect your devices to the system.
4. Run the script with sudo:

```bash
sudo python3 rule_generator.py
```

#### Functionality
- Automatically detects all `/dev/ttyUSB*` and `/dev/ttyACM*` devices.
- Communicates with each device to retrieve its custom ID.
- Generates udev rules based on the device's kernel information and custom ID.
- Reloads udev rules to apply changes immediately.

#### Output
- Displays the custom ID received from each device.
- Shows the matching KERNELS information.
- Confirms the creation of udev rules for each device.
- Reports any errors encountered during the process.

### 4. find_usb_kernels.sh

#### Description
Detects the specific `KERNELS` value associated with a given USB device path (e.g., `/dev/ttyUSB0`). The `KERNELS` value represents the physical port path in the system's kernel device tree. This value is useful for creating udev rules that target a specific physical USB port regardless of the device plugged into it.

#### Usage
```bash
./find_usb_kernels.sh /dev/ttyUSB0
```
Replace `/dev/ttyUSB0` with the actual device path you want to inspect.

#### Output
- The script outputs the `KERNELS` value found for the specified device (e.g., `1-1.2`, `2-3.4.1`).

### 5. set_usb_name_by_kernels.sh

#### Description
Creates a udev rule that assigns a fixed symbolic link name (e.g., `/dev/my_specific_usb`) to *any* compatible device plugged into a specific physical USB port. It identifies the port using the `KERNELS` value obtained from `find_usb_kernels.sh`. This is useful when you want a specific port to always map to the same device name, regardless of which identical device is plugged in or the order they are plugged in.

#### Usage
1.  **Find the KERNELS value:** Use `./find_usb_kernels.sh <DEVICE_PATH>` for a device currently plugged into the desired physical port to get its `KERNELS` value.
2.  **Run the script:** Execute the script with `sudo`, providing the `KERNELS` value and the desired symbolic link name.

```bash
# Example: Assign /dev/front_sensor to the port with KERNELS="1-1.2"
sudo ./set_usb_name_by_kernels.sh "1-1.2" front_sensor
```
Replace `"1-1.2"` with the actual `KERNELS` value and `front_sensor` with your desired fixed name.

#### Functionality
- Creates a udev rule file in `/etc/udev/rules.d/` (e.g., `99-front_sensor.rules`).
- The rule matches the specified `KERNELS` value and `SUBSYSTEM=="tty"`.
- Assigns the specified `SYMLINK+` name.
- Sets `MODE="0666"` for general read/write access (modify if needed).
- Reloads udev rules to apply the change. You might need to replug the device.

