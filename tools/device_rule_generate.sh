#!/bin/bash

# 定義規則檔案的路徑
RULES_FILE="/etc/udev/rules.d/99-usb_devices.rules"

# 檢查是否以 root 權限執行
if [[ $EUID -ne 0 ]]; then
   echo "請使用 root 權限執行此腳本 (sudo)。"
   exit 1
fi

# 創建或覆蓋 udev 規則檔案
echo "創建或更新 $RULES_FILE..."

cat <<EOL > $RULES_FILE

# 以下開放新增---------------------

# LiDAR 設備規則
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{manufacturer}=="Silicon Labs", ATTRS{serial}=="881236a7a3239c42ad0b7f9f850a44a0",SYMLINK+="usb_ydlidar", MODE="0666"

# Arduino 設備規則
SUBSYSTEM=="tty", ATTRS{idVendor}=="1d6b", ATTRS{idProduct}=="0002", ATTRS{serial}=="0001", SYMLINK+="usb_rear_wheel", MODE="0666"

#--------------------------------

EOL

# 重新載入 udev 規則並觸發
echo "重新載入 udev 規則..."
udevadm control --reload-rules
echo "觸發 udev 規則..."
udevadm trigger

echo "完成。"
echo "已經為 LiDAR 設備創建規則 usb_ydlidar，並為 Arduino 設備創建規則 usb_rear_wheel。"

