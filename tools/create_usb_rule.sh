#!/bin/bash

# 預設的熱門名稱
POPULAR_NAMES=("usb_rear_wheel" "usb_front_wheel" "usb_robot_arm" "usb_lidar")

# 檢查參數
if [ "$#" -ne 2 ]; then
    echo "用法: $0 <USB_DEVICE_PATH> <USB_NAME>"
    echo "例如: $0 /dev/ttyUSB0 my_usb_rule"
    echo ""
    echo "熱門名稱 (可用作 <USB_NAME>):"
    for name in "${POPULAR_NAMES[@]}"; do
        echo "  - $name"
    done
    exit 1
fi

USB_DEVICE="$1"
USB_NAME="$2"
RULE_FILE="/etc/udev/rules.d/${USB_NAME}.rules"

# 獲取 USB 設備的 udev 信息
udevadm info --query=all --name="$USB_DEVICE" > /tmp/usb_info.txt

# 提取 Vendor ID, Product ID, Serial Number
ID_VENDOR=$(grep -oP 'ID_VENDOR_ID=\K.*' /tmp/usb_info.txt)
ID_PRODUCT=$(grep -oP 'ID_MODEL_ID=\K.*' /tmp/usb_info.txt)
ID_SERIAL=$(grep -oP 'ID_SERIAL_SHORT=\K.*' /tmp/usb_info.txt)

# 檢查是否成功獲取資訊
if [ -z "$ID_VENDOR" ] || [ -z "$ID_PRODUCT" ]; then
    echo "無法獲取 USB 設備的 Vendor ID 或 Product ID，請確認設備是否連接正確。"
    exit 1
fi

# 生成 UDEV 規則，只有在有 serial 的情況下才添加 serial 匹配條件
if [ -z "$ID_SERIAL" ]; then
    echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$ID_VENDOR\", ATTRS{idProduct}==\"$ID_PRODUCT\", MODE=\"0666\", SYMLINK+=\"$USB_NAME\"" | sudo tee "$RULE_FILE"
    echo "注意: 未檢測到 Serial Number，已創建不含 Serial Number 的規則。"
else
    echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$ID_VENDOR\", ATTRS{idProduct}==\"$ID_PRODUCT\", ATTRS{serial}==\"$ID_SERIAL\", MODE=\"0666\", SYMLINK+=\"$USB_NAME\"" | sudo tee "$RULE_FILE"
fi

# 重新載入 UDEV 規則
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo "🔥 UDEV 規則已創建: $RULE_FILE"
echo "🔌 請重新插入 USB 設備以應用新規則。"
echo ""
echo "✅ 你現在可以使用 /dev/$USB_NAME 來存取該設備！"
echo ""
echo "🔹 常用名稱 (可用於設定符號連結):"
for name in "${POPULAR_NAMES[@]}"; do
    echo "  - $name"
done