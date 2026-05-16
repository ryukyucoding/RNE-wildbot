#!/bin/bash

# 檢查參數
if [ "$#" -ne 1 ]; then
    echo "用法: $0 <USB_DEVICE_PATH>"
    echo "例如: $0 /dev/ttyUSB0"
    exit 1
fi

USB_DEVICE="$1"

# 檢查設備是否存在
if [ ! -e "$USB_DEVICE" ]; then
    echo "錯誤: 設備 $USB_DEVICE 不存在。"
    exit 1
fi

echo "正在偵測 $USB_DEVICE 的 KERNELS 值..."

# 獲取設備的 sysfs 路徑
UDEV_PATH=$(udevadm info -q path -n "$USB_DEVICE")

if [ -z "$UDEV_PATH" ]; then
    echo "錯誤: 無法獲取 $USB_DEVICE 的 udev 路徑。"
    exit 1
fi

# 獲取 udev 屬性並提取 KERNELS 值
KERNELS_VALUE=$(udevadm info -a -p "$UDEV_PATH" | grep -oP '^\s*KERNELS=="\K[^"]*')

if [ -n "$KERNELS_VALUE" ]; then
    echo "找到的 KERNELS 值: $KERNELS_VALUE"
else
    echo "錯誤: 無法找到 $USB_DEVICE 的 KERNELS 值。"
    echo "請確認 $USB_DEVICE 是正確的設備路徑。"
fi

exit 0