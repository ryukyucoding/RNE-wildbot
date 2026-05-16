#!/bin/bash

# 檢查是否有參數傳入
if [ "$#" -ne 1 ]; then
    echo "錯誤：未提供設備名稱作為參數。"
    echo "使用方式："
    echo "  $0 /dev/ttyUSBx"
    echo ""
    echo "示例："
    echo "  $0 /dev/ttyUSB0"
    echo "  使用此命令來查看 /dev/ttyUSB0 的設備屬性資訊。"
    echo ""
    echo "說明："
    echo "  本腳本用於列出指定 USB 設備的父級設備屬性，包括 idVendor、idProduct、manufacturer 和 serial 等資訊。"
    echo "  請確保提供正確的設備名稱，例如 /dev/ttyUSB0、/dev/ttyUSB1 等。"
    exit 1
fi

DEVICE="$1"

# 使用 udevadm 獲取設備資訊
INFO=$(udevadm info --name=${DEVICE} --attribute-walk)

# 遍歷每個 `looking at parent` 區塊
echo "$INFO" | awk '/looking at parent/{flag=1; print; next} /^$/{flag=0} flag' | while read -r LINE; do
    # 檢查並提取需要的屬性
    if echo "$LINE" | grep -q 'ATTRS{idVendor}'; then
        ID_VENDOR=$(echo "$LINE" | awk -F\" '{print $2}')
        echo "  idVendor: $ID_VENDOR"
    fi
    if echo "$LINE" | grep -q 'ATTRS{idProduct}'; then
        ID_PRODUCT=$(echo "$LINE" | awk -F\" '{print $2}')
        echo "  idProduct: $ID_PRODUCT"
    fi
    if echo "$LINE" | grep -q 'ATTRS{manufacturer}'; then
        MANUFACTURER=$(echo "$LINE" | awk -F\" '{print $2}')
        echo "  manufacturer: $MANUFACTURER"
    fi
    if echo "$LINE" | grep -q 'ATTRS{serial}'; then
        SERIAL=$(echo "$LINE" | awk -F\" '{print $2}')
        echo "  serial: $SERIAL"
    fi
done
