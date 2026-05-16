#!/bin/bash

# 檢查參數數量
if [ "$#" -ne 2 ]; then
    echo "用法: $0 <KERNELS_VALUE> <SYMLINK_NAME>"
    echo "例如: $0 \"1-1.2\" my_specific_usb"
    echo ""
    echo "說明:"
    echo "  <KERNELS_VALUE>: USB 埠的 KERNELS 值 (例如 \"1-1.2\", \"2-3\")。"
    echo "                   你可以使用 ./find_usb_kernels.sh <DEVICE_PATH> 來找到這個值。"
    echo "  <SYMLINK_NAME>:  你想要為這個 USB 埠上的設備指定的名稱 (例如 my_sensor, rear_lidar)。"
    echo "                   設備將會出現在 /dev/<SYMLINK_NAME>。"
    exit 1
fi

KERNELS_VALUE="$1"
SYMLINK_NAME="$2"
RULE_PRIORITY="99" # 規則的優先級，數字越大越晚執行
RULE_FILE="/etc/udev/rules.d/${RULE_PRIORITY}-${SYMLINK_NAME}.rules"

# 檢查 SYMLINK_NAME 是否為空
if [ -z "$SYMLINK_NAME" ]; then
    echo "錯誤: SYMLINK_NAME 不可為空。"
    exit 1
fi

# 檢查 KERNELS_VALUE 是否為空
if [ -z "$KERNELS_VALUE" ]; then
    echo "錯誤: KERNELS_VALUE 不可為空。"
    exit 1
fi


echo "正在為 KERNELS=\"$KERNELS_VALUE\" 的 USB 埠創建規則..."
echo "設備將會被命名為 /dev/$SYMLINK_NAME"

# 生成 UDEV 規則內容
# 注意：這裡假設是 tty 設備 (如 USB 轉串口)。
# 如果是其他類型的 USB 設備，你可能需要調整 SUBSYSTEM。
# 同時，我們添加 MODE="0666" 讓所有使用者都有讀寫權限，如果需要更嚴格的權限請修改。
RULE_CONTENT="SUBSYSTEM==\"tty\", KERNELS==\"$KERNELS_VALUE\", MODE=\"0666\", SYMLINK+=\"$SYMLINK_NAME\""

# 寫入 UDEV 規則文件 (需要 root 權限)
echo "正在寫入規則到 $RULE_FILE ..."
echo "$RULE_CONTENT" | sudo tee "$RULE_FILE"

if [ $? -ne 0 ]; then
    echo "錯誤: 無法寫入規則文件。請確認您有 sudo 權限。"
    exit 1
fi

# 重新載入 UDEV 規則 (需要 root 權限)
echo "正在重新載入 udev 規則..."
sudo udevadm control --reload-rules && sudo udevadm trigger

if [ $? -ne 0 ]; then
    echo "錯誤: 無法重新載入 udev 規則。"
    exit 1
fi

echo ""
echo "✅ UDEV 規則已成功創建於: $RULE_FILE"
echo "🔌 請重新插入連接到 KERNELS=\"$KERNELS_VALUE\" 埠的 USB 設備。"
echo "   或者，如果設備已經插入，它應該很快會出現在 /dev/$SYMLINK_NAME"
echo ""
echo "➡️ 你現在可以使用 /dev/$SYMLINK_NAME 來存取該設備了！"

exit 0