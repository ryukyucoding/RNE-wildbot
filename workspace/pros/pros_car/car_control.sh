#!/bin/bash

# 須與 pros_app localization / navigation 的 Docker bridge **同名**，否則收不到 /amcl_pose。
# 請在主機執行：docker network ls | grep my_bridge
# 常見：pros_app_my_bridge_network（在 pros_app 目錄跑 compose）或 compose_my_bridge_network。
: "${ROS_BRIDGE_NETWORK:=compose_my_bridge_network}"

case "${ROS_BRIDGE_NETWORK}" in
    /*)
        echo "[car_control] 錯誤：ROS_BRIDGE_NETWORK='${ROS_BRIDGE_NETWORK}' 以 '/' 開頭。"
        echo "這是 ROS **話題**寫法，不是 Docker **網路**名稱。"
        echo "請執行：docker network ls | grep bridge"
        echo "再設例如：export ROS_BRIDGE_NETWORK=compose_my_bridge_network"
        exit 1
        ;;
esac

# 若主機已 export ROS_DOMAIN_ID，強制傳進容器（須與 yolo／localization 容器一致）。
EXTRA_DOCKER_ENV=""
if [ -n "${ROS_DOMAIN_ID+x}" ] && [ -n "${ROS_DOMAIN_ID}" ]; then
    EXTRA_DOCKER_ENV="-e ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
    echo "[car_control] Will pass ${EXTRA_DOCKER_ENV} into container (must match yolo & localization)."
fi

# 1. 統一管理 -v 參數
VOLUME_ARGS="-v $(pwd)/src:/workspaces/src -v $(pwd)/launch:/workspaces/launch"

echo "[car_control] Docker bridge network: ${ROS_BRIDGE_NETWORK} (override: ROS_BRIDGE_NETWORK=... $0)"

# Port mapping check
PORT_MAPPING=""
if [ "$1" = "--port" ] && [ -n "$2" ] && [ -n "$3" ]; then
    PORT_MAPPING="-p $2:$3"
    shift 3  # Remove the first three arguments
fi

# 檢查系統架構與作業系統
ARCH=$(uname -m)
OS=$(uname -s)

# 初始化 GPU 相關變數
GPU_FLAGS=""
USE_GPU=false

# 檢查是否為 Linux 並且支援 NVIDIA GPU
if [ "$OS" = "Linux" ]; then
    if [ -f "/etc/nv_tegra_release" ]; then
        GPU_FLAGS="--runtime=nvidia"
        USE_GPU=true
    elif docker info --format '{{json .}}' | grep -q '"Runtimes".*nvidia'; then
        GPU_FLAGS="--gpus all"
        USE_GPU=true
    fi
fi

# 測試 GPU 是否可用
if [ "$USE_GPU" = true ]; then
    echo "Testing Docker run with GPU..."
    docker run --rm $GPU_FLAGS ghcr.io/screamlab/pros_car_docker_image:latest /bin/bash -c "echo GPU test" > /dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "GPU not supported or failed, disabling GPU flags."
        GPU_FLAGS=""
        USE_GPU=false
    fi
fi

echo "Detected OS: $OS, Architecture: $ARCH"
echo "GPU Flags: $GPU_FLAGS"

# 設定適當的 Docker 參數
device_options=""

# 檢查設備並加入 --device 參數
if [ -e /dev/usb_front_wheel ]; then
    device_options+=" --device=/dev/usb_front_wheel"
fi
if [ -e /dev/usb_rear_wheel ]; then
    device_options+=" --device=/dev/usb_rear_wheel"
fi
if [ -e /dev/usb_robot_arm ]; then
    device_options+=" --device=/dev/usb_robot_arm"
fi

# 根據不同架構選擇適當的 Docker 圖像
if [ "$ARCH" = "aarch64" ]; then
    echo "Detected architecture: arm64"
    docker run -it --rm \
        --network "${ROS_BRIDGE_NETWORK}" \
        $PORT_MAPPING \
        $device_options \
        --runtime=nvidia \
        --env-file .env \
        $EXTRA_DOCKER_ENV \
        -v "$(pwd)/src:/workspaces/src" \
        ghcr.io/screamlab/pros_car_docker_image:latest \
        /bin/bash

elif [ "$ARCH" = "x86_64" ] || ([ "$ARCH" = "arm64" ] && [ "$OS" = "Darwin" ]); then
    echo "Detected architecture: amd64 or macOS arm64"

    if [ "$OS" = "Darwin" ]; then
        echo "Running Docker on macOS (without GPU support)..."
        docker run -it --rm \
            --network "${ROS_BRIDGE_NETWORK}" \
            $PORT_MAPPING \
            $device_options \
            --env-file .env \
            $EXTRA_DOCKER_ENV \
            $VOLUME_ARGS \
            ghcr.io/screamlab/pros_car_docker_image:latest \
            /bin/bash
    else
        echo "Trying to run with GPU support..."
        docker run -it --rm \
            --network "${ROS_BRIDGE_NETWORK}" \
            $PORT_MAPPING \
            $GPU_FLAGS \
            $device_options \
            --env-file .env \
            $EXTRA_DOCKER_ENV \
            $VOLUME_ARGS \
            ghcr.io/screamlab/pros_car_docker_image:latest \
            /bin/bash

        # 如果 GPU 啟動失敗，回退到 CPU 模式
        if [ $? -ne 0 ]; then
            echo "GPU not supported or failed, falling back to CPU mode..."
            docker run -it --rm \
                --network "${ROS_BRIDGE_NETWORK}" \
                $PORT_MAPPING \
                --env-file .env \
                $EXTRA_DOCKER_ENV \
                $device_options \
                $VOLUME_ARGS \
                ghcr.io/screamlab/pros_car_docker_image:latest \
                /bin/bash
        fi
    fi
else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi
