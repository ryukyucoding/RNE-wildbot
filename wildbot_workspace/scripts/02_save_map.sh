#!/bin/bash
# 將 SLAM 即時地圖存成 maps/map01/（需 01_mapping.sh 仍在跑）
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_DIR="$REPO_ROOT/maps/map01"
SLAM_CONTAINER="${SLAM_CONTAINER:-compose-slam-1}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

mkdir -p "$MAP_DIR"

if ! docker ps --format '{{.Names}}' | grep -qx "$SLAM_CONTAINER"; then
  echo "[02_save_map] 失敗：找不到正在跑的 SLAM 容器 '$SLAM_CONTAINER'"
  echo "請確認 ./scripts/01_mapping.sh 仍在執行（不要先跑 stop_nav_stack.sh）"
  exit 1
fi

_map_wait_ready() {
  local container="$1"
  local wait_sec="${2:-45}"
  docker exec "$container" bash -lc "
    set -e
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
    deadline=\$(( \$(date +%s) + ${wait_sec} ))
    while [ \$(date +%s) -lt \$deadline ]; do
      if ros2 topic type /map >/dev/null 2>&1; then
        if timeout 8 ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once \
          --qos-reliability reliable \
          --qos-durability transient_local >/dev/null 2>&1; then
          exit 0
        fi
      fi
      sleep 2
    done
    exit 1
  "
}

_map_diag() {
  local container="$1"
  docker exec "$container" bash -lc "
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
    echo '--- ros2 topic list (map/scan) ---'
    ros2 topic list 2>/dev/null | grep -E '^/(map|scan|tf)' || true
    echo '--- ros2 topic type /map ---'
    ros2 topic type /map 2>&1 || true
    echo '--- ros2 node list (slam) ---'
    ros2 node list 2>/dev/null | grep -i slam || true
  " 2>/dev/null || true
}

echo "[02_save_map] 在 $SLAM_CONTAINER 內存圖 → $MAP_DIR ..."
echo "[02_save_map] 等待 /map 就緒（最多 45 秒；01_mapping 須保持運行）…"

if ! _map_wait_ready "$SLAM_CONTAINER" 45; then
  echo "[02_save_map] 失敗：SLAM 容器內收不到 /map"
  echo "  常見原因："
  echo "  1) 01_mapping 尚未啟動，或已 Ctrl+C / stop_nav_stack 停掉 SLAM"
  echo "  2) SLAM 剛啟動，slam_toolbox 尚未發布 /map（請等 10–30 秒再試）"
  echo "  3) 車子還沒慢速移動，地圖尚未建立（Foxglove Map 面板仍空白）"
  echo "  正確順序：01_mapping 保持運行 → 慢速繞場 → Foxglove 看到地圖 → 本腳本存圖"
  _map_diag "$SLAM_CONTAINER"
  exit 1
fi

docker exec "$SLAM_CONTAINER" bash -lc "
  set -e
  source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
  ros2 run nav2_map_server map_saver_cli -f /workspace/maps/map01/map01 --ros-args \
    -p map_subscribe_transient_local:=true \
    -p save_map_timeout:=15.0
"

if [ ! -f "$MAP_DIR/map01.yaml" ] || [ ! -f "$MAP_DIR/map01.pgm" ]; then
  echo "[02_save_map] 失敗：找不到 map01.yaml / map01.pgm"
  echo "  請確認 docker volume 已掛載：maps/map01 → /workspace/maps/map01"
  exit 1
fi

echo "[02_save_map] 成功！"
echo "  - $MAP_DIR/map01.yaml"
echo "  - $MAP_DIR/map01.pgm"
echo "[02_save_map] 下一步：Ctrl+C 停 01_mapping → ./scripts/stop_nav_stack.sh"
