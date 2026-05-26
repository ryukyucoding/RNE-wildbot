#!/bin/bash
# 實車建圖（SLAM）— 需先執行 ./scripts/00_start_all.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_DIR="$REPO_ROOT/docker/compose"

if docker compose version &>/dev/null; then
  DC="docker compose"
elif command -v docker-compose &>/dev/null; then
  DC="docker-compose"
else
  echo "需要 docker compose"; exit 1
fi

if ! docker network inspect compose_my_bridge_network &>/dev/null; then
  echo "[01_mapping] 警告：compose_my_bridge_network 不存在，請先跑 ./scripts/00_start_all.sh"
fi

echo "[01_mapping] 啟動 odom TF bridge + lidar TF + SLAM（Ctrl+C 停止）"
echo "[01_mapping] Foxglove：加「Map」面板（不是 3D）→ Topic=/map，Fixed frame=map"
echo "[01_mapping] 建圖時請慢速移動；存圖請另開 terminal 跑 ./scripts/02_save_map.sh"
echo "[01_mapping] ⚠️  不要 Ctrl+C 本 terminal（會把 SLAM 停掉）"

cleanup() {
  echo "[01_mapping] 停止建圖 stack …"
  $DC -f "$COMPOSE_DIR/docker-compose_slam_wildbot.yml" down --timeout 0 || true
  $DC -f "$COMPOSE_DIR/docker-compose_lidar_tf.yml" down --timeout 0 || true
  $DC -f "$COMPOSE_DIR/docker-compose_odom_tf_bridge.yml" down --timeout 0 || true
  exit 0
}
trap cleanup SIGINT SIGTERM

echo "[01_mapping] 啟動 odom TF + lidar TF …"
$DC -f "$COMPOSE_DIR/docker-compose_odom_tf_bridge.yml" up -d
$DC -f "$COMPOSE_DIR/docker-compose_lidar_tf.yml" up -d

echo "[01_mapping] 等待 /scan 固定長度 …"
if docker ps --format '{{.Names}}' | grep -qx 'compose-lidar_pkg-1'; then
  docker exec compose-lidar_pkg-1 bash -lc '
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=0
    python3 - <<'"'"'PY'"'"'
import sys, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
node = Node("scan_wait")
lengths = []

def cb(msg):
    lengths.append(len(msg.ranges))

node.create_subscription(LaserScan, "/scan", cb, qos_profile_sensor_data)
deadline = time.time() + 25.0
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
    if len(lengths) >= 12 and len(set(lengths[-8:])) == 1:
        print(f"[01_mapping] /scan 穩定 length={lengths[-1]}")
        break
else:
    print("[01_mapping] 警告：/scan 尚未穩定", sorted(set(lengths)) if lengths else "no data", file=sys.stderr)
node.destroy_node()
rclpy.shutdown()
PY
  ' || true
else
  echo "[01_mapping] 警告：compose-lidar_pkg-1 未運行，請先 ./scripts/00_start_all.sh"
fi

echo "[01_mapping] 啟動 SLAM …"
$DC -f "$COMPOSE_DIR/docker-compose_slam_wildbot.yml" up -d

echo "[01_mapping] 開始追 log（慢速開車後 Map 面板應出現灰白障礙圖）…"
$DC -f "$COMPOSE_DIR/docker-compose_odom_tf_bridge.yml" logs -f &
$DC -f "$COMPOSE_DIR/docker-compose_lidar_tf.yml" logs -f &
$DC -f "$COMPOSE_DIR/docker-compose_slam_wildbot.yml" logs -f &
wait
