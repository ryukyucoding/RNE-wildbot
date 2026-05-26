#!/bin/bash
# 停止建圖或定位 stack（不影響 00_start_all 的車體/相機/LiDAR）
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

FILES=(
  docker-compose_slam_wildbot.yml
  docker-compose_localization_wildbot.yml
  docker-compose_navigation_wildbot.yml
  docker-compose_odom_tf_bridge.yml
  docker-compose_lidar_tf.yml
  docker-compose_scan_matcher.yml
  docker-compose_cmd_vel_relay.yml
  docker-compose_store_map_wildbot.yml
)

for f in "${FILES[@]}"; do
  path="$COMPOSE_DIR/$f"
  if [ -f "$path" ]; then
    echo "[stop_nav_stack] down $f"
    $DC -f "$path" down --timeout 0 2>/dev/null || true
  fi
done

echo "[stop_nav_stack] 完成"
