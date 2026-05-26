#!/bin/bash
# 實車定位 + Nav2 — 先 AMCL（含 initial pose），再啟動 Nav2，避免 bt_navigator inactive
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_DIR="$REPO_ROOT/docker/compose"
MAP_YAML="$REPO_ROOT/maps/map01/map01.yaml"
ENV_FILE="$REPO_ROOT/docker/compose/.env"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
[ -f "$ENV_FILE" ] && source "$ENV_FILE"

if [ ! -f "$MAP_YAML" ]; then
  echo "[03_localization_nav] 找不到 $MAP_YAML"
  echo "請先跑 ./scripts/01_mapping.sh → ./scripts/02_save_map.sh"
  exit 1
fi

if docker compose version &>/dev/null; then
  DC="docker compose"
elif command -v docker-compose &>/dev/null; then
  DC="docker-compose"
else
  echo "[03_localization_nav] 需要 docker compose"; exit 1
fi

ros_exec() {
  local container="$1"
  shift
  docker exec -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" "$container" bash -lc "
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
    $*
  "
}

pick_ros_container() {
  if docker ps --format '{{.Names}}' | grep -qx wildbot; then
    echo wildbot
  elif docker ps --format '{{.Names}}' | grep -qx compose-localization-1; then
    echo compose-localization-1
  else
    echo ""
  fi
}

echo "[03_localization_nav] Phase 1: odom TF + lidar TF + AMCL"
for f in \
  docker-compose_odom_tf_bridge.yml \
  docker-compose_lidar_tf.yml \
  docker-compose_cmd_vel_relay.yml \
  docker-compose_localization_wildbot.yml; do
  echo "[03_localization_nav] up --force-recreate $f"
  $DC -f "$COMPOSE_DIR/$f" up -d --force-recreate
done

sleep 5
ROS_BOX="$(pick_ros_container)"
if [ -z "$ROS_BOX" ]; then
  echo "[03_localization_nav] 警告：無 wildbot / localization 容器可檢查 ROS，請手動設 initial pose 後跑："
  echo "  ./scripts/restart_navigation.sh"
else
  echo "[03_localization_nav] 等待 /map …"
  for _ in $(seq 1 20); do
    if ros_exec "$ROS_BOX" 'timeout 3 ros2 topic echo /map --once >/dev/null 2>&1'; then
      break
    fi
    sleep 1
  done

  echo "[03_localization_nav] 請設 initial pose（Foxglove 2D Pose Estimate 或 ./scripts/set_initial_pose.sh）"
  echo "[03_localization_nav] 等待 /amcl_pose（最多 90s）…"
  AMCL_OK=0
  for _ in $(seq 1 90); do
    if ros_exec "$ROS_BOX" 'timeout 2 ros2 topic echo /amcl_pose --once >/dev/null 2>&1'; then
      AMCL_OK=1
      break
    fi
    sleep 1
  done
  if [ "$AMCL_OK" -eq 0 ]; then
    echo "[03_localization_nav] 警告：90s 內仍無 /amcl_pose，Nav2 可能無法 activate"
    echo "[03_localization_nav] 設好 pose 後請跑：./scripts/restart_navigation.sh"
  else
    echo "[03_localization_nav] /amcl_pose OK"
  fi
fi

echo "[03_localization_nav] Phase 2: Nav2 navigation（須在 AMCL 有 map TF 後啟動）"
$DC -f "$COMPOSE_DIR/docker-compose_navigation_wildbot.yml" up -d --force-recreate
sleep 8

if [ -n "$ROS_BOX" ]; then
  NAV_STATE="$(ros_exec "$ROS_BOX" 'ros2 lifecycle get /bt_navigator 2>/dev/null | tail -1' || true)"
  if echo "$NAV_STATE" | grep -qi active; then
    echo "[03_localization_nav] bt_navigator: active ✓"
  else
    echo "[03_localization_nav] 警告：bt_navigator 仍 inactive（$NAV_STATE）"
    echo "[03_localization_nav] 請確認 initial pose 後跑：./scripts/restart_navigation.sh"
  fi
fi

echo "[03_localization_nav] 驗證：./scripts/verify_nav.sh"
echo "[03_localization_nav] follow logs（Ctrl+C 只停 log）"
trap 'exit 0' SIGINT
$DC -f "$COMPOSE_DIR/docker-compose_localization_wildbot.yml" logs -f &
$DC -f "$COMPOSE_DIR/docker-compose_navigation_wildbot.yml" logs -f &
wait
