#!/bin/bash
# Nav2 啟動時若沒有 map TF 會卡在 inactive；設好 initial pose 後用此腳本重啟 navigation
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_DIR="$REPO_ROOT/docker/compose"

if docker compose version &>/dev/null; then
  DC="docker compose"
else
  DC="docker-compose"
fi

echo "[restart_navigation] 重啟 Nav2 navigation 容器 …"
$DC -f "$COMPOSE_DIR/docker-compose_navigation_wildbot.yml" up -d --force-recreate
sleep 10

if docker ps --format '{{.Names}}' | grep -qx wildbot; then
  # shellcheck disable=SC1091
  source "$REPO_ROOT/docker/compose/.env" 2>/dev/null || true
  STATE=$(docker exec -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" wildbot bash -lc \
    'set +u; source /opt/ros/jazzy/setup.bash; ros2 lifecycle get /bt_navigator 2>/dev/null | tail -1' || true)
  echo "[restart_navigation] bt_navigator: ${STATE:-unknown}"
  if echo "$STATE" | grep -qi active; then
    echo "[restart_navigation] OK — NavigateToPose 應可接受 goal"
  else
    echo "[restart_navigation] 仍 inactive — 請先 ./scripts/set_initial_pose.sh 再重跑本腳本"
    docker logs compose-navigation-1 2>&1 | tail -15
    exit 1
  fi
else
  echo "[restart_navigation] 完成（無 wildbot 容器，請手動 ./scripts/verify_nav.sh）"
fi
