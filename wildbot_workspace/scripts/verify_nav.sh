#!/bin/bash
# 檢查 AMCL / Nav2 / scan / map 是否就緒（在 host 執行，會 docker exec 進 wildbot）
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="$REPO_ROOT/docker/compose/.env"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [ -f "$ENV_FILE" ]; then
  # shellcheck disable=SC1090
  source "$ENV_FILE"
fi

if ! docker ps --format '{{.Names}}' | grep -qx wildbot; then
  echo "[verify_nav] wildbot 容器未運行，請先 ./launch_shell.sh"
  exit 1
fi

docker exec -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" wildbot bash -lc '
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
set -u

check_hz() {
  local topic="$1"
  local label="$2"
  if timeout 8 ros2 topic echo "$topic" --once >/dev/null 2>&1; then
    echo "  OK  $label ($topic)"
    return 0
  fi
  echo "  FAIL $label ($topic)"
  return 1
}

check_once() {
  local topic="$1"
  local label="$2"
  if timeout 5 ros2 topic echo "$topic" --once >/dev/null 2>&1; then
    echo "  OK  $label ($topic)"
    return 0
  fi
  echo "  FAIL $label ($topic)"
  return 1
}

echo "=== Wildbot Nav 檢查 (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}) ==="
FAIL=0

check_hz /scan "LiDAR scan" || FAIL=1
check_hz /odom "odom TF bridge" || FAIL=1
check_once /amcl_pose "AMCL pose" || FAIL=1
check_once /map "map server" || FAIL=1

if ros2 action list 2>/dev/null | grep -q navigate_to_pose; then
  echo "  OK  Nav2 navigate_to_pose action"
else
  echo "  FAIL Nav2 navigate_to_pose action"
  FAIL=1
fi

BT_STATE=$(ros2 lifecycle get /bt_navigator 2>/dev/null | tail -1 || true)
if echo "$BT_STATE" | grep -qi "active"; then
  echo "  OK  bt_navigator lifecycle active"
else
  echo "  FAIL bt_navigator lifecycle ($BT_STATE)"
  FAIL=1
fi

if [ "$FAIL" -eq 0 ]; then
  echo "=== 全部通過：可以跑 bear_mission（建議 amcl_wait_timeout_sec>=30.0）==="
else
  echo "=== 有項目失敗 ==="
  echo "提示："
  echo "  - 沒 /amcl_pose：先跑 ./scripts/set_initial_pose.sh 或在 Foxglove 設 2D Pose Estimate"
  echo "    docker logs compose-localization-1 2>&1 | tail -20"
  echo "    若看到 symbol lookup error / BadParamException：重建 wildbot_workspace 映像"
  echo "  - 沒 /odom：odom_tf_bridge 有跑嗎？/base_controller/odom 有資料嗎？"
  echo "  - 沒 navigate_to_pose / bt_navigator inactive："
  echo "    Nav2 在 AMCL 設 initial pose **之前** 啟動會永遠 reject goal"
  echo "    解法：./scripts/set_initial_pose.sh → ./scripts/restart_navigation.sh"
  echo "    docker logs compose-navigation-1 2>&1 | grep -i inactive"
  exit 1
fi
'
