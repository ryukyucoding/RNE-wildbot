#!/bin/bash
# 讀取目前 AMCL 位置，印出下次用的 set_initial_pose.sh 指令
# 用法：./scripts/save_current_pose.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="$REPO_ROOT/docker/compose/.env"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
[ -f "$ENV_FILE" ] && source "$ENV_FILE"

if ! docker ps --format '{{.Names}}' | grep -qx wildbot; then
  echo "[save_pose] wildbot 容器未運行"; exit 1
fi

echo "[save_pose] 讀取 /amcl_pose …"
POSE=$(docker exec -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" wildbot bash -lc "
  set +u; source /opt/ros/jazzy/setup.bash; set -u
  timeout 8 ros2 topic echo /amcl_pose --once 2>/dev/null
")

if [ -z "$POSE" ]; then
  echo "[save_pose] 無法讀取 /amcl_pose，請先設 initial pose 並確認 AMCL active"
  exit 1
fi

X=$(echo  "$POSE" | awk '/position:/{p=1}  p && /x:/{print $2; exit}')
Y=$(echo  "$POSE" | awk '/position:/{p=1}  p && /y:/{print $2; exit}')
QZ=$(echo "$POSE" | awk '/orientation:/{p=1} p && /z:/{print $2; exit}')
QW=$(echo "$POSE" | awk '/orientation:/{p=1} p && /w:/{print $2; exit}')
YAW=$(python3 -c "import math; print(round(2*math.atan2($QZ,$QW), 4))")

echo ""
echo "目前位置：x=$X  y=$Y  yaw=$YAW"
echo ""
echo "下次設 initial pose 用這行："
echo "  ./scripts/set_initial_pose.sh $X $Y $YAW"
echo ""

# 直接更新 COMMANDS.md 裡的 Step 4 座標
CMDS="$REPO_ROOT/../COMMANDS.md"
if [ -f "$CMDS" ]; then
  # 只更新 Step 4 那一行（夾在 cd ~/RNE/wildbot_workspace 和 # 若位置 之間）
  OLD=$(grep "set_initial_pose.sh" "$CMDS" | grep -v "save_current\|#\|記錄\|Goal\|AMCL\|解法\|nav stack" | head -1 | xargs)
  if [ -n "$OLD" ]; then
    sed -i "s|$OLD|./scripts/set_initial_pose.sh $X $Y $YAW|" "$CMDS"
    echo "[save_pose] COMMANDS.md Step 4 已更新"
  fi
fi
