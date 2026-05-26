#!/bin/bash
# 在 map 座標系設定 AMCL 初始位置（Foxglove 2D Pose Estimate 的 CLI 替代）
set -euo pipefail

X="${1:--0.77}"
Y="${2:--0.12}"
YAW="${3:-0.0}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="$REPO_ROOT/docker/compose/.env"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [ -f "$ENV_FILE" ]; then
  # shellcheck disable=SC1090
  source "$ENV_FILE"
fi

if ! docker ps --format '{{.Names}}' | grep -qx wildbot; then
  echo "[set_initial_pose] wildbot 容器未運行"
  exit 1
fi

docker exec -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" wildbot bash -lc "
set +u
source /opt/ros/jazzy/setup.bash
set -u
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \"{
  header: {frame_id: map},
  pose: {
    pose: {
      position: {x: ${X}, y: ${Y}, z: 0.0},
      orientation: {z: $(python3 -c "import math; print(math.sin(${YAW}/2))"), w: $(python3 -c "import math; print(math.cos(${YAW}/2))")}
    },
    covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06]
  }
}\"
echo '[set_initial_pose] 已發送 initial pose (${X}, ${Y}, yaw=${YAW})'
sleep 3
if timeout 8 ros2 topic echo /amcl_pose --once >/dev/null 2>&1; then
  echo '[set_initial_pose] /amcl_pose OK'
else
  echo '[set_initial_pose] 警告：仍收不到 /amcl_pose（AMCL 可能尚未啟動或 odom TF 異常）'
fi
"
