#!/bin/bash
# Phase 1 — Start SLAM for map scanning
# Usage: ./mission/start_slam.sh

PROS_APP="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROS_APP"

# Detect docker compose command
if command -v docker-compose &>/dev/null; then
    DC="docker-compose"
elif docker compose version &>/dev/null; then
    DC="docker compose"
else
    echo "[ERROR] Docker Compose not found."
    exit 1
fi

COMPOSE_DIR="./docker/compose"

echo "[bridge-mission] Starting rosbridge..."
$DC -f "$COMPOSE_DIR/docker-compose_rosbridge_server.yml" up -d

echo "[bridge-mission] Starting robot bringup..."
$DC -f "$COMPOSE_DIR/docker-compose_robot_unity.yml" up -d

echo "[bridge-mission] Starting SLAM..."
$DC -f "$COMPOSE_DIR/docker-compose_slam_unity.yml" up -d

echo ""
echo "============================================"
echo " SLAM started."
echo "============================================"
echo " 1. Open Unity → Press Play"
echo " 2. Open Foxglove → Connect ws://localhost:8765"
echo "    Add 3D panel → /map topic"
echo " 3. Drive the robot to scan the map:"
echo "    cd .../pros_car && ./car_control.sh"
echo "    (inside) r → ros2 run pros_car_py robot_control"
echo " 4. When map is complete, run:"
echo "    ./mission/save_map.sh"
echo "============================================"
