#!/bin/bash
# Stop all bridge mission containers (SLAM and/or Navigation)
# Usage: ./mission/stop_all.sh

PROS_APP="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROS_APP"

if command -v docker-compose &>/dev/null; then
    DC="docker-compose"
elif docker compose version &>/dev/null; then
    DC="docker compose"
else
    echo "[ERROR] Docker Compose not found."
    exit 1
fi

COMPOSE_DIR="./docker/compose"

echo "[bridge-mission] Stopping all containers..."

$DC -f "$COMPOSE_DIR/docker-compose_navigation_unity.yml"  down --timeout 0 2>/dev/null
$DC -f "$COMPOSE_DIR/docker-compose_localization_unity.yml" down --timeout 0 2>/dev/null
$DC -f "$COMPOSE_DIR/docker-compose_slam_unity.yml"        down --timeout 0 2>/dev/null
$DC -f "$COMPOSE_DIR/docker-compose_robot_unity.yml"       down --timeout 0 2>/dev/null
$DC -f "$COMPOSE_DIR/docker-compose_rosbridge_server.yml"  down --timeout 0 2>/dev/null

echo ""
echo "✓ All mission containers stopped."
