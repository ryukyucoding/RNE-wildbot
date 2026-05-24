#!/bin/bash
# Phase 3 — Start Nav2 navigation for bridge mission (SLAM stays running)
# Usage: ./mission/start_nav.sh

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

echo "[bridge-mission] Starting rosbridge (idempotent)..."
$DC -f "$COMPOSE_DIR/docker-compose_rosbridge_server.yml" up -d

echo "[bridge-mission] Starting robot bringup (idempotent)..."
$DC -f "$COMPOSE_DIR/docker-compose_robot_unity.yml" up -d

echo "[bridge-mission] Starting Nav2 navigation..."
$DC -f "$COMPOSE_DIR/docker-compose_navigation_unity.yml" up -d

echo ""
echo "============================================"
echo " Nav2 started. SLAM is providing localization."
echo "============================================"
echo " (No AMCL / no initial pose estimate needed —"
echo "  SLAM has been tracking the robot since Phase 1)"
echo ""
echo " 1. Unity should still be running from Phase 1."
echo "    If not: Open Unity → Press Play"
echo ""
echo " 2. Foxglove → Reconnect → ws://localhost:8765"
echo "    Verify /map and /tf are live."
echo "    Check pose: ros2 run tf2_ros tf2_echo map base_footprint"
echo ""
echo " 3. Terminal 2 — launch car_control:"
echo "    cd .../pros_car && ./car_control.sh"
echo "    (inside) r"
echo "    (inside) ros2 launch car_control_pkg bridge_nav.launch.py"
echo ""
echo " 4. Terminal 3 — trigger the mission:"
echo "    docker ps   (find pros_car container name)"
echo "    docker exec -it <container_name> bash"
echo "    (inside) source /workspaces/install/setup.bash"
echo "    (inside) ros2 action send_goal /nav_action_server action_interface/action/NavGoal \"{mode: 'Bridge_Nav'}\""
echo "============================================"
