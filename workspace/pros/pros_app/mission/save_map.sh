#!/bin/bash
# Phase 1 finish — Save map (SLAM keeps running)
# Usage: ./mission/save_map.sh

PROS_APP="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROS_APP"

echo "[bridge-mission] Saving map..."
./store_map.sh

echo ""
echo "============================================"
echo " Map saved. SLAM is still running."
echo "============================================"
echo " Next steps:"
echo " 1. Read bridge coordinates from Foxglove:"
echo "    (Publish point tool → click Point A & B)"
echo "    ros2 topic echo /clicked_point"
echo ""
echo " 2. Compute heading:"
echo "    python3 -c \"import math; print(math.degrees(math.atan2(By-Ay, Bx-Ax)))\""
echo ""
echo " 3. Edit bridge_params.yaml:"
echo "    .../pros_car/src/car_control_pkg/launch/bridge_params.yaml"
echo ""
echo " 4. When ready, run (starts Nav2 only — SLAM already provides localization):"
echo "    ./mission/start_nav.sh"
echo "============================================"
