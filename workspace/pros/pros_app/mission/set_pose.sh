#!/bin/bash
# Auto-set AMCL initial pose from current odometry.
# Run this AFTER start_nav.sh when AMCL has no initial pose.
# Usage: ./mission/set_pose.sh
#
# Must be run inside the car_control container or any container
# that can reach the ROS network (same ROS_DOMAIN_ID).

echo "[bridge-mission] Reading current position from /odom..."

# Read one odom message and extract x, y
ODOM=$(ros2 topic echo /odom --once 2>/dev/null)

if [ -z "$ODOM" ]; then
    echo "[ERROR] Could not read /odom. Is the robot bringup running?"
    exit 1
fi

X=$(echo "$ODOM" | grep -A3 "position:" | grep "x:" | head -1 | awk '{print $2}')
Y=$(echo "$ODOM" | grep -A3 "position:" | grep "y:" | head -1 | awk '{print $2}')

if [ -z "$X" ] || [ -z "$Y" ]; then
    echo "[ERROR] Could not parse odom position."
    echo "Raw odom output:"
    echo "$ODOM"
    exit 1
fi

echo "[bridge-mission] Robot position from odom: x=$X  y=$Y"
echo "[bridge-mission] Publishing initial pose to AMCL..."

ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" \
  --once

echo ""
echo "[bridge-mission] Waiting for AMCL to publish pose..."
sleep 2

AMCL=$(ros2 topic echo /amcl_pose --once 2>/dev/null)
if [ -n "$AMCL" ]; then
    echo "✓ AMCL is now localized."
else
    echo "⚠ AMCL not responding yet. Try running this script again in a few seconds."
    echo "  Or in Foxglove: 3D Panel → Publish pose estimate → click robot location"
fi
