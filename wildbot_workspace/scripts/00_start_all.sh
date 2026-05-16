#!/bin/bash
source "$(dirname "${BASH_SOURCE[0]}")/utils.sh"
main \
    "$COMPOSE_DIR/docker-compose_kros_car.yml" \
    "$COMPOSE_DIR/docker-compose_rosbridge_server.yml" \
    "$COMPOSE_DIR/docker-compose_camera_gemini.yml" \
    "$COMPOSE_DIR/docker-compose_lidar_pkg.yml" \
    "$COMPOSE_DIR/docker-compose_oradarlidar.yml"
