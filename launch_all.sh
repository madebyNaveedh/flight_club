#!/usr/bin/env bash
set -e

# Cleanup existing processes
fuser -k 8080/tcp 2>/dev/null || true
killall -9 ign ruby gzserver gzclient parameter_bridge image_bridge survivor_detector grid_scorer gcs_server 2>/dev/null || true
sleep 1

export IGN_IP="127.0.0.1"

WS_DIR="/home/mohammed/nidar_airmouse_ws"
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

echo "Starting NIDAR AirMouse Simulation Stack..."
ros2 launch nidar_sim simulation.launch.py
