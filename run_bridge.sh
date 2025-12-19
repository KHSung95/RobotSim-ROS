#!/bin/bash
# This script builds and runs the ur5e_unity_bridge package.

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Step 0: Cleaning up previous processes..."
pkill -f bridge_node || true
pkill -f rviz2 || true
pkill -f ros2_control_node || true
pkill -f robot_state_publisher || true
pkill -f move_group || true
pkill -f rosbridge_websocket || true
sleep 2

echo "Step 1: Building..."
$SCRIPT_DIR/scripts/build.sh

echo ""
echo "Step 2: Running..."
$SCRIPT_DIR/scripts/run.sh "$@"
