#!/bin/bash
# This script builds and runs the ur5e_unity_bridge package.

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Step 0: Deep cleaning ROS 2 and stale processes..."
# Stop ROS 2 daemon to clear DDS cache
ros2 daemon stop || true

# Kill all possible ROS/Bridge related processes
pkill -9 -f bridge_node || true
pkill -9 -f moveit_bridge || true
pkill -9 -f rviz2 || true
pkill -9 -f move_group || true
pkill -9 -f rosbridge_websocket || true
pkill -9 -f robot_state_publisher || true
pkill -9 -f ros2_control_node || true
pkill -9 -f controller_manager || true
pkill -9 -f spawner || true

# Force clear any remaining shared memory (if using FastDDS)
rm -rf ~/.ros/log/*
sleep 2

echo "Step 1: Building..."
$SCRIPT_DIR/scripts/build.sh

echo ""
echo "Step 2: Running..."
$SCRIPT_DIR/scripts/run.sh "$@"
