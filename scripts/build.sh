#!/bin/bash
set -e

# Find the workspace root (assumes script is in src/RobotSim-ROS/scripts/)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_ROOT="$( cd "$SCRIPT_DIR/../../.." && pwd )"

echo "Building workspace at $WS_ROOT..."

cd "$WS_ROOT"
source /opt/ros/humble/setup.bash

# Clean previous build artifacts to ensure updates are applied
echo "Cleaning previous build for ur5e_unity_bridge..."
rm -rf "$WS_ROOT/build/ur5e_unity_bridge"
rm -rf "$WS_ROOT/install/ur5e_unity_bridge"

# Build the package
colcon build --symlink-install --packages-select ur5e_unity_bridge

echo "Build complete."
