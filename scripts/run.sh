#!/bin/bash
set -e

# Find the workspace root (assumes script is in src/RobotSim-ROS/scripts/)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_ROOT="$( cd "$SCRIPT_DIR/../../.." && pwd )"

source /opt/ros/humble/setup.bash

if [ -f "$WS_ROOT/install/setup.bash" ]; then
    source "$WS_ROOT/install/setup.bash"
else
    echo "Error: install/setup.bash not found. Please run build.sh first."
    exit 1
fi

echo "Launching ur5e_unity_bridge..."
ros2 launch ur5e_unity_bridge bringup.launch.py "$@"
