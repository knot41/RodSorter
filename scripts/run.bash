#!/bin/bash
source /opt/ros/humble/setup.bash
source ./install/setup.bash

cd "$(dirname "$0")/.."

# bash scripts/debug.bash &
echo "Launching auto_aim"
ros2 launch tf_tree run.launch.py

wait