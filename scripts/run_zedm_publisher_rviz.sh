#!/bin/bash

# 현재 스크립트의 디렉토리 경로
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HUMBLE_SETUP="/opt/ros/humble/setup.bash"

# ROS 2 Humble 확인
if [ -f "$HUMBLE_SETUP" ]; then
    echo "ROS 2 Humble found. Sourcing $HUMBLE_SETUP"
    source "$HUMBLE_SETUP"
else
    echo "Error: Neither ROS 2 Humble nor Foxy found in /opt/ros."
    exit 1
fi

ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zedm
