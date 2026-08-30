#!/usr/bin/env bash

cd ~/ros2_ws3_1 || exit 1

source /opt/ros/humble/setup.bash
source ~/ros2_ws3_1/install/setup.bash

echo "Starting camera viewer..."
echo "Topic: /robot_camera/image"

ros2 run rqt_image_view rqt_image_view /robot_camera/image
