#!/usr/bin/env bash

cd "$HOME/ros2_ws3_1" || exit 1

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws3_1/install/setup.bash"

exec ros2 launch \
mobile_arm_robot_autonomy \
autonomy_robot_1.launch.py
