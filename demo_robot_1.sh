#!/usr/bin/env bash

cd ~/ros2_ws3_1
source /opt/ros/humble/setup.bash
source ~/ros2_ws3_1/install/setup.bash

export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$IGN_GAZEBO_SYSTEM_PLUGIN_PATH:/opt/ros/humble/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/lib
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/kamel/ros2_ws3_1/src/mobile_arm_robot_description/models:/home/kamel/ros2_ws3_1/install/mobile_arm_robot_description/share

ros2 launch mobile_arm_robot_control demo_mapping.launch.py
