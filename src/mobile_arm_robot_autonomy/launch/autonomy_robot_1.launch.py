#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    arm_sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='arm_front_lidar_bridge',
        arguments=[
            '/arm_front_lidar'
            '@sensor_msgs/msg/LaserScan'
            '[ignition.msgs.LaserScan'
        ],
        output='screen'
    )

    lower_sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lower_front_lidar_bridge',
        arguments=[
            '/lower_front_lidar'
            '@sensor_msgs/msg/LaserScan'
            '[ignition.msgs.LaserScan'
        ],
        output='screen'
    )

    autonomy_node = Node(
        package='mobile_arm_robot_autonomy',
        executable='lidar_decision_node',
        name='lidar_decision_node',
        output='screen',
        parameters=[{
            'forward_speed': 0.30,
            'slow_forward_speed': 0.08,
            'turn_speed': 0.58,
            'reverse_speed': -0.08,

            'obstacle_distance': 0.80,
            'danger_distance': 0.45,

            'arm_obstacle_distance': 0.80,
            'arm_danger_distance': 0.55,
            'arm_clear_distance': 0.95,

            'lower_obstacle_distance': 0.55,
            'lower_danger_distance': 0.30,
            'lower_clear_distance': 0.75,

            'base_slow_distance': 1.30,
            'arm_slow_distance': 1.20,
            'lower_slow_distance': 0.90,

            'max_linear_accel': 0.35,
            'max_linear_decel': 1.20,
            'max_angular_accel': 1.50,

            'minimum_avoid_time': 0.50,
            'clear_hold_time': 0.35,

            'sensor_timeout': 1.0,
            'turn_sign': 1.0
        }]
    )

    return LaunchDescription([
        arm_sensor_bridge,
        lower_sensor_bridge,
        autonomy_node
    ])
