cmake_minimum_required(VERSION 3.8)
project(mobile_arm_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(mobile_base_velocity_controller src/mobile_base_velocity_controller.cpp)

ament_target_dependencies(
  mobile_base_velocity_controller
  rclcpp
  geometry_msgs
  std_msgs
)

add_executable(mobile_base_odometry_node src/mobile_base_odometry_node.cpp)

ament_target_dependencies(
  mobile_base_odometry_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(mobile_base_trajectory_recorder src/mobile_base_trajectory_recorder.cpp)

ament_target_dependencies(
  mobile_base_trajectory_recorder
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
)

install(
  TARGETS
    mobile_base_velocity_controller
    mobile_base_odometry_node
    mobile_base_trajectory_recorder
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


add_executable(mobile_base_trajectory_player src/mobile_base_trajectory_player.cpp)
ament_target_dependencies(mobile_base_trajectory_player rclcpp geometry_msgs)

install(TARGETS
  mobile_base_trajectory_player
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
