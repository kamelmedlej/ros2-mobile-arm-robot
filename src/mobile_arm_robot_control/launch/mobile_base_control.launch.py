from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    cmd_timeout = LaunchConfiguration("cmd_timeout")
    max_wheel_speed = LaunchConfiguration("max_wheel_speed")
    left_multiplier = LaunchConfiguration("left_multiplier")
    right_multiplier = LaunchConfiguration("right_multiplier")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    wheel_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wheel_velocity_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    mobile_base_velocity_controller = Node(
        package="mobile_arm_robot_control",
        executable="mobile_base_velocity_controller",
        name="mobile_base_velocity_controller",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "wheel_radius": ParameterValue(wheel_radius, value_type=float),
                "wheel_separation": ParameterValue(wheel_separation, value_type=float),
                "cmd_timeout": ParameterValue(cmd_timeout, value_type=float),
                "max_wheel_speed": ParameterValue(max_wheel_speed, value_type=float),
                "left_multiplier": ParameterValue(left_multiplier, value_type=float),
                "right_multiplier": ParameterValue(right_multiplier, value_type=float),
            }
        ],
    )

    mobile_base_odometry_node = Node(
        package="mobile_arm_robot_control",
        executable="mobile_base_odometry_node",
        name="mobile_base_odometry_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "wheel_radius": ParameterValue(wheel_radius, value_type=float),
                "wheel_separation": ParameterValue(wheel_separation, value_type=float),
                "odom_frame_id": "odom",
                "base_frame_id": "base_link",
                "publish_tf": True,
                "left_wheel_names": [
                    "front_left_wheel_joint",
                    "rear_left_wheel_joint",
                ],
                "right_wheel_names": [
                    "front_right_wheel_joint",
                    "rear_right_wheel_joint",
                ],
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if running in Gazebo",
            ),
            DeclareLaunchArgument(
                "wheel_radius",
                default_value="0.0545",
                description="Wheel radius in meters",
            ),
            DeclareLaunchArgument(
                "wheel_separation",
                default_value="0.405",
                description="Distance between left and right wheels in meters",
            ),
            DeclareLaunchArgument(
                "cmd_timeout",
                default_value="0.5",
                description="Stop wheels if no /cmd_vel is received",
            ),
            DeclareLaunchArgument(
                "max_wheel_speed",
                default_value="10.0",
                description="Maximum wheel angular speed rad/s",
            ),
            DeclareLaunchArgument(
                "left_multiplier",
                default_value="1.0",
                description="Direction multiplier for left wheels",
            ),
            DeclareLaunchArgument(
                "right_multiplier",
                default_value="1.0",
                description="Direction multiplier for right wheels",
            ),

            joint_state_broadcaster_spawner,
            wheel_velocity_controller_spawner,
            mobile_base_velocity_controller,
            mobile_base_odometry_node,
        ]
    )
