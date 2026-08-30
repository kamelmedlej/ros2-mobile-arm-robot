import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def append_env(name, value):
    old = os.environ.get(name, "")
    if old:
        return old + ":" + value
    return value


def generate_launch_description():
    home = os.path.expanduser("~")

    ws = os.path.join(home, "ros2_ws3_1")

    navigation_pkg_dir = get_package_share_directory(
        "mobile_arm_robot_navigation"
    )

    default_map_file = os.path.join(
        navigation_pkg_dir,
        "maps",
        "warehouse_map.yaml"
    )

    selected_map = LaunchConfiguration("map")

    world_file = os.path.join(
        ws,
        "src/mobile_arm_robot_description/worlds/small_warehouse.world"
    )

    urdf_file = os.path.join(
        ws,
        "src/mobile_arm_robot_description/urdf/mobile_arm_robot.urdf"
    )

    sdf_file = os.path.join(
        ws,
        "src/mobile_arm_robot_description/urdf/mobile_arm_robot.sdf"
    )

    description_pkg_src = os.path.join(
        ws,
        "src/mobile_arm_robot_description"
    )

    description_models = os.path.join(
        ws,
        "src/mobile_arm_robot_description/models"
    )

    control_launch = os.path.join(
        get_package_share_directory("mobile_arm_robot_control"),
        "launch",
        "mobile_base_control.launch.py"
    )

    nav_launch = os.path.join(
        get_package_share_directory("mobile_arm_robot_navigation"),
        "launch",
        "nav2_warehouse.launch.py"
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "map",
            default_value=default_map_file,
            description="Full path to the selected ROS map YAML file"
        ),


        # Environment
        SetEnvironmentVariable(
            name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
            value=append_env("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "/opt/ros/humble/lib")
        ),

        SetEnvironmentVariable(
            name="LD_LIBRARY_PATH",
            value=append_env("LD_LIBRARY_PATH", "/opt/ros/humble/lib")
        ),

        SetEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=f"{description_pkg_src}:{description_models}"
        ),

        # 1) Gazebo
        ExecuteProcess(
            cmd=[
                "ign", "gazebo", "-r", world_file
            ],
            output="screen"
        ),

        # 2) robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            arguments=[urdf_file],
            parameters=[{"use_sim_time": True}]
        ),

        # 3) Spawn robot
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ign", "service",
                        "-s", "/world/default/create",
                        "--reqtype", "ignition.msgs.EntityFactory",
                        "--reptype", "ignition.msgs.Boolean",
                        "--timeout", "1000",
                        "--req",
                        f'sdf_filename: "{sdf_file}", name: "mobile_arm_robot", allow_renaming: false, pose: {{position: {{x: 0, y: 0, z: 0.23}}}}'
                    ],
                    output="screen"
                )
            ]
        ),

        # 4) Unpause world
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ign", "service",
                        "-s", "/world/default/control",
                        "--reqtype", "ignition.msgs.WorldControl",
                        "--reptype", "ignition.msgs.Boolean",
                        "--timeout", "1000",
                        "--req", "pause: false"
                    ],
                    output="screen"
                )
            ]
        ),

        # 5) Gazebo <-> ROS bridge
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name="ros_gz_bridge",
                    output="screen",
                    arguments=[
                        "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                        "/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                        "/robot_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
                        "/robot_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                    ],
                    remappings=[
                        ("/lidar", "/scan")
                    ]
                )
            ]
        ),

        # 6) LiDAR static TF القديم
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="lidar_static_tf",
                    output="screen",
                    arguments=[
                        "-0.0908615203408174",
                        "0.00179684818200293",
                        "0.04",
                        "0",
                        "0",
                        "0",
                        "base_link",
                        "mobile_arm_robot/base_link/lidar_sensor"
                    ]
                )
            ]
        ),

        # 7) Control
        TimerAction(
            period=7.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(control_launch)
                )
            ]
        ),

        # 8) Nav2
        TimerAction(
            period=11.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav_launch),
                    launch_arguments={
                        "map": selected_map
                    }.items()
                )
            ]
        ),
    ])
