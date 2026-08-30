import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    home = os.path.expanduser("~")

    world_file = os.path.join(
        home,
        "ros2_ws3_1/src/mobile_arm_robot_description/worlds/small_warehouse.world"
    )

    urdf_file = os.path.join(
        home,
        "ros2_ws3_1/src/mobile_arm_robot_description/urdf/mobile_arm_robot.urdf"
    )

    sdf_file = os.path.join(
        home,
        "ros2_ws3_1/src/mobile_arm_robot_description/urdf/mobile_arm_robot.sdf"
    )

    slam_params = os.path.join(
        home,
        "ros2_ws3_1/src/mobile_arm_robot_control/config/slam_mobile_params.yaml"
    )

    control_launch = os.path.join(
        get_package_share_directory("mobile_arm_robot_control"),
        "launch",
        "mobile_base_control.launch.py"
    )

    slam_launch = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "launch",
        "online_async_launch.py"
    )

    rviz_config = os.path.join(home, "ros2_ws3_1/src/mobile_arm_robot_control/rviz/mapping_demo.rviz")

    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", world_file],
        output="screen"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[urdf_file],
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ign", "service",
                    "-s", "/world/default/create",
                    "--reqtype", "ignition.msgs.EntityFactory",
                    "--reptype", "ignition.msgs.Boolean",
                    "--timeout", "300",
                    "--req",
                    f'sdf_filename: "{sdf_file}", name: "mobile_arm_robot", allow_renaming: false, pose: {{position: {{x: 0, y: 0, z: 0.23}}}}'
                ],
                output="screen"
            )
        ]
    )

    unpause_world = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ign", "service",
                    "-s", "/world/default/control",
                    "--reqtype", "ignition.msgs.WorldControl",
                    "--reptype", "ignition.msgs.Boolean",
                    "--timeout", "300",
                    "--req", "pause: false"
                ],
                output="screen"
            )
        ]
    )

    bridge = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                    "/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                    "/robot_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
                ],
                remappings=[
                    ("/lidar", "/scan"),
                ],
                output="screen"
            )
        ]
    )

    lidar_tf = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x", "-0.0908615203408174",
                    "--y", "0.00179684818200293",
                    "--z", "0.04",
                    "--roll", "0",
                    "--pitch", "0",
                    "--yaw", "0",
                    "--frame-id", "base_link",
                    "--child-frame-id", "mobile_arm_robot/base_link/lidar_sensor",
                ],
                output="screen"
            )
        ]
    )

    control_nodes = TimerAction(
        period=11.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(control_launch),
                launch_arguments={"use_sim_time": "true"}.items()
            )
        ]
    )

    slam_toolbox = TimerAction(
        period=16.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    "use_sim_time": "true",
                    "slam_params_file": slam_params,
                }.items()
            )
        ]
    )

    rviz = TimerAction(
        period=20.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
                output="screen"
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        unpause_world,
        bridge,
        lidar_tf,
        control_nodes,
        slam_toolbox,
        rviz,
    ])
