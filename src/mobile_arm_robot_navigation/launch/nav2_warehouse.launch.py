import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    navigation_pkg_dir = get_package_share_directory('mobile_arm_robot_navigation')

    map_file = os.path.join(
        navigation_pkg_dir,
        'maps',
        'warehouse_map.yaml'
    )

    params_file = os.path.join(
        navigation_pkg_dir,
        'config',
        'nav2_warehouse_params.yaml'
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'map': map_file,
            'params_file': params_file,
            'use_composition': 'False',
            'log_level': 'info'
        }.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        nav2_bringup,
        rviz
    ])
