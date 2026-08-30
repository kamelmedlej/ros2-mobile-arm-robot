import os
from glob import glob

from setuptools import setup


package_name = 'mobile_arm_robot_autonomy'


setup(
    name=package_name,
    version='0.0.0',

    packages=[
        package_name
    ],

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='kamel',
    maintainer_email='kamel@example.com',

    description=(
        'Autonomous robot movement using lower LiDAR '
        'and an arm-mounted distance sensor'
    ),

    license='MIT',

    tests_require=[
        'pytest'
    ],

    entry_points={
        'console_scripts': [
            (
                'lidar_decision_node = '
                'mobile_arm_robot_autonomy.'
                'lidar_decision_node:main'
            ),
        ],
    },
)
