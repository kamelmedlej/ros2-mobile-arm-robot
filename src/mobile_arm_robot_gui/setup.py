from setuptools import find_packages, setup


package_name = "mobile_arm_robot_gui"


setup(
    name=package_name,
    version="0.0.2",

    packages=find_packages(
        exclude=["test"]
    ),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="kamel",
    maintainer_email="kamel@todo.todo",

    description=(
        "Graphical control, monitoring and ROS 2 "
        "system-flow interface for the mobile robot"
    ),

    license="MIT",

    tests_require=[
        "pytest",
    ],

    entry_points={
        "console_scripts": [
            (
                "robot_gui = "
                "mobile_arm_robot_gui.robot_gui:main"
            ),
            (
                "system_flow = "
                "mobile_arm_robot_gui."
                "system_flow_window:main"
            ),
        ],
    },
)
