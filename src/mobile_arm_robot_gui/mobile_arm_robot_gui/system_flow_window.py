#!/usr/bin/env python3

import math
import sys
import time
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, LaserScan
from std_msgs.msg import Float64MultiArray

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


class SystemFlowNode(Node):
    def __init__(self):
        super().__init__("system_flow_gui_node")

        self.live_data = {}
        self.last_message_times = {}
        self.message_rates = {}

        self.gui_subscriptions = [
            self.create_subscription(
                LaserScan,
                "/scan",
                lambda msg, topic="/scan":
                self.laser_callback(topic, msg),
                10
            ),
            self.create_subscription(
                LaserScan,
                "/arm_front_lidar",
                lambda msg, topic="/arm_front_lidar":
                self.laser_callback(topic, msg),
                10
            ),
            self.create_subscription(
                LaserScan,
                "/lower_front_lidar",
                lambda msg, topic="/lower_front_lidar":
                self.laser_callback(topic, msg),
                10
            ),
            self.create_subscription(
                Twist,
                "/cmd_vel",
                self.cmd_vel_callback,
                10
            ),
            self.create_subscription(
                Float64MultiArray,
                "/wheel_velocity_controller/commands",
                self.wheel_commands_callback,
                10
            ),
            self.create_subscription(
                JointState,
                "/joint_states",
                self.joint_states_callback,
                10
            ),
            self.create_subscription(
                Odometry,
                "/odom",
                self.odom_callback,
                10
            ),
            self.create_subscription(
                OccupancyGrid,
                "/map",
                self.map_callback,
                10
            ),
            self.create_subscription(
                Image,
                "/robot_camera/image",
                self.camera_callback,
                10
            ),
        ]

    def update_live_data(self, topic, values):
        now = time.monotonic()
        previous_time = self.last_message_times.get(topic)

        if previous_time is not None:
            delta_time = now - previous_time

            if delta_time > 0.0:
                instantaneous_rate = 1.0 / delta_time
                old_rate = self.message_rates.get(topic, 0.0)

                if old_rate <= 0.0:
                    filtered_rate = instantaneous_rate
                else:
                    filtered_rate = (
                        0.75 * old_rate
                        + 0.25 * instantaneous_rate
                    )

                self.message_rates[topic] = filtered_rate

        self.last_message_times[topic] = now

        self.live_data[topic] = {
            "stamp": now,
            "values": values,
        }

    def laser_callback(self, topic, msg):
        valid_values = [
            value
            for value in msg.ranges
            if math.isfinite(value)
            and msg.range_min <= value <= msg.range_max
        ]

        minimum_distance = (
            min(valid_values)
            if valid_values
            else float("inf")
        )

        self.update_live_data(
            topic,
            {
                "minimum distance": minimum_distance,
                "valid measurements": len(valid_values),
                "total measurements": len(msg.ranges),
                "range minimum": msg.range_min,
                "range maximum": msg.range_max,
            }
        )

    def cmd_vel_callback(self, msg):
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)

        if abs(vx) < 0.01 and abs(wz) < 0.01:
            action = "STOPPED"
        elif abs(vx) >= 0.01 and abs(wz) < 0.05:
            action = "LINEAR MOTION"
        elif abs(vx) < 0.01 and wz > 0.05:
            action = "TURN LEFT"
        elif abs(vx) < 0.01 and wz < -0.05:
            action = "TURN RIGHT"
        else:
            action = "MOVE AND TURN"

        self.update_live_data(
            "/cmd_vel",
            {
                "linear.x": vx,
                "angular.z": wz,
                "estimated action": action,
            }
        )

    def wheel_commands_callback(self, msg):
        values = [
            float(value)
            for value in msg.data
        ]

        self.update_live_data(
            "/wheel_velocity_controller/commands",
            {
                "command count": len(values),
                "wheel commands": (
                    "[" +
                    ", ".join(
                        f"{value:.3f}"
                        for value in values
                    )
                    + "] rad/s"
                ),
            }
        )

    def joint_states_callback(self, msg):
        wheel_values = []

        for index, joint_name in enumerate(msg.name):
            if "wheel" not in joint_name:
                continue

            velocity = (
                msg.velocity[index]
                if index < len(msg.velocity)
                else float("nan")
            )

            wheel_values.append(
                f"{joint_name}={velocity:.3f}"
            )

        self.update_live_data(
            "/joint_states",
            {
                "total joints": len(msg.name),
                "wheel velocities": (
                    " | ".join(wheel_values)
                    if wheel_values
                    else "No wheel velocity data"
                ),
            }
        )

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        siny_cosp = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )

        yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        self.update_live_data(
            "/odom",
            {
                "position x": float(position.x),
                "position y": float(position.y),
                "yaw": math.degrees(yaw),
                "linear velocity": float(
                    msg.twist.twist.linear.x
                ),
                "angular velocity": float(
                    msg.twist.twist.angular.z
                ),
            }
        )

    def map_callback(self, msg):
        occupied_cells = sum(
            1
            for value in msg.data
            if value > 50
        )

        self.update_live_data(
            "/map",
            {
                "width": msg.info.width,
                "height": msg.info.height,
                "resolution": float(
                    msg.info.resolution
                ),
                "occupied cells": occupied_cells,
            }
        )

    def camera_callback(self, msg):
        self.update_live_data(
            "/robot_camera/image",
            {
                "width": msg.width,
                "height": msg.height,
                "encoding": msg.encoding,
                "step": msg.step,
            }
        )

    def describe_live_topic(self, topic):
        item = self.live_data.get(topic)

        if item is None:
            return (
                f"{topic}\n"
                "  Status: NO LIVE DATA"
            )

        now = time.monotonic()
        age = now - item["stamp"]
        rate = self.message_rates.get(topic, 0.0)

        freshness = (
            "FRESH"
            if age < 2.0
            else "STALE"
        )

        lines = [
            topic,
            f"  Status: {freshness}",
            f"  Message rate: {rate:.2f} Hz",
            f"  Last message age: {age:.2f} s",
        ]

        for name, value in item["values"].items():
            if isinstance(value, float):
                if math.isfinite(value):
                    lines.append(
                        f"  {name}: {value:.3f}"
                    )
                else:
                    lines.append(
                        f"  {name}: inf"
                    )
            else:
                lines.append(
                    f"  {name}: {value}"
                )

        return "\n".join(lines)


class SystemFlowWindow(QMainWindow):
    def __init__(self, ros_node: Node):
        super().__init__()

        self.ros_node = ros_node
        self.buttons: Dict[str, QPushButton] = {}
        self.statuses: Dict[str, str] = {}
        self.selected_key: Optional[str] = None

        self.setWindowTitle("ROS 2 System Flow")
        self.resize(1250, 820)

        self.details = self.create_details_database()

        self.build_interface()

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh_graph)
        self.refresh_timer.start(500)

        self.refresh_graph()

    def create_details_database(self) -> Dict[str, Dict[str, object]]:
        return {
            "gazebo": {
                "title": "Gazebo Simulation",
                "role": (
                    "Создаёт виртуальный склад, физическую модель робота, "
                    "колёса и виртуальные датчики."
                ),
                "inputs": ["SDF model", "World file"],
                "outputs": [
                    "/clock",
                    "Gazebo sensor data",
                    "Wheel joint states",
                ],
            },
            "scan": {
                "title": "Base LiDAR — /scan",
                "role": (
                    "Передаёт измерения расстояний вокруг основной "
                    "платформы робота."
                ),
                "inputs": ["Gazebo LiDAR sensor"],
                "outputs": ["/scan"],
            },
            "arm_scan": {
                "title": "Arm sensor — /arm_front_lidar",
                "role": (
                    "Обнаруживает препятствия на уровне "
                    "роботизированного манипулятора."
                ),
                "inputs": ["Arm-mounted distance sensor"],
                "outputs": ["/arm_front_lidar"],
            },
            "lower_scan": {
                "title": "Lower sensor — /lower_front_lidar",
                "role": (
                    "Обнаруживает низкие препятствия перед нижней "
                    "частью мобильной платформы."
                ),
                "inputs": ["Lower front distance sensor"],
                "outputs": ["/lower_front_lidar"],
            },
            "camera": {
                "title": "Robot camera",
                "role": (
                    "Передаёт изображение окружающей среды "
                    "с точки зрения робота."
                ),
                "inputs": ["Gazebo camera sensor"],
                "outputs": ["/robot_camera/image"],
            },
            "bridge": {
                "title": "ros_gz_bridge",
                "role": (
                    "Передаёт данные датчиков и времени между "
                    "Gazebo и ROS 2."
                ),
                "inputs": ["Gazebo Transport topics"],
                "outputs": [
                    "/scan",
                    "/arm_front_lidar",
                    "/lower_front_lidar",
                    "/robot_camera/image",
                    "/clock",
                ],
            },
            "decision": {
                "title": "lidar_decision_node",
                "role": (
                    "Анализирует данные LiDAR, обнаруживает препятствия, "
                    "выбирает направление движения и формирует "
                    "команду скорости."
                ),
                "inputs": [
                    "/scan",
                    "/arm_front_lidar",
                    "/lower_front_lidar",
                ],
                "outputs": ["/cmd_vel"],
            },
            "cmd_vel": {
                "title": "/cmd_vel",
                "role": (
                    "Передаёт линейную и угловую скорость "
                    "мобильной платформы."
                ),
                "inputs": [
                    "lidar_decision_node",
                    "Navigation 2",
                    "Manual control",
                    "GUI stop command",
                ],
                "outputs": ["mobile_base_velocity_controller"],
            },
            "base_controller": {
                "title": "mobile_base_velocity_controller",
                "role": (
                    "Преобразует linear.x и angular.z в требуемые "
                    "угловые скорости левых и правых колёс."
                ),
                "inputs": ["/cmd_vel"],
                "outputs": [
                    "/wheel_velocity_controller/commands"
                ],
            },
            "wheel_commands": {
                "title": "/wheel_velocity_controller/commands",
                "role": (
                    "Содержит четыре команды угловой скорости "
                    "для колёс робота."
                ),
                "inputs": ["mobile_base_velocity_controller"],
                "outputs": ["wheel_velocity_controller"],
            },
            "wheel_controller": {
                "title": "wheel_velocity_controller",
                "role": (
                    "Передаёт команды скорости на velocity interfaces "
                    "четырёх колёс."
                ),
                "inputs": [
                    "/wheel_velocity_controller/commands"
                ],
                "outputs": ["ros2_control hardware interfaces"],
            },
            "controller_manager": {
                "title": "ros2_control / controller_manager",
                "role": (
                    "Управляет контроллерами и взаимодействием "
                    "с суставами колёс в Gazebo."
                ),
                "inputs": ["Controller commands"],
                "outputs": [
                    "Wheel joint commands",
                    "/joint_states",
                ],
            },
            "joint_states": {
                "title": "/joint_states",
                "role": (
                    "Передаёт положения и скорости суставов, "
                    "включая суставы колёс."
                ),
                "inputs": ["joint_state_broadcaster"],
                "outputs": [
                    "robot_state_publisher",
                    "mobile_base_odometry_node",
                ],
            },
            "odometry_node": {
                "title": "mobile_base_odometry_node",
                "role": (
                    "Рассчитывает положение x, y и yaw робота "
                    "по вращению колёс."
                ),
                "inputs": ["/joint_states"],
                "outputs": [
                    "/odom",
                    "TF: odom → base_link",
                ],
            },
            "odom": {
                "title": "/odom",
                "role": (
                    "Содержит рассчитанное положение, ориентацию "
                    "и скорость мобильного робота."
                ),
                "inputs": ["mobile_base_odometry_node"],
                "outputs": [
                    "SLAM Toolbox",
                    "Navigation 2",
                    "GUI monitoring",
                ],
            },
            "tf": {
                "title": "TF transforms",
                "role": (
                    "Связывает системы координат map, odom, base_link, "
                    "датчиков, колёс и манипулятора."
                ),
                "inputs": [
                    "robot_state_publisher",
                    "odometry node",
                    "SLAM / localization",
                ],
                "outputs": [
                    "RViz",
                    "SLAM Toolbox",
                    "Navigation 2",
                ],
            },
            "slam": {
                "title": "SLAM Toolbox",
                "role": (
                    "Объединяет данные LiDAR и одометрии для "
                    "построения карты и определения положения робота."
                ),
                "inputs": [
                    "/scan",
                    "/odom",
                    "TF",
                ],
                "outputs": [
                    "/map",
                    "TF: map → odom",
                ],
            },
            "map": {
                "title": "/map",
                "role": (
                    "Содержит двумерную карту окружающей среды."
                ),
                "inputs": ["SLAM Toolbox or map_server"],
                "outputs": [
                    "RViz",
                    "Navigation 2",
                ],
            },
            "nav2": {
                "title": "Navigation 2",
                "role": (
                    "Определяет положение робота на карте, "
                    "строит маршрут и формирует команды движения "
                    "к заданной цели."
                ),
                "inputs": [
                    "/map",
                    "/scan",
                    "/odom",
                    "TF",
                    "Nav2 Goal",
                ],
                "outputs": [
                    "Path",
                    "/cmd_vel",
                    "Navigation feedback",
                ],
            },
        }

    def build_interface(self):
        central = QWidget()
        main_layout = QHBoxLayout(central)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)

        title = QLabel("ROS 2 SYSTEM ARCHITECTURE AND DATA FLOW")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(
            "font-size: 22px;"
            "font-weight: bold;"
            "padding: 12px;"
        )
        left_layout.addWidget(title)

        legend = QLabel(
            "● Green: active   ● Yellow: topic exists without publisher   "
            "● Red: inactive"
        )
        legend.setAlignment(Qt.AlignCenter)
        legend.setStyleSheet(
            "font-size: 14px;"
            "padding: 7px;"
            "background-color: #eeeeee;"
        )
        left_layout.addWidget(legend)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)

        flow_widget = QWidget()
        self.flow_layout = QVBoxLayout(flow_widget)
        self.flow_layout.setAlignment(Qt.AlignTop)

        self.add_flow_button(
            "gazebo",
            "GAZEBO SIMULATION"
        )

        self.add_arrow("↓  Virtual sensors and robot physics")

        sensors_group = QGroupBox("PERCEPTION SENSORS")
        sensors_layout = QHBoxLayout(sensors_group)

        for key, title_text in (
            ("scan", "BASE LiDAR\n/scan"),
            ("arm_scan", "ARM SENSOR\n/arm_front_lidar"),
            ("lower_scan", "LOWER SENSOR\n/lower_front_lidar"),
            ("camera", "CAMERA\n/robot_camera/image"),
        ):
            button = self.make_button(key, title_text)
            sensors_layout.addWidget(button)

        self.flow_layout.addWidget(sensors_group)

        self.add_arrow("↓  Gazebo Transport → ROS 2")

        self.add_flow_button(
            "bridge",
            "ros_gz_bridge"
        )

        self.add_arrow("↓  LaserScan messages")

        self.add_flow_button(
            "decision",
            "lidar_decision_node"
        )

        self.add_arrow("↓  Selected linear and angular velocity")

        self.add_flow_button(
            "cmd_vel",
            "TOPIC: /cmd_vel"
        )

        self.add_arrow("↓")

        self.add_flow_button(
            "base_controller",
            "mobile_base_velocity_controller"
        )

        self.add_arrow("↓  Four wheel angular velocities")

        self.add_flow_button(
            "wheel_commands",
            "TOPIC: /wheel_velocity_controller/commands"
        )

        self.add_arrow("↓")

        self.add_flow_button(
            "wheel_controller",
            "wheel_velocity_controller"
        )

        self.add_arrow("↓  velocity interfaces")

        self.add_flow_button(
            "controller_manager",
            "ros2_control / controller_manager"
        )

        self.add_arrow("↓  Measured joint positions and velocities")

        self.add_flow_button(
            "joint_states",
            "TOPIC: /joint_states"
        )

        self.add_arrow("↓")

        self.add_flow_button(
            "odometry_node",
            "mobile_base_odometry_node"
        )

        odom_tf_group = QGroupBox("ROBOT POSITION AND TRANSFORMS")
        odom_tf_layout = QHBoxLayout(odom_tf_group)
        odom_tf_layout.addWidget(
            self.make_button("odom", "TOPIC: /odom")
        )
        odom_tf_layout.addWidget(
            self.make_button("tf", "TOPICS: /tf + /tf_static")
        )
        self.flow_layout.addWidget(odom_tf_group)

        self.add_arrow("↓  LiDAR + Odometry + TF")

        self.add_flow_button(
            "slam",
            "SLAM Toolbox"
        )

        self.add_arrow("↓")

        self.add_flow_button(
            "map",
            "TOPIC: /map"
        )

        self.add_arrow("↓  Saved or live map")

        self.add_flow_button(
            "nav2",
            "NAVIGATION 2"
        )

        self.add_arrow("↺  Navigation commands return to /cmd_vel")

        scroll.setWidget(flow_widget)
        left_layout.addWidget(scroll)

        details_group = QGroupBox("SELECTED COMPONENT INFORMATION")
        details_layout = QVBoxLayout(details_group)

        self.details_title = QLabel(
            "Click any node or topic"
        )
        self.details_title.setAlignment(Qt.AlignCenter)
        self.details_title.setStyleSheet(
            "font-size: 18px;"
            "font-weight: bold;"
            "padding: 10px;"
        )

        self.details_text = QTextEdit()
        self.details_text.setReadOnly(True)
        self.details_text.setMinimumWidth(390)
        self.details_text.setStyleSheet(
            "font-size: 15px;"
        )

        refresh_button = QPushButton("REFRESH ROS 2 GRAPH")
        refresh_button.setMinimumHeight(45)
        refresh_button.clicked.connect(
            self.refresh_graph
        )

        details_layout.addWidget(self.details_title)
        details_layout.addWidget(self.details_text)
        details_layout.addWidget(refresh_button)

        main_layout.addWidget(left_panel, 3)
        main_layout.addWidget(details_group, 2)

        self.setCentralWidget(central)

    def add_arrow(self, text: str):
        label = QLabel(text)
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet(
            "font-size: 15px;"
            "font-weight: bold;"
            "color: #455a64;"
            "padding: 4px;"
        )
        self.flow_layout.addWidget(label)

    def make_button(
        self,
        key: str,
        text: str
    ) -> QPushButton:
        button = QPushButton(text)
        button.setMinimumHeight(58)
        button.clicked.connect(
            lambda checked=False, item_key=key:
            self.show_details(item_key)
        )

        self.buttons[key] = button
        return button

    def add_flow_button(
        self,
        key: str,
        text: str
    ):
        self.flow_layout.addWidget(
            self.make_button(key, text)
        )

    def graph_snapshot(
        self
    ) -> Tuple[Set[str], Set[str], Dict[str, List[str]]]:
        node_names: Set[str] = set()

        try:
            nodes = self.ros_node.get_node_names_and_namespaces()

            for node_name, namespace in nodes:
                node_names.add(node_name)

                if namespace == "/":
                    node_names.add(f"/{node_name}")
                else:
                    node_names.add(
                        f"{namespace.rstrip('/')}/{node_name}"
                    )

        except Exception:
            pass

        topics: Dict[str, List[str]] = {}

        try:
            for topic_name, topic_types in (
                self.ros_node.get_topic_names_and_types()
            ):
                topics[topic_name] = topic_types

        except Exception:
            pass

        return node_names, set(topics.keys()), topics

    def node_exists(
        self,
        node_names: Set[str],
        expected: str
    ) -> bool:
        expected_clean = expected.lstrip("/")

        return any(
            name.lstrip("/") == expected_clean
            for name in node_names
        )

    def topic_status(
        self,
        topic_name: str,
        topics: Set[str]
    ) -> str:
        if topic_name not in topics:
            return "inactive"

        try:
            publishers = (
                self.ros_node.get_publishers_info_by_topic(
                    topic_name
                )
            )

            if publishers:
                return "active"

            return "warning"

        except Exception:
            return "warning"

    def calculate_statuses(
        self,
        node_names: Set[str],
        topics: Set[str]
    ):
        bridge_running = any(
            "bridge" in name.lower()
            for name in node_names
        )

        nav_nodes = (
            "map_server",
            "amcl",
            "planner_server",
            "controller_server",
            "bt_navigator",
            "behavior_server",
        )

        nav_running = any(
            self.node_exists(node_names, name)
            for name in nav_nodes
        )

        self.statuses = {
            "gazebo": (
                "active"
                if "/clock" in topics
                else "inactive"
            ),
            "scan": self.topic_status("/scan", topics),
            "arm_scan": self.topic_status(
                "/arm_front_lidar",
                topics
            ),
            "lower_scan": self.topic_status(
                "/lower_front_lidar",
                topics
            ),
            "camera": self.topic_status(
                "/robot_camera/image",
                topics
            ),
            "bridge": (
                "active"
                if bridge_running
                else "inactive"
            ),
            "decision": (
                "active"
                if self.node_exists(
                    node_names,
                    "lidar_decision_node"
                )
                else "inactive"
            ),
            "cmd_vel": self.topic_status(
                "/cmd_vel",
                topics
            ),
            "base_controller": (
                "active"
                if self.node_exists(
                    node_names,
                    "mobile_base_velocity_controller"
                )
                else "inactive"
            ),
            "wheel_commands": self.topic_status(
                "/wheel_velocity_controller/commands",
                topics
            ),
            "wheel_controller": (
                "active"
                if self.node_exists(
                    node_names,
                    "wheel_velocity_controller"
                )
                else "inactive"
            ),
            "controller_manager": (
                "active"
                if self.node_exists(
                    node_names,
                    "controller_manager"
                )
                else "inactive"
            ),
            "joint_states": self.topic_status(
                "/joint_states",
                topics
            ),
            "odometry_node": (
                "active"
                if self.node_exists(
                    node_names,
                    "mobile_base_odometry_node"
                )
                else "inactive"
            ),
            "odom": self.topic_status(
                "/odom",
                topics
            ),
            "tf": (
                "active"
                if "/tf" in topics
                else "inactive"
            ),
            "slam": (
                "active"
                if self.node_exists(
                    node_names,
                    "slam_toolbox"
                )
                else "inactive"
            ),
            "map": self.topic_status(
                "/map",
                topics
            ),
            "nav2": (
                "active"
                if nav_running
                else "inactive"
            ),
        }

    def apply_button_status(
        self,
        key: str,
        status: str
    ):
        button = self.buttons.get(key)

        if button is None:
            return

        if status == "active":
            background = "#2e7d32"
            foreground = "white"
            border = "#1b5e20"

        elif status == "warning":
            background = "#f9a825"
            foreground = "black"
            border = "#f57f17"

        else:
            background = "#c62828"
            foreground = "white"
            border = "#8e0000"

        if key == self.selected_key:
            border_width = "4px"
            border = "#1565c0"
        else:
            border_width = "2px"

        button.setStyleSheet(
            f"font-size: 14px;"
            f"font-weight: bold;"
            f"background-color: {background};"
            f"color: {foreground};"
            f"border: {border_width} solid {border};"
            f"border-radius: 8px;"
            f"padding: 8px;"
        )

    def refresh_graph(self):
        node_names, topics, _ = self.graph_snapshot()

        self.calculate_statuses(
            node_names,
            topics
        )

        for key, status in self.statuses.items():
            self.apply_button_status(
                key,
                status
            )

        if self.selected_key is not None:
            self.show_details(
                self.selected_key,
                refresh_only=True
            )

    def endpoint_node_name(self, endpoint):
        node_name = getattr(
            endpoint,
            "node_name",
            ""
        ) or "unknown_node"

        namespace = getattr(
            endpoint,
            "node_namespace",
            "/"
        ) or "/"

        node_name = node_name.lstrip("/")

        if not namespace.startswith("/"):
            namespace = "/" + namespace

        namespace = namespace.rstrip("/")

        if namespace in ("", "/"):
            return f"/{node_name}"

        return f"{namespace}/{node_name}"

    def endpoint_branches(self, names):
        if not names:
            return [
                "       └──► NONE"
            ]

        lines = []

        for index, name in enumerate(names):
            if index == len(names) - 1:
                branch = "       └──►"
            else:
                branch = "       ├──►"

            lines.append(
                f"{branch} {name}"
            )

        return lines

    def topic_runtime_information(
        self,
        topic_name: str
    ) -> str:
        try:
            topics = dict(
                self.ros_node.get_topic_names_and_types()
            )

            topic_types = topics.get(
                topic_name,
                []
            )

            publishers = (
                self.ros_node.get_publishers_info_by_topic(
                    topic_name
                )
            )

            subscribers = (
                self.ros_node.get_subscriptions_info_by_topic(
                    topic_name
                )
            )

            publisher_names = sorted(
                {
                    self.endpoint_node_name(endpoint)
                    for endpoint in publishers
                }
            )

            subscriber_names = sorted(
                {
                    self.endpoint_node_name(endpoint)
                    for endpoint in subscribers
                }
            )

            topic_type = (
                ", ".join(topic_types)
                if topic_types
                else "unknown"
            )

            lines = [
                "",
                "================================",
                "LIVE ROS 2 CONNECTIONS",
                "================================",
                f"Topic type: {topic_type}",
                "",
                f"PUBLISHERS ({len(publisher_names)}):",
            ]

            if publisher_names:
                lines.extend(
                    self.endpoint_branches(
                        publisher_names
                    )
                )

                lines.extend([
                    "",
                    "              │ publishes to",
                    "              ▼",
                ])
            else:
                lines.extend([
                    "       └──► No active publisher",
                    "",
                    "              ▼",
                ])

            lines.extend([
                f"          {topic_name}",
                "              │ subscribed by",
                "              ▼",
                "",
                f"SUBSCRIBERS ({len(subscriber_names)}):",
            ])

            if subscriber_names:
                lines.extend(
                    self.endpoint_branches(
                        subscriber_names
                    )
                )
            else:
                lines.append(
                    "       └──► No active subscriber"
                )

            return "\n".join(lines)

        except Exception as error:
            return (
                "\nLIVE ROS 2 CONNECTIONS\n"
                f"Runtime information unavailable: {error}"
            )


    def nav2_runtime_information(self) -> str:
        expected_nodes = (
            "map_server",
            "amcl",
            "planner_server",
            "controller_server",
            "bt_navigator",
            "behavior_server",
        )

        node_names, _, _ = self.graph_snapshot()

        lines = ["", "Navigation nodes:"]

        for node_name in expected_nodes:
            status = (
                "RUNNING"
                if self.node_exists(node_names, node_name)
                else "STOPPED"
            )

            lines.append(
                f"  {node_name}: {status}"
            )

        return "\n".join(lines)

    def live_component_text(self, key):
        component_topics = {
            "gazebo": [
                ("SENSOR OUTPUT", "/scan"),
                ("JOINT OUTPUT", "/joint_states"),
            ],
            "scan": [
                ("LIVE OUTPUT", "/scan"),
            ],
            "arm_scan": [
                ("LIVE OUTPUT", "/arm_front_lidar"),
            ],
            "lower_scan": [
                ("LIVE OUTPUT", "/lower_front_lidar"),
            ],
            "camera": [
                ("LIVE OUTPUT", "/robot_camera/image"),
            ],
            "bridge": [
                ("OUTPUT", "/scan"),
                ("OUTPUT", "/arm_front_lidar"),
                ("OUTPUT", "/lower_front_lidar"),
                ("OUTPUT", "/robot_camera/image"),
            ],
            "decision": [
                ("INPUT 1", "/scan"),
                ("INPUT 2", "/arm_front_lidar"),
                ("INPUT 3", "/lower_front_lidar"),
                ("OUTPUT", "/cmd_vel"),
            ],
            "cmd_vel": [
                ("LIVE TOPIC", "/cmd_vel"),
            ],
            "base_controller": [
                ("INPUT", "/cmd_vel"),
                (
                    "OUTPUT",
                    "/wheel_velocity_controller/commands"
                ),
            ],
            "wheel_commands": [
                (
                    "LIVE TOPIC",
                    "/wheel_velocity_controller/commands"
                ),
            ],
            "wheel_controller": [
                (
                    "INPUT",
                    "/wheel_velocity_controller/commands"
                ),
                ("MEASURED OUTPUT", "/joint_states"),
            ],
            "controller_manager": [
                ("JOINT OUTPUT", "/joint_states"),
            ],
            "joint_states": [
                ("LIVE TOPIC", "/joint_states"),
            ],
            "odometry_node": [
                ("INPUT", "/joint_states"),
                ("OUTPUT", "/odom"),
            ],
            "odom": [
                ("LIVE TOPIC", "/odom"),
            ],
            "slam": [
                ("INPUT LiDAR", "/scan"),
                ("INPUT ODOMETRY", "/odom"),
                ("OUTPUT MAP", "/map"),
            ],
            "map": [
                ("LIVE TOPIC", "/map"),
            ],
            "nav2": [
                ("INPUT MAP", "/map"),
                ("INPUT LiDAR", "/scan"),
                ("INPUT ODOMETRY", "/odom"),
                ("OUTPUT COMMAND", "/cmd_vel"),
            ],
        }

        entries = component_topics.get(key, [])

        if not entries:
            return (
                "No numeric live stream is configured "
                "for this component yet."
            )

        blocks = []

        for label, topic in entries:
            blocks.append(
                f"{label}\n"
                f"{self.ros_node.describe_live_topic(topic)}"
            )

        return "\n\n".join(blocks)

    def show_details(
        self,
        key: str,
        refresh_only: bool = False
    ):
        if key not in self.details:
            return

        self.selected_key = key

        if not refresh_only:
            for item_key, status in self.statuses.items():
                self.apply_button_status(
                    item_key,
                    status
                )

        data = self.details[key]
        status = self.statuses.get(
            key,
            "inactive"
        ).upper()

        self.details_title.setText(
            str(data["title"])
        )

        inputs = "\n".join(
            f"  • {item}"
            for item in data["inputs"]
        )

        outputs = "\n".join(
            f"  • {item}"
            for item in data["outputs"]
        )

        runtime = ""

        topic_mapping = {
            "scan": "/scan",
            "arm_scan": "/arm_front_lidar",
            "lower_scan": "/lower_front_lidar",
            "camera": "/robot_camera/image",
            "cmd_vel": "/cmd_vel",
            "wheel_commands":
                "/wheel_velocity_controller/commands",
            "joint_states": "/joint_states",
            "odom": "/odom",
            "tf": "/tf",
            "map": "/map",
        }

        if key in topic_mapping:
            runtime = self.topic_runtime_information(
                topic_mapping[key]
            )

        if key == "nav2":
            runtime += self.nav2_runtime_information()

        live_information = self.live_component_text(key)

        text = (
            f"STATUS: {status}\n\n"
            f"FUNCTION:\n{data['role']}\n\n"
            f"INPUTS:\n{inputs}\n\n"
            f"OUTPUTS:\n{outputs}\n"
            f"{runtime}\n\n"
            f"==============================\n"
            f"LIVE INPUTS AND OUTPUTS\n"
            f"==============================\n"
            f"{live_information}"
        )

        vertical_scroll = (
            self.details_text.verticalScrollBar()
        )
        horizontal_scroll = (
            self.details_text.horizontalScrollBar()
        )

        old_vertical_value = vertical_scroll.value()
        old_horizontal_value = horizontal_scroll.value()

        old_vertical_maximum = vertical_scroll.maximum()
        was_at_bottom = (
            old_vertical_value
            >= old_vertical_maximum - 5
        )

        self.details_text.setPlainText(text)

        if refresh_only:
            def restore_scroll_position():
                if was_at_bottom:
                    vertical_scroll.setValue(
                        vertical_scroll.maximum()
                    )
                else:
                    vertical_scroll.setValue(
                        min(
                            old_vertical_value,
                            vertical_scroll.maximum()
                        )
                    )

                horizontal_scroll.setValue(
                    min(
                        old_horizontal_value,
                        horizontal_scroll.maximum()
                    )
                )

            QTimer.singleShot(
                0,
                restore_scroll_position
            )
        else:
            vertical_scroll.setValue(0)
            horizontal_scroll.setValue(0)

        for item_key, item_status in self.statuses.items():
            self.apply_button_status(
                item_key,
                item_status
            )


def main(args=None):
    rclpy.init(args=args)

    ros_node = SystemFlowNode()

    app = QApplication(sys.argv)

    window = SystemFlowWindow(ros_node)
    window.show()

    ros_spin_timer = QTimer()
    ros_spin_timer.timeout.connect(
        lambda: rclpy.spin_once(
            ros_node,
            timeout_sec=0.0
        )
    )
    ros_spin_timer.start(20)

    exit_code = app.exec_()

    ros_spin_timer.stop()
    ros_node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
