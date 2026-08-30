#!/usr/bin/env python3

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QFileDialog,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)


WORKSPACE = Path.home() / "ros2_ws3_1"
MAPS_DIRECTORY = WORKSPACE / "maps"
DEMO_SCRIPT = WORKSPACE / "demo_robot_1.sh"
AUTONOMY_SCRIPT = WORKSPACE / "autonomy_robot_1.sh"
CAMERA_SCRIPT = WORKSPACE / "kamera_robot_1.sh"


class RobotGuiNode(Node):

    def __init__(self):
        super().__init__("robot_gui_node")

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

    def publish_velocity(self, linear_x, angular_z):
        message = Twist()
        message.linear.x = float(linear_x)
        message.angular.z = float(angular_z)

        self.cmd_vel_publisher.publish(message)

    def publish_stop(self):
        stop_message = Twist()

        # نرسل التوقف عدة مرات لضمان وصول الأمر.
        for _ in range(6):
            self.cmd_vel_publisher.publish(stop_message)

    def node_exists(self, expected_name):
        expected_name = expected_name.lstrip("/")

        try:
            node_names = self.get_node_names()

            return any(
                name.lstrip("/") == expected_name
                for name in node_names
            )

        except Exception:
            return False

    def topic_has_publisher(self, topic_name):
        try:
            publishers = self.get_publishers_info_by_topic(
                topic_name
            )

            return len(publishers) > 0

        except Exception:
            return False


class RobotControlWindow(QMainWindow):

    def __init__(self, ros_node):
        super().__init__()

        self.ros_node = ros_node

        self.demo_process = None
        self.autonomy_process = None
        self.system_flow_process = None
        self.camera_process = None
        self.navigation_process = None
        self.map_save_process = None
        self.pending_map_base = None
        self.selected_map_path = None

        self.current_mode = "STOPPED"

        self.manual_linear_command = 0.0
        self.manual_angular_command = 0.0

        self.setWindowTitle(
            "Mobile Robot Control and Monitoring System"
        )
        self.resize(1000, 760)
        self.setMinimumSize(900, 680)

        self.build_interface()
        self.apply_general_style()

        self.manual_publish_timer = QTimer(self)
        self.manual_publish_timer.timeout.connect(
            self.publish_manual_command
        )

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(
            self.update_system_status
        )
        self.status_timer.start(500)

        self.map_process_timer = QTimer(self)
        self.map_process_timer.timeout.connect(
            self.check_map_save_process
        )
        self.map_process_timer.start(300)

        self.update_system_status()

    def build_interface(self):
        central_widget = QWidget()
        main_layout = QVBoxLayout(central_widget)

        main_layout.setContentsMargins(
            24,
            20,
            24,
            20
        )
        main_layout.setSpacing(16)

        title = QLabel(
            "Mobile Robot Control and Monitoring System"
        )
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(
            "font-size: 27px;"
            "font-weight: bold;"
            "color: #263238;"
            "padding: 8px;"
        )

        subtitle = QLabel(
            "ROS 2 • Gazebo • Autonomous and Manual Control"
        )
        subtitle.setAlignment(Qt.AlignCenter)
        subtitle.setStyleSheet(
            "font-size: 15px;"
            "color: #607d8b;"
            "padding-bottom: 8px;"
        )

        main_layout.addWidget(title)
        main_layout.addWidget(subtitle)

        # بطاقات الحالة في أعلى الواجهة.
        cards_layout = QHBoxLayout()
        cards_layout.setSpacing(12)

        self.ros_status_card = self.create_status_card(
            "ROS 2",
            "CONNECTED",
            "#2e7d32"
        )

        self.simulation_status_card = self.create_status_card(
            "SIMULATION",
            "STOPPED",
            "#c62828"
        )

        self.mode_status_card = self.create_status_card(
            "CONTROL MODE",
            "STOPPED",
            "#455a64"
        )

        cards_layout.addWidget(self.ros_status_card)
        cards_layout.addWidget(
            self.simulation_status_card
        )
        cards_layout.addWidget(self.mode_status_card)

        main_layout.addLayout(cards_layout)

        # قسم تشغيل النظام.
        system_group = QGroupBox(
            "System Control"
        )
        system_layout = QGridLayout(system_group)
        system_layout.setSpacing(12)

        self.start_simulation_button = self.create_button(
            "START SIMULATION",
            "#1976d2"
        )
        self.start_simulation_button.clicked.connect(
            self.start_simulation
        )

        self.manual_mode_button = self.create_button(
            "MANUAL MODE",
            "#00838f"
        )
        self.manual_mode_button.clicked.connect(
            self.enter_manual_mode
        )

        self.start_autonomy_button = self.create_button(
            "AUTONOMOUS MODE",
            "#388e3c"
        )
        self.start_autonomy_button.clicked.connect(
            self.start_autonomy
        )

        self.stop_robot_button = self.create_button(
            "STOP ROBOT",
            "#ef6c00"
        )
        self.stop_robot_button.clicked.connect(
            self.stop_robot
        )

        self.stop_all_button = self.create_button(
            "STOP ALL",
            "#c62828"
        )
        self.stop_all_button.clicked.connect(
            self.stop_all
        )

        self.system_flow_button = self.create_button(
            "SYSTEM FLOW",
            "#455a64"
        )
        self.system_flow_button.clicked.connect(
            self.start_system_flow
        )

        self.camera_view_button = self.create_button(
            "CAMERA VIEW",
            "#6a1b9a"
        )
        self.camera_view_button.clicked.connect(
            self.start_camera_view
        )

        system_layout.addWidget(
            self.start_simulation_button,
            0,
            0
        )
        system_layout.addWidget(
            self.manual_mode_button,
            0,
            1
        )
        system_layout.addWidget(
            self.start_autonomy_button,
            0,
            2
        )

        system_layout.addWidget(
            self.stop_robot_button,
            1,
            0
        )
        system_layout.addWidget(
            self.stop_all_button,
            1,
            1
        )
        system_layout.addWidget(
            self.system_flow_button,
            1,
            2
        )

        system_layout.addWidget(
            self.camera_view_button,
            2,
            0,
            1,
            3
        )

        main_layout.addWidget(system_group)

        # قسم إدارة الخرائط.
        map_group = QGroupBox(
            "Map Management"
        )
        map_layout = QGridLayout(map_group)
        map_layout.setSpacing(12)

        self.save_map_button = self.create_button(
            "SAVE CURRENT MAP",
            "#7b1fa2"
        )
        self.save_map_button.clicked.connect(
            self.save_current_map
        )

        self.select_map_button = self.create_button(
            "SELECT SAVED MAP",
            "#5e35b1"
        )
        self.select_map_button.clicked.connect(
            self.select_saved_map
        )

        self.start_navigation_button = self.create_button(
            "START NAVIGATION",
            "#1565c0"
        )
        self.start_navigation_button.clicked.connect(
            self.start_navigation
        )

        self.stop_navigation_button = self.create_button(
            "STOP NAVIGATION",
            "#ad1457"
        )
        self.stop_navigation_button.clicked.connect(
            self.stop_navigation
        )

        self.selected_map_label = QLabel(
            "Selected map: None"
        )
        self.selected_map_label.setWordWrap(True)
        self.selected_map_label.setTextInteractionFlags(
            Qt.TextSelectableByMouse
        )
        self.selected_map_label.setStyleSheet(
            "font-size: 14px;"
            "font-weight: bold;"
            "color: #37474f;"
            "background-color: #eceff1;"
            "border: 1px solid #cfd8dc;"
            "border-radius: 7px;"
            "padding: 10px;"
        )

        map_layout.addWidget(
            self.save_map_button,
            0,
            0
        )
        map_layout.addWidget(
            self.select_map_button,
            0,
            1
        )

        map_layout.addWidget(
            self.start_navigation_button,
            1,
            0
        )
        map_layout.addWidget(
            self.stop_navigation_button,
            1,
            1
        )

        map_layout.addWidget(
            self.selected_map_label,
            2,
            0,
            1,
            2
        )

        main_layout.addWidget(map_group)

        # قسم التحكم اليدوي.
        manual_group = QGroupBox(
            "Manual Robot Control"
        )
        manual_main_layout = QVBoxLayout(
            manual_group
        )
        manual_main_layout.setSpacing(14)

        speed_layout = QHBoxLayout()

        linear_label = QLabel(
            "Linear speed:"
        )

        self.linear_speed_input = QDoubleSpinBox()
        self.linear_speed_input.setRange(
            0.01,
            1.00
        )
        self.linear_speed_input.setSingleStep(
            0.05
        )
        self.linear_speed_input.setDecimals(2)
        self.linear_speed_input.setValue(0.30)
        self.linear_speed_input.setSuffix(" m/s")

        angular_label = QLabel(
            "Angular speed:"
        )

        self.angular_speed_input = QDoubleSpinBox()
        self.angular_speed_input.setRange(
            0.05,
            2.00
        )
        self.angular_speed_input.setSingleStep(
            0.05
        )
        self.angular_speed_input.setDecimals(2)
        self.angular_speed_input.setValue(0.58)
        self.angular_speed_input.setSuffix(" rad/s")

        speed_layout.addStretch()
        speed_layout.addWidget(linear_label)
        speed_layout.addWidget(
            self.linear_speed_input
        )
        speed_layout.addSpacing(30)
        speed_layout.addWidget(angular_label)
        speed_layout.addWidget(
            self.angular_speed_input
        )
        speed_layout.addStretch()

        manual_main_layout.addLayout(speed_layout)

        direction_layout = QGridLayout()
        direction_layout.setHorizontalSpacing(12)
        direction_layout.setVerticalSpacing(12)

        self.forward_button = self.create_manual_button(
            "▲\nFORWARD"
        )
        self.left_button = self.create_manual_button(
            "◀\nLEFT"
        )
        self.manual_stop_button = self.create_manual_button(
            "■\nSTOP",
            "#d32f2f"
        )
        self.right_button = self.create_manual_button(
            "▶\nRIGHT"
        )
        self.backward_button = self.create_manual_button(
            "▼\nBACKWARD"
        )

        # الحركة تستمر أثناء الضغط فقط.
        self.forward_button.pressed.connect(
            self.manual_forward
        )
        self.forward_button.released.connect(
            self.stop_manual_motion
        )

        self.backward_button.pressed.connect(
            self.manual_backward
        )
        self.backward_button.released.connect(
            self.stop_manual_motion
        )

        self.left_button.pressed.connect(
            self.manual_left
        )
        self.left_button.released.connect(
            self.stop_manual_motion
        )

        self.right_button.pressed.connect(
            self.manual_right
        )
        self.right_button.released.connect(
            self.stop_manual_motion
        )

        self.manual_stop_button.clicked.connect(
            self.stop_manual_motion
        )

        direction_layout.setColumnStretch(0, 1)
        direction_layout.setColumnStretch(1, 1)
        direction_layout.setColumnStretch(2, 1)

        direction_layout.addWidget(
            self.forward_button,
            0,
            1
        )
        direction_layout.addWidget(
            self.left_button,
            1,
            0
        )
        direction_layout.addWidget(
            self.manual_stop_button,
            1,
            1
        )
        direction_layout.addWidget(
            self.right_button,
            1,
            2
        )
        direction_layout.addWidget(
            self.backward_button,
            2,
            1
        )

        manual_main_layout.addLayout(
            direction_layout
        )

        self.manual_help_label = QLabel(
            "Select MANUAL MODE, then hold a direction "
            "button to move. Releasing the button stops the robot."
        )
        self.manual_help_label.setAlignment(
            Qt.AlignCenter
        )
        self.manual_help_label.setStyleSheet(
            "font-size: 14px;"
            "color: #607d8b;"
            "padding: 8px;"
        )

        manual_main_layout.addWidget(
            self.manual_help_label
        )

        main_layout.addWidget(manual_group)
        main_layout.addStretch()

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(
            QScrollArea.NoFrame
        )
        scroll_area.setWidget(
            central_widget
        )

        self.setCentralWidget(
            scroll_area
        )

        self.manual_buttons = [
            self.forward_button,
            self.backward_button,
            self.left_button,
            self.right_button,
            self.manual_stop_button,
        ]

        self.set_manual_controls_enabled(False)

    def apply_general_style(self):
        self.setStyleSheet(
            """
            QMainWindow {
                background-color: #f4f6f8;
            }

            QGroupBox {
                font-size: 17px;
                font-weight: bold;
                color: #37474f;
                border: 1px solid #cfd8dc;
                border-radius: 10px;
                margin-top: 14px;
                padding: 14px;
                background-color: white;
            }

            QGroupBox::title {
                subcontrol-origin: margin;
                left: 14px;
                padding: 0 7px;
                background-color: #f4f6f8;
            }

            QLabel {
                font-size: 15px;
            }

            QDoubleSpinBox {
                min-height: 34px;
                min-width: 125px;
                font-size: 15px;
                padding: 3px 8px;
                border: 1px solid #b0bec5;
                border-radius: 6px;
                background-color: white;
            }

            QPushButton:disabled {
                background-color: #cfd8dc;
                color: #78909c;
                border-color: #b0bec5;
            }
            """
        )

    def create_status_card(
        self,
        title,
        value,
        color
    ):
        card = QLabel(
            f"{title}\n{value}"
        )
        card.setAlignment(Qt.AlignCenter)
        card.setMinimumHeight(78)
        card.setStyleSheet(
            f"font-size: 16px;"
            f"font-weight: bold;"
            f"color: white;"
            f"background-color: {color};"
            f"border-radius: 10px;"
            f"padding: 10px;"
        )

        return card

    def update_status_card(
        self,
        card,
        title,
        value,
        color
    ):
        card.setText(
            f"{title}\n{value}"
        )
        card.setStyleSheet(
            f"font-size: 16px;"
            f"font-weight: bold;"
            f"color: white;"
            f"background-color: {color};"
            f"border-radius: 10px;"
            f"padding: 10px;"
        )

    def create_button(
        self,
        text,
        color
    ):
        button = QPushButton(text)
        button.setMinimumHeight(58)
        button.setStyleSheet(
            f"font-size: 16px;"
            f"font-weight: bold;"
            f"color: white;"
            f"background-color: {color};"
            f"border: none;"
            f"border-radius: 8px;"
            f"padding: 8px;"
        )

        return button

    def create_manual_button(
        self,
        text,
        color="#546e7a"
    ):
        button = QPushButton(text)
        button.setMinimumSize(150, 78)
        button.setStyleSheet(
            f"font-size: 16px;"
            f"font-weight: bold;"
            f"color: white;"
            f"background-color: {color};"
            f"border: 2px solid #37474f;"
            f"border-radius: 10px;"
        )

        return button

    def show_error(self, message):
        QMessageBox.critical(
            self,
            "Error",
            message
        )

    def simulation_available(self):
        demo_running = (
            self.demo_process is not None
            and self.demo_process.poll() is None
        )

        navigation_running = (
            self.navigation_process is not None
            and self.navigation_process.poll() is None
        )

        clock_running = self.ros_node.topic_has_publisher(
            "/clock"
        )

        controller_running = self.ros_node.node_exists(
            "mobile_base_velocity_controller"
        )

        external_simulation_running = (
            clock_running
            and controller_running
        )

        return (
            demo_running
            or navigation_running
            or external_simulation_running
        )


    def autonomy_available(self):
        process_running = (
            self.autonomy_process is not None
            and self.autonomy_process.poll() is None
        )

        node_running = self.ros_node.node_exists(
            "lidar_decision_node"
        )

        return process_running or node_running

    def set_control_mode(self, mode):
        self.current_mode = mode

        if mode == "MANUAL":
            color = "#00838f"
            self.set_manual_controls_enabled(True)

        elif mode == "AUTONOMOUS":
            color = "#388e3c"
            self.set_manual_controls_enabled(False)

        elif mode == "NAVIGATION":
            color = "#1565c0"
            self.set_manual_controls_enabled(False)

        else:
            color = "#455a64"
            self.set_manual_controls_enabled(False)

        self.update_status_card(
            self.mode_status_card,
            "CONTROL MODE",
            mode,
            color
        )

    def set_manual_controls_enabled(
        self,
        enabled
    ):
        for button in self.manual_buttons:
            button.setEnabled(enabled)

    def start_script(self, script_path):
        if not script_path.exists():
            raise FileNotFoundError(
                f"Script not found:\n{script_path}"
            )

        return subprocess.Popen(
            [str(script_path)],
            cwd=str(WORKSPACE),
            start_new_session=True
        )

    def start_simulation(self):
        if self.simulation_available():
            QMessageBox.information(
                self,
                "Simulation",
                "Simulation is already running."
            )
            return

        try:
            self.demo_process = self.start_script(
                DEMO_SCRIPT
            )

            self.update_status_card(
                self.simulation_status_card,
                "SIMULATION",
                "STARTING",
                "#1976d2"
            )

            self.ros_node.get_logger().info(
                "demo_robot_1 started from GUI"
            )

        except Exception as error:
            self.show_error(str(error))

    def enter_manual_mode(self):
        if self.navigation_available():
            QMessageBox.warning(
                self,
                "Navigation is running",
                (
                    "Stop Navigation first, then start "
                    "the normal simulation for Manual Mode."
                )
            )
            return

        if not self.simulation_available():
            QMessageBox.warning(
                self,
                "Simulation is not running",
                "Start the simulation before enabling manual control."
            )
            return

        # لا نسمح بوجود التحكم الذاتي واليدوي معًا.
        self.stop_autonomy_process()
        self.stop_manual_motion()

        self.set_control_mode(
            "MANUAL"
        )

        self.ros_node.get_logger().info(
            "Manual control mode enabled"
        )

    def start_autonomy(self):
        if self.navigation_available():
            QMessageBox.warning(
                self,
                "Navigation is running",
                (
                    "Stop Navigation first, then start "
                    "the normal simulation for Autonomous Mode."
                )
            )
            return

        if not self.simulation_available():
            QMessageBox.warning(
                self,
                "Simulation is not running",
                "Start the simulation before enabling autonomy."
            )
            return

        if self.autonomy_available():
            self.stop_manual_motion()
            self.set_control_mode(
                "AUTONOMOUS"
            )

            QMessageBox.information(
                self,
                "Autonomy",
                "Autonomous mode is already running."
            )
            return

        self.stop_manual_motion()
        self.set_manual_controls_enabled(False)

        try:
            self.autonomy_process = self.start_script(
                AUTONOMY_SCRIPT
            )

            self.set_control_mode(
                "AUTONOMOUS"
            )

            self.ros_node.get_logger().info(
                "autonomy_robot_1 started from GUI"
            )

        except Exception as error:
            self.set_control_mode("STOPPED")
            self.show_error(str(error))

    def start_system_flow(self):
        if (
            self.system_flow_process is not None
            and self.system_flow_process.poll() is None
        ):
            QMessageBox.information(
                self,
                "System Flow",
                "System Flow window is already open."
            )
            return

        try:
            self.system_flow_process = subprocess.Popen(
                [
                    "ros2",
                    "run",
                    "mobile_arm_robot_gui",
                    "system_flow",
                ],
                cwd=str(WORKSPACE),
                start_new_session=True
            )

        except Exception as error:
            self.show_error(str(error))

    def start_camera_view(self):
        if not self.simulation_available():
            QMessageBox.warning(
                self,
                "Simulation is not running",
                "Start the simulation before opening the camera."
            )
            return

        if (
            self.camera_process is not None
            and self.camera_process.poll() is None
        ):
            QMessageBox.information(
                self,
                "Camera View",
                "The camera window is already open."
            )
            return

        try:
            if CAMERA_SCRIPT.exists():
                self.camera_process = subprocess.Popen(
                    [str(CAMERA_SCRIPT)],
                    cwd=str(WORKSPACE),
                    start_new_session=True
                )
            else:
                self.camera_process = subprocess.Popen(
                    [
                        "ros2",
                        "run",
                        "rqt_image_view",
                        "rqt_image_view",
                        "/robot_camera/image",
                    ],
                    cwd=str(WORKSPACE),
                    start_new_session=True
                )

            self.ros_node.get_logger().info(
                "Robot camera window opened"
            )

        except Exception as error:
            self.show_error(str(error))

    def navigation_available(self):
        process_running = (
            self.navigation_process is not None
            and self.navigation_process.poll() is None
        )

        nav2_running = self.ros_node.node_exists(
            "bt_navigator"
        )

        return process_running or nav2_running

    def start_navigation(self):
        if self.selected_map_path is None:
            QMessageBox.warning(
                self,
                "No map selected",
                (
                    "Select a saved map before starting "
                    "Navigation."
                )
            )
            return

        map_path = Path(
            self.selected_map_path
        ).expanduser().resolve()

        validation_error = self.validate_map_yaml(
            map_path
        )

        if validation_error is not None:
            QMessageBox.warning(
                self,
                "Invalid map",
                validation_error
            )
            return

        if self.navigation_available():
            QMessageBox.information(
                self,
                "Navigation",
                "Navigation is already running."
            )
            return

        # Stop manual and autonomous commands first.
        self.stop_manual_motion()
        self.stop_autonomy_process()

        # warehouse_full_navigation starts its own Gazebo,
        # robot, controllers, Nav2 and RViz.
        # Therefore the mapping simulation must be stopped.
        self.terminate_process(
            self.demo_process,
            "demo_robot_1"
        )
        self.demo_process = None

        self.ros_node.publish_stop()

        command = [
            "ros2",
            "launch",
            "mobile_arm_robot_navigation",
            "warehouse_full_navigation.launch.py",
            f"map:={map_path}",
        ]

        try:
            self.navigation_process = subprocess.Popen(
                command,
                cwd=str(WORKSPACE),
                start_new_session=True
            )

            self.set_control_mode(
                "NAVIGATION"
            )

            self.update_status_card(
                self.simulation_status_card,
                "NAVIGATION",
                "STARTING",
                "#1565c0"
            )

            self.selected_map_label.setText(
                "Navigation map:\n"
                + str(map_path)
            )

            self.ros_node.get_logger().info(
                f"Navigation started with map: {map_path}"
            )

        except Exception as error:
            self.navigation_process = None
            self.set_control_mode(
                "STOPPED"
            )
            self.show_error(
                str(error)
            )

    def stop_navigation(self):
        self.ros_node.publish_stop()

        self.terminate_process(
            self.navigation_process,
            "Navigation"
        )

        self.navigation_process = None

        if self.current_mode == "NAVIGATION":
            self.set_control_mode(
                "STOPPED"
            )

        self.update_status_card(
            self.simulation_status_card,
            "SIMULATION",
            "STOPPED",
            "#c62828"
        )

        self.ros_node.get_logger().info(
            "Navigation stopped"
        )

    def save_current_map(self):
        if (
            self.map_save_process is not None
            and self.map_save_process.poll() is None
        ):
            QMessageBox.information(
                self,
                "Save Map",
                "A map is already being saved."
            )
            return

        if not self.ros_node.topic_has_publisher(
            "/map"
        ):
            QMessageBox.warning(
                self,
                "Map is not available",
                (
                    "No publisher is currently publishing /map.\n\n"
                    "Start the simulation and SLAM, then move "
                    "the robot before saving the map."
                )
            )
            return

        MAPS_DIRECTORY.mkdir(
            parents=True,
            exist_ok=True
        )

        default_name = (
            "warehouse_map_"
            + time.strftime("%Y%m%d_%H%M%S")
            + ".yaml"
        )

        default_path = (
            MAPS_DIRECTORY
            / default_name
        )

        selected_file, _ = QFileDialog.getSaveFileName(
            self,
            "Choose map name and save location",
            str(default_path),
            "ROS Map YAML (*.yaml);;All Files (*)"
        )

        if not selected_file:
            return

        selected_path = Path(
            selected_file
        ).expanduser()

        if selected_path.suffix.lower() in (
            ".yaml",
            ".yml",
            ".pgm",
            ".png",
        ):
            map_base = selected_path.with_suffix("")
        else:
            map_base = selected_path

        try:
            map_base.parent.mkdir(
                parents=True,
                exist_ok=True
            )

            command = [
                "ros2",
                "run",
                "nav2_map_server",
                "map_saver_cli",
                "-f",
                str(map_base),
            ]

            self.map_save_process = subprocess.Popen(
                command,
                cwd=str(WORKSPACE),
                start_new_session=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True
            )

            self.pending_map_base = map_base

            self.save_map_button.setEnabled(False)
            self.selected_map_label.setText(
                "Saving map:\n"
                + str(map_base)
            )

            self.ros_node.get_logger().info(
                f"Saving map to {map_base}"
            )

        except Exception as error:
            self.map_save_process = None
            self.pending_map_base = None
            self.save_map_button.setEnabled(True)

            self.show_error(
                str(error)
            )

    def check_map_save_process(self):
        process = self.map_save_process

        if process is None:
            return

        return_code = process.poll()

        if return_code is None:
            return

        try:
            output, _ = process.communicate(
                timeout=1
            )
        except Exception:
            output = ""

        map_base = self.pending_map_base

        self.map_save_process = None
        self.pending_map_base = None
        self.save_map_button.setEnabled(True)

        if map_base is None:
            return

        yaml_path = map_base.with_suffix(
            ".yaml"
        )

        pgm_path = map_base.with_suffix(
            ".pgm"
        )

        if (
            return_code == 0
            and yaml_path.exists()
        ):
            self.set_selected_map(
                yaml_path
            )

            image_text = (
                str(pgm_path)
                if pgm_path.exists()
                else "Created by map_saver_cli"
            )

            QMessageBox.information(
                self,
                "Map saved successfully",
                (
                    "The map was saved successfully.\n\n"
                    f"YAML:\n{yaml_path}\n\n"
                    f"Image:\n{image_text}"
                )
            )

            self.ros_node.get_logger().info(
                f"Map saved successfully: {yaml_path}"
            )

        else:
            message = (
                output.strip()
                if output.strip()
                else "map_saver_cli returned an error."
            )

            QMessageBox.critical(
                self,
                "Map saving failed",
                (
                    "The map could not be saved.\n\n"
                    + message[-3000:]
                )
            )

    def select_saved_map(self):
        MAPS_DIRECTORY.mkdir(
            parents=True,
            exist_ok=True
        )

        selected_file, _ = QFileDialog.getOpenFileName(
            self,
            "Select a saved ROS map",
            str(MAPS_DIRECTORY),
            "ROS Map YAML (*.yaml *.yml);;All Files (*)"
        )

        if not selected_file:
            return

        map_path = Path(
            selected_file
        ).expanduser()

        validation_error = self.validate_map_yaml(
            map_path
        )

        if validation_error is not None:
            QMessageBox.warning(
                self,
                "Invalid map",
                validation_error
            )
            return

        self.set_selected_map(
            map_path
        )

        QMessageBox.information(
            self,
            "Map selected",
            (
                "The following map is ready "
                "for Navigation:\n\n"
                f"{map_path}"
            )
        )

    def validate_map_yaml(self, map_path):
        if not map_path.exists():
            return (
                "The selected YAML file does not exist."
            )

        if map_path.suffix.lower() not in (
            ".yaml",
            ".yml",
        ):
            return (
                "Select a ROS map YAML file."
            )

        try:
            content = map_path.read_text(
                encoding="utf-8"
            )
        except Exception as error:
            return (
                "The map file could not be read:\n"
                + str(error)
            )

        image_value = None

        for line in content.splitlines():
            stripped = line.strip()

            if stripped.startswith("image:"):
                image_value = (
                    stripped
                    .split(":", 1)[1]
                    .strip()
                    .strip('"')
                    .strip("'")
                )
                break

        if not image_value:
            return (
                "The YAML file does not contain "
                "an image entry."
            )

        image_path = Path(
            image_value
        )

        if not image_path.is_absolute():
            image_path = (
                map_path.parent
                / image_path
            )

        if not image_path.exists():
            return (
                "The map image referenced by the YAML "
                "file was not found:\n"
                + str(image_path)
            )

        return None

    def set_selected_map(self, map_path):
        self.selected_map_path = Path(
            map_path
        ).resolve()

        self.selected_map_label.setText(
            "Selected map:\n"
            + str(self.selected_map_path)
        )

    def manual_forward(self):
        if self.current_mode != "MANUAL":
            return

        # اتجاه الأمام الحقيقي عند الروبوت يحتاج linear.x سالب.
        self.start_manual_motion(
            -self.linear_speed_input.value(),
            0.0
        )

    def manual_backward(self):
        if self.current_mode != "MANUAL":
            return

        self.start_manual_motion(
            self.linear_speed_input.value(),
            0.0
        )

    def manual_left(self):
        if self.current_mode != "MANUAL":
            return

        self.start_manual_motion(
            0.0,
            self.angular_speed_input.value()
        )

    def manual_right(self):
        if self.current_mode != "MANUAL":
            return

        self.start_manual_motion(
            0.0,
            -self.angular_speed_input.value()
        )

    def start_manual_motion(
        self,
        linear_x,
        angular_z
    ):
        self.manual_linear_command = float(
            linear_x
        )
        self.manual_angular_command = float(
            angular_z
        )

        self.publish_manual_command()

        if not self.manual_publish_timer.isActive():
            self.manual_publish_timer.start(100)

    def publish_manual_command(self):
        if self.current_mode != "MANUAL":
            return

        self.ros_node.publish_velocity(
            self.manual_linear_command,
            self.manual_angular_command
        )

    def stop_manual_motion(self):
        self.manual_publish_timer.stop()

        self.manual_linear_command = 0.0
        self.manual_angular_command = 0.0

        self.ros_node.publish_stop()

    def terminate_process(
        self,
        process,
        process_name
    ):
        if process is None or process.poll() is not None:
            return

        try:
            process_group = os.getpgid(
                process.pid
            )

            os.killpg(
                process_group,
                signal.SIGINT
            )

            try:
                process.wait(timeout=5)

            except subprocess.TimeoutExpired:
                os.killpg(
                    process_group,
                    signal.SIGKILL
                )
                process.wait(timeout=2)

            self.ros_node.get_logger().info(
                f"{process_name} stopped"
            )

        except ProcessLookupError:
            pass

        except Exception as error:
            self.ros_node.get_logger().error(
                f"Could not stop {process_name}: {error}"
            )

    def stop_autonomy_process(self):
        self.terminate_process(
            self.autonomy_process,
            "autonomy_robot_1"
        )

        self.autonomy_process = None

    def stop_robot(self):
        self.stop_manual_motion()
        self.stop_autonomy_process()

        self.ros_node.publish_stop()
        self.set_control_mode(
            "STOPPED"
        )

    def stop_all(self):
        self.stop_robot()

        self.terminate_process(
            self.navigation_process,
            "Navigation"
        )
        self.navigation_process = None

        self.terminate_process(
            self.demo_process,
            "demo_robot_1"
        )
        self.demo_process = None

        self.terminate_process(
            self.system_flow_process,
            "system_flow"
        )
        self.system_flow_process = None

        self.terminate_process(
            self.camera_process,
            "camera_view"
        )
        self.camera_process = None

        self.set_control_mode(
            "STOPPED"
        )

        self.update_status_card(
            self.simulation_status_card,
            "SIMULATION",
            "STOPPED",
            "#c62828"
        )

    def update_system_status(self):
        simulation_running = (
            self.simulation_available()
        )

        autonomy_running = (
            self.autonomy_available()
        )

        navigation_running = (
            self.navigation_available()
        )

        navigation_running = (
            self.navigation_available()
        )

        if simulation_running:
            self.update_status_card(
                self.simulation_status_card,
                "SIMULATION",
                "RUNNING",
                "#2e7d32"
            )
        else:
            self.update_status_card(
                self.simulation_status_card,
                "SIMULATION",
                "STOPPED",
                "#c62828"
            )

            if self.current_mode != "STOPPED":
                self.stop_manual_motion()
                self.set_control_mode(
                    "STOPPED"
                )

        if (
            self.current_mode == "AUTONOMOUS"
            and not autonomy_running
        ):
            self.set_control_mode(
                "STOPPED"
            )
        if (
            self.current_mode == "NAVIGATION"
            and not navigation_running
        ):
            self.navigation_process = None
            self.set_control_mode(
                "STOPPED"
            )


    def closeEvent(self, event):
        answer = QMessageBox.question(
            self,
            "Close interface",
            "Stop all robot processes and close the interface?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes
        )

        if answer == QMessageBox.Yes:
            self.status_timer.stop()
            self.map_process_timer.stop()
            self.stop_all()
            event.accept()
        else:
            event.ignore()


def main(args=None):
    rclpy.init(args=args)

    ros_node = RobotGuiNode()

    app = QApplication(sys.argv)

    window = RobotControlWindow(
        ros_node
    )
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
