#!/usr/bin/env bash

cd ~/ros2_ws3_1 || exit 1
source /opt/ros/humble/setup.bash
source ~/ros2_ws3_1/install/setup.bash

pkill -f "teleop1_linear_inverter_1" 2>/dev/null || true

python3 - <<'PY' &
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopLinearInverter(Node):
    def __init__(self):
        super().__init__("teleop1_linear_inverter_1")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(Twist, "/cmd_vel_teleop_raw", self.cb, 10)
        self.get_logger().info("teleop1_robot_1 يعمل الآن")
        self.get_logger().info("i سوف تصبح للأمام إذا كان الروبوت سابقاً يرجع للخلف")

    def cb(self, msg):
        out = Twist()

        # نعكس الأمام والخلف فقط
        out.linear.x = -msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z

        # الدوران لا نلمسه
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z

        self.pub.publish(out)

rclpy.init()
node = TeleopLinearInverter()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
node.destroy_node()
rclpy.shutdown()
PY

INV_PID=$!

cleanup() {
    kill $INV_PID 2>/dev/null || true
    timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
}

trap cleanup EXIT INT TERM

echo "========================================"
echo "teleop1_robot_1 started"
echo "اضغط i للأمام"
echo "اضغط k للتوقف"
echo "اضغط Ctrl+C للخروج"
echo "========================================"

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
--ros-args -r /cmd_vel:=/cmd_vel_teleop_raw
