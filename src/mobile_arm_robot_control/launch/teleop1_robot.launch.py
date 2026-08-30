from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


INVERTER_CODE = r'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopLinearInverter(Node):
    def __init__(self):
        super().__init__("teleop1_linear_inverter")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(Twist, "/cmd_vel_teleop_raw", self.cb, 10)
        self.get_logger().info("teleop1_robot started: /cmd_vel_teleop_raw -> /cmd_vel")
        self.get_logger().info("linear.x is inverted. angular.z stays normal.")

    def cb(self, msg):
        out = Twist()

        # نعكس الحركة الأمامية/الخلفية فقط
        out.linear.x = -msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z

        # لا نعكس الدوران
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
'''


def generate_launch_description():
    inverter = ExecuteProcess(
        cmd=["python3", "-c", INVERTER_CODE],
        name="teleop1_linear_inverter",
        output="screen",
        emulate_tty=True
    )

    teleop = ExecuteProcess(
        cmd=[
            "ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard",
            "--ros-args",
            "-r", "/cmd_vel:=/cmd_vel_teleop_raw"
        ],
        name="teleop1_keyboard",
        output="screen",
        emulate_tty=True
    )

    shutdown_when_teleop_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=teleop,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    return LaunchDescription([
        LogInfo(msg="Starting teleop1_robot: inverse linear teleop only"),
        inverter,
        teleop,
        shutdown_when_teleop_exits
    ])
