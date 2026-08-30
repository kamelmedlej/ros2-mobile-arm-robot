#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


class LidarDecisionNode(Node):

    def __init__(self):
        super().__init__("lidar_decision_node")

        # Lower base LiDAR directions
        self.declare_parameter("front_angle_deg", 180.0)
        self.declare_parameter("left_angle_deg", -90.0)
        self.declare_parameter("right_angle_deg", 90.0)
        self.declare_parameter("back_angle_deg", 0.0)

        self.declare_parameter("front_width_deg", 50.0)
        self.declare_parameter("side_width_deg", 60.0)

        # Base LiDAR distances
        self.declare_parameter("obstacle_distance", 0.80)
        self.declare_parameter("danger_distance", 0.45)

        # Arm sensor
        self.declare_parameter("arm_front_width_deg", 170.0)
        self.declare_parameter("arm_left_center_deg", 45.0)
        self.declare_parameter("arm_right_center_deg", -45.0)
        self.declare_parameter("arm_side_width_deg", 90.0)

        self.declare_parameter("arm_obstacle_distance", 0.80)
        self.declare_parameter("arm_danger_distance", 0.55)
        self.declare_parameter("arm_clear_distance", 0.95)
        self.declare_parameter("arm_ignore_below", 0.10)

        # New lower sensor
        self.declare_parameter("lower_front_width_deg", 115.0)
        self.declare_parameter("lower_left_center_deg", 35.0)
        self.declare_parameter("lower_right_center_deg", -35.0)
        self.declare_parameter("lower_side_width_deg", 60.0)

        self.declare_parameter("lower_obstacle_distance", 0.55)
        self.declare_parameter("lower_danger_distance", 0.30)
        self.declare_parameter("lower_clear_distance", 0.75)
        self.declare_parameter("lower_ignore_below", 0.04)

        # Speeds
        self.declare_parameter("forward_speed", 0.30)
        self.declare_parameter("slow_forward_speed", 0.08)
        self.declare_parameter("turn_speed", 0.58)
        self.declare_parameter("reverse_speed", -0.08)

        # Smooth movement
        self.declare_parameter("max_linear_accel", 0.35)
        self.declare_parameter("max_linear_decel", 1.20)
        self.declare_parameter("max_angular_accel", 1.50)

        self.declare_parameter("base_slow_distance", 1.30)
        self.declare_parameter("arm_slow_distance", 1.20)
        self.declare_parameter("lower_slow_distance", 0.90)

        # Avoidance stability
        self.declare_parameter("minimum_avoid_time", 0.50)
        self.declare_parameter("clear_hold_time", 0.35)
        self.declare_parameter("turn_margin", 0.05)

        self.declare_parameter("turn_sign", 1.0)
        self.declare_parameter("sensor_timeout", 1.0)

        self.scan = None
        self.arm_scan = None
        self.lower_scan = None

        self.last_scan_time = 0.0
        self.last_arm_scan_time = 0.0
        self.last_lower_scan_time = 0.0
        self.last_log_time = 0.0

        self.avoid_mode = None
        self.avoid_turn_wz = 0.0
        self.avoid_direction = ""
        self.avoid_started = 0.0
        self.clear_started = None

        self.current_vx = 0.0
        self.current_wz = 0.0
        self.last_command_time = time.monotonic()

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.arm_scan_sub = self.create_subscription(
            LaserScan,
            "/arm_front_lidar",
            self.arm_scan_callback,
            qos_profile_sensor_data
        )

        self.lower_scan_sub = self.create_subscription(
            LaserScan,
            "/lower_front_lidar",
            self.lower_scan_callback,
            qos_profile_sensor_data
        )

        self.timer = self.create_timer(
            0.10,
            self.control_loop
        )

        self.get_logger().info(
            "Autonomy started: base + arm + lower sensors."
        )

        self.get_logger().info(
            "Smooth acceleration, deceleration and avoidance enabled."
        )

    def scan_callback(self, msg):
        self.scan = msg
        self.last_scan_time = time.monotonic()

    def arm_scan_callback(self, msg):
        self.arm_scan = msg
        self.last_arm_scan_time = time.monotonic()

    def lower_scan_callback(self, msg):
        self.lower_scan = msg
        self.last_lower_scan_time = time.monotonic()

    def sector_min(
        self,
        scan,
        center_deg,
        width_deg,
        ignore_below=0.0
    ):
        center = math.radians(center_deg)
        half_width = math.radians(width_deg / 2.0)

        minimum_allowed = max(
            float(scan.range_min),
            float(ignore_below)
        )

        values = []
        angle = scan.angle_min

        for distance in scan.ranges:
            if (
                math.isfinite(distance)
                and minimum_allowed <= distance <= scan.range_max
            ):
                difference = normalize_angle(angle - center)

                if abs(difference) <= half_width:
                    values.append(distance)

            angle += scan.angle_increment

        if not values:
            return float("inf")

        return min(values)

    def publish_direct(self, vx, wz):
        msg = Twist()

        # Real forward direction is opposite to positive linear.x
        msg.linear.x = -float(vx)
        msg.angular.z = float(wz)

        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.current_vx = 0.0
        self.current_wz = 0.0
        self.publish_direct(0.0, 0.0)

    @staticmethod
    def ramp_value(current, target, acceleration, deceleration, dt):
        changing_direction = current * target < 0.0
        slowing_down = abs(target) < abs(current)

        limit = (
            deceleration
            if changing_direction or slowing_down
            else acceleration
        )

        maximum_change = max(0.0, float(limit)) * dt

        if target > current:
            return min(target, current + maximum_change)

        return max(target, current - maximum_change)

    def send_smooth_command(self, target_vx, target_wz, now):
        dt = now - self.last_command_time
        dt = min(max(dt, 0.02), 0.20)
        self.last_command_time = now

        linear_accel = float(
            self.get_parameter("max_linear_accel").value
        )

        linear_decel = float(
            self.get_parameter("max_linear_decel").value
        )

        angular_accel = float(
            self.get_parameter("max_angular_accel").value
        )

        self.current_vx = self.ramp_value(
            self.current_vx,
            float(target_vx),
            linear_accel,
            linear_decel,
            dt
        )

        self.current_wz = self.ramp_value(
            self.current_wz,
            float(target_wz),
            angular_accel,
            angular_accel,
            dt
        )

        self.publish_direct(
            self.current_vx,
            self.current_wz
        )

    @staticmethod
    def distance_scale(distance, obstacle_distance, slow_distance):
        if not math.isfinite(distance):
            return 1.0

        if distance <= obstacle_distance:
            return 0.0

        if distance >= slow_distance:
            return 1.0

        denominator = slow_distance - obstacle_distance

        if denominator <= 0.0:
            return 1.0

        return (
            distance - obstacle_distance
        ) / denominator

    def choose_turn(
        self,
        sensor_left,
        sensor_right,
        base_left,
        base_right
    ):
        turn_speed = float(
            self.get_parameter("turn_speed").value
        )

        turn_sign = float(
            self.get_parameter("turn_sign").value
        )

        margin = float(
            self.get_parameter("turn_margin").value
        )

        if sensor_left > sensor_right + margin:
            return turn_sign * turn_speed, "left"

        if sensor_right > sensor_left + margin:
            return -turn_sign * turn_speed, "right"

        if base_left > base_right:
            return turn_sign * turn_speed, "left"

        return -turn_sign * turn_speed, "right"

    def enter_avoidance(
        self,
        mode,
        sensor_left,
        sensor_right,
        base_left,
        base_right,
        now
    ):
        (
            self.avoid_turn_wz,
            self.avoid_direction
        ) = self.choose_turn(
            sensor_left,
            sensor_right,
            base_left,
            base_right
        )

        self.avoid_mode = mode
        self.avoid_started = now
        self.clear_started = None

    def control_loop(self):
        now = time.monotonic()

        sensor_timeout = float(
            self.get_parameter("sensor_timeout").value
        )

        # Stop safely until all sensors are available
        if (
            self.scan is None
            or self.arm_scan is None
            or self.lower_scan is None
        ):
            self.stop_robot()
            return

        if (
            now - self.last_scan_time > sensor_timeout
            or now - self.last_arm_scan_time > sensor_timeout
            or now - self.last_lower_scan_time > sensor_timeout
        ):
            self.get_logger().warn(
                "Sensor timeout. Stopping robot."
            )
            self.stop_robot()
            return

        front_angle = float(
            self.get_parameter("front_angle_deg").value
        )

        left_angle = float(
            self.get_parameter("left_angle_deg").value
        )

        right_angle = float(
            self.get_parameter("right_angle_deg").value
        )

        back_angle = float(
            self.get_parameter("back_angle_deg").value
        )

        front_width = float(
            self.get_parameter("front_width_deg").value
        )

        side_width = float(
            self.get_parameter("side_width_deg").value
        )

        obstacle_distance = float(
            self.get_parameter("obstacle_distance").value
        )

        danger_distance = float(
            self.get_parameter("danger_distance").value
        )

        arm_obstacle = float(
            self.get_parameter("arm_obstacle_distance").value
        )

        arm_danger = float(
            self.get_parameter("arm_danger_distance").value
        )

        arm_clear = float(
            self.get_parameter("arm_clear_distance").value
        )

        lower_obstacle = float(
            self.get_parameter("lower_obstacle_distance").value
        )

        lower_danger = float(
            self.get_parameter("lower_danger_distance").value
        )

        lower_clear = float(
            self.get_parameter("lower_clear_distance").value
        )

        # Base sensor
        front = self.sector_min(
            self.scan,
            front_angle,
            front_width
        )

        left = self.sector_min(
            self.scan,
            left_angle,
            side_width
        )

        right = self.sector_min(
            self.scan,
            right_angle,
            side_width
        )

        back = self.sector_min(
            self.scan,
            back_angle,
            side_width
        )

        # Arm sensor
        arm_ignore = float(
            self.get_parameter("arm_ignore_below").value
        )

        arm_front = self.sector_min(
            self.arm_scan,
            0.0,
            float(
                self.get_parameter(
                    "arm_front_width_deg"
                ).value
            ),
            arm_ignore
        )

        arm_left = self.sector_min(
            self.arm_scan,
            float(
                self.get_parameter(
                    "arm_left_center_deg"
                ).value
            ),
            float(
                self.get_parameter(
                    "arm_side_width_deg"
                ).value
            ),
            arm_ignore
        )

        arm_right = self.sector_min(
            self.arm_scan,
            float(
                self.get_parameter(
                    "arm_right_center_deg"
                ).value
            ),
            float(
                self.get_parameter(
                    "arm_side_width_deg"
                ).value
            ),
            arm_ignore
        )

        # Lower sensor
        lower_ignore = float(
            self.get_parameter("lower_ignore_below").value
        )

        lower_front = self.sector_min(
            self.lower_scan,
            0.0,
            float(
                self.get_parameter(
                    "lower_front_width_deg"
                ).value
            ),
            lower_ignore
        )

        lower_left = self.sector_min(
            self.lower_scan,
            float(
                self.get_parameter(
                    "lower_left_center_deg"
                ).value
            ),
            float(
                self.get_parameter(
                    "lower_side_width_deg"
                ).value
            ),
            lower_ignore
        )

        lower_right = self.sector_min(
            self.lower_scan,
            float(
                self.get_parameter(
                    "lower_right_center_deg"
                ).value
            ),
            float(
                self.get_parameter(
                    "lower_side_width_deg"
                ).value
            ),
            lower_ignore
        )

        # Lower obstacle has the highest priority
        if (
            lower_front < lower_obstacle
            and self.avoid_mode != "lower"
        ):
            self.enter_avoidance(
                "lower",
                lower_left,
                lower_right,
                left,
                right,
                now
            )

        # Arm obstacle has the second priority
        elif (
            arm_front < arm_obstacle
            and self.avoid_mode in (None, "base")
        ):
            self.enter_avoidance(
                "arm",
                arm_left,
                arm_right,
                left,
                right,
                now
            )

        # Base obstacle
        elif (
            front < obstacle_distance
            and self.avoid_mode is None
        ):
            self.enter_avoidance(
                "base",
                left,
                right,
                left,
                right,
                now
            )

        action = ""

        if self.avoid_mode is not None:
            if self.avoid_mode == "lower":
                active_distance = lower_front
                danger_limit = lower_danger
                clear_limit = lower_clear

            elif self.avoid_mode == "arm":
                active_distance = arm_front
                danger_limit = arm_danger
                clear_limit = arm_clear

            else:
                active_distance = front
                danger_limit = danger_distance
                clear_limit = obstacle_distance + 0.15

            minimum_avoid_time = float(
                self.get_parameter("minimum_avoid_time").value
            )

            clear_hold_time = float(
                self.get_parameter("clear_hold_time").value
            )

            if active_distance >= clear_limit:
                if self.clear_started is None:
                    self.clear_started = now

                enough_time = (
                    now - self.avoid_started
                    >= minimum_avoid_time
                )

                clear_stable = (
                    now - self.clear_started
                    >= clear_hold_time
                )

                if enough_time and clear_stable:
                    self.avoid_mode = None
                    self.clear_started = None

            else:
                self.clear_started = None

            if self.avoid_mode is not None:
                if active_distance < danger_limit:
                    reverse_speed = float(
                        self.get_parameter(
                            "reverse_speed"
                        ).value
                    )

                    self.send_smooth_command(
                        reverse_speed,
                        self.avoid_turn_wz,
                        now
                    )

                    action = (
                        f"{self.avoid_mode.upper()} DANGER: "
                        f"reverse + turn {self.avoid_direction}"
                    )

                else:
                    self.send_smooth_command(
                        0.0,
                        self.avoid_turn_wz,
                        now
                    )

                    action = (
                        f"{self.avoid_mode.upper()} AVOID: "
                        f"turn {self.avoid_direction}"
                    )

        if self.avoid_mode is None:
            forward_speed = float(
                self.get_parameter("forward_speed").value
            )

            slow_speed = float(
                self.get_parameter(
                    "slow_forward_speed"
                ).value
            )

            base_scale = self.distance_scale(
                front,
                obstacle_distance,
                float(
                    self.get_parameter(
                        "base_slow_distance"
                    ).value
                )
            )

            arm_scale = self.distance_scale(
                arm_front,
                arm_obstacle,
                float(
                    self.get_parameter(
                        "arm_slow_distance"
                    ).value
                )
            )

            lower_scale = self.distance_scale(
                lower_front,
                lower_obstacle,
                float(
                    self.get_parameter(
                        "lower_slow_distance"
                    ).value
                )
            )

            speed_scale = min(
                base_scale,
                arm_scale,
                lower_scale
            )

            target_speed = (
                slow_speed
                + (forward_speed - slow_speed) * speed_scale
            )

            turn_sign = float(
                self.get_parameter("turn_sign").value
            )

            if left < 0.45:
                self.send_smooth_command(
                    min(target_speed, slow_speed),
                    -turn_sign * 0.25,
                    now
                )

                action = "FORWARD SLOW: correcting right"

            elif right < 0.45:
                self.send_smooth_command(
                    min(target_speed, slow_speed),
                    turn_sign * 0.25,
                    now
                )

                action = "FORWARD SLOW: correcting left"

            else:
                self.send_smooth_command(
                    target_speed,
                    0.0,
                    now
                )

                action = (
                    f"FORWARD SMOOTH: "
                    f"target={target_speed:.2f}"
                )

        if now - self.last_log_time >= 0.5:
            self.last_log_time = now

            self.get_logger().info(
                f"base={front:.2f} | "
                f"arm={arm_front:.2f} | "
                f"lower={lower_front:.2f} | "
                f"left={left:.2f} | "
                f"right={right:.2f} | "
                f"cmd_v={self.current_vx:.2f} | "
                f"cmd_w={self.current_wz:.2f} | "
                f"{action}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = LidarDecisionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            node.stop_robot()
            time.sleep(0.1)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
