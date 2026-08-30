#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class MobileBaseVelocityController : public rclcpp::Node
{
public:
  MobileBaseVelocityController() : Node("mobile_base_velocity_controller")
  {
    wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.0545);
    wheel_separation_ = this->declare_parameter<double>("wheel_separation", 0.405);
    cmd_timeout_ = this->declare_parameter<double>("cmd_timeout", 0.5);
    max_wheel_speed_ = this->declare_parameter<double>("max_wheel_speed", 10.0);

    left_multiplier_ = this->declare_parameter<double>("left_multiplier", 1.0);
    right_multiplier_ = this->declare_parameter<double>("right_multiplier", 1.0);

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10,
      std::bind(&MobileBaseVelocityController::cmdCallback, this, std::placeholders::_1)
    );

    wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/wheel_velocity_controller/commands",
      10
    );

    timer_ = this->create_wall_timer(
      20ms,
      std::bind(&MobileBaseVelocityController::update, this)
    );

    last_cmd_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "Mobile base velocity controller started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing: /cmd_vel");
    RCLCPP_INFO(this->get_logger(), "Publishing: /wheel_velocity_controller/commands");
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_ = *msg;
    last_cmd_time_ = std::chrono::steady_clock::now();
    have_cmd_ = true;
  }

  void update()
  {
    double v = 0.0;
    double w = 0.0;

    const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_cmd_time_).count();

    if (have_cmd_ && age <= cmd_timeout_) {
      v = last_cmd_.linear.x;
      w = -last_cmd_.angular.z;
    }

    double left_wheel_speed =
      (v + (w * wheel_separation_ / 2.0)) / wheel_radius_;

    double right_wheel_speed =
      (v - (w * wheel_separation_ / 2.0)) / wheel_radius_;

    left_wheel_speed *= left_multiplier_;
    right_wheel_speed *= right_multiplier_;

    left_wheel_speed = std::clamp(left_wheel_speed, -max_wheel_speed_, max_wheel_speed_);
    right_wheel_speed = std::clamp(right_wheel_speed, -max_wheel_speed_, max_wheel_speed_);

    std_msgs::msg::Float64MultiArray cmd;

    // نفس ترتيب joints في controllers.yaml:
    // front_left, rear_left, front_right, rear_right
    cmd.data = {
      left_wheel_speed,
      left_wheel_speed,
      right_wheel_speed,
      right_wheel_speed
    };

    wheel_pub_->publish(cmd);
  }

  double wheel_radius_;
  double wheel_separation_;
  double cmd_timeout_;
  double max_wheel_speed_;
  double left_multiplier_;
  double right_multiplier_;

  bool have_cmd_ = false;
  geometry_msgs::msg::Twist last_cmd_;
  std::chrono::steady_clock::time_point last_cmd_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MobileBaseVelocityController>());
  rclcpp::shutdown();
  return 0;
}
