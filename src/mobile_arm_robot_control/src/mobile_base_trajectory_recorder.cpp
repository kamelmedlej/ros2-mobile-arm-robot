#include <cmath>
#include <fstream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

class MobileBaseTrajectoryRecorder : public rclcpp::Node
{
public:
  MobileBaseTrajectoryRecorder() : Node("mobile_base_trajectory_recorder")
  {
    output_file_path_ = this->declare_parameter<std::string>(
      "output_file",
      "/tmp/mobile_base_trajectory.csv"
    );

    sample_period_ = this->declare_parameter<double>("sample_period", 0.05);

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10,
      std::bind(&MobileBaseTrajectoryRecorder::cmdCallback, this, std::placeholders::_1)
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      std::bind(&MobileBaseTrajectoryRecorder::odomCallback, this, std::placeholders::_1)
    );

    file_.open(output_file_path_, std::ios::out);

    if (!file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open output file: %s", output_file_path_.c_str());
      return;
    }

    file_ << "time,cmd_linear_x,cmd_angular_z,odom_x,odom_y,odom_theta\n";

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(sample_period_),
      std::bind(&MobileBaseTrajectoryRecorder::recordSample, this)
    );

    start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Trajectory recorder started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing: /cmd_vel");
    RCLCPP_INFO(this->get_logger(), "Subscribing: /odom");
    RCLCPP_INFO(this->get_logger(), "Writing CSV file: %s", output_file_path_.c_str());
  }

  ~MobileBaseTrajectoryRecorder()
  {
    if (file_.is_open()) {
      file_.close();
    }
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_linear_x_ = msg->linear.x;
    last_cmd_angular_z_ = msg->angular.z;
    have_cmd_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_x_ = msg->pose.pose.position.x;
    last_odom_y_ = msg->pose.pose.position.y;

    const auto & q_msg = msg->pose.pose.orientation;

    tf2::Quaternion q(
      q_msg.x,
      q_msg.y,
      q_msg.z,
      q_msg.w
    );

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    last_odom_theta_ = yaw;
    have_odom_ = true;
  }

  void recordSample()
  {
    if (!file_.is_open()) {
      return;
    }

    if (!have_odom_) {
      return;
    }

    double t = (this->now() - start_time_).seconds();

    file_
      << t << ","
      << last_cmd_linear_x_ << ","
      << last_cmd_angular_z_ << ","
      << last_odom_x_ << ","
      << last_odom_y_ << ","
      << last_odom_theta_
      << "\n";

    file_.flush();
  }

private:
  std::string output_file_path_;
  double sample_period_;

  std::ofstream file_;

  rclcpp::Time start_time_;

  bool have_cmd_ = false;
  bool have_odom_ = false;

  double last_cmd_linear_x_ = 0.0;
  double last_cmd_angular_z_ = 0.0;

  double last_odom_x_ = 0.0;
  double last_odom_y_ = 0.0;
  double last_odom_theta_ = 0.0;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MobileBaseTrajectoryRecorder>());
  rclcpp::shutdown();
  return 0;
}
