#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class MobileBaseOdometryNode : public rclcpp::Node
{
public:
  MobileBaseOdometryNode() : Node("mobile_base_odometry_node")
  {
    wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.0545);
    wheel_separation_ = this->declare_parameter<double>("wheel_separation", 0.405);

    odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_link");

    publish_tf_ = this->declare_parameter<bool>("publish_tf", true);

    left_wheel_names_ = this->declare_parameter<std::vector<std::string>>(
      "left_wheel_names",
      {"front_left_wheel_joint", "rear_left_wheel_joint"}
    );

    right_wheel_names_ = this->declare_parameter<std::vector<std::string>>(
      "right_wheel_names",
      {"front_right_wheel_joint", "rear_right_wheel_joint"}
    );

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&MobileBaseOdometryNode::jointStateCallback, this, std::placeholders::_1)
    );

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "Mobile base odometry node started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing: /joint_states");
    RCLCPP_INFO(this->get_logger(), "Publishing: /odom");
    RCLCPP_INFO(this->get_logger(), "TF: %s -> %s", odom_frame_id_.c_str(), base_frame_id_.c_str());
  }

private:
  bool getJointPosition(
    const sensor_msgs::msg::JointState & msg,
    const std::string & joint_name,
    double & position) const
  {
    for (size_t i = 0; i < msg.name.size(); ++i) {
      if (msg.name[i] == joint_name && i < msg.position.size()) {
        position = msg.position[i];
        return true;
      }
    }
    return false;
  }

  bool getAverageWheelPosition(
    const sensor_msgs::msg::JointState & msg,
    const std::vector<std::string> & joint_names,
    double & average_position) const
  {
    double sum = 0.0;
    int count = 0;

    for (const auto & name : joint_names) {
      double position = 0.0;
      if (getJointPosition(msg, name, position)) {
        sum += position;
        count++;
      }
    }

    if (count == 0) {
      return false;
    }

    average_position = sum / static_cast<double>(count);
    return true;
  }

  rclcpp::Time getMessageTime(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    rclcpp::Time stamp(msg->header.stamp);

    if (stamp.nanoseconds() == 0) {
      return this->now();
    }

    return stamp;
  }

  double normalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    double left_position = 0.0;
    double right_position = 0.0;

    bool left_ok = getAverageWheelPosition(*msg, left_wheel_names_, left_position);
    bool right_ok = getAverageWheelPosition(*msg, right_wheel_names_, right_position);

    if (!left_ok || !right_ok) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Cannot find wheel joints in /joint_states."
      );
      return;
    }

    rclcpp::Time current_time = getMessageTime(msg);

    if (!initialized_) {
      last_left_position_ = left_position;
      last_right_position_ = right_position;
      last_time_ = current_time;
      initialized_ = true;

      publishOdometry(current_time, 0.0, 0.0);
      return;
    }

    double dt = (current_time - last_time_).seconds();

    if (dt <= 0.0) {
      return;
    }

    double delta_left_position = left_position - last_left_position_;
    double delta_right_position = right_position - last_right_position_;

    last_left_position_ = left_position;
    last_right_position_ = right_position;
    last_time_ = current_time;    // Use wheel joint increments directly for odometry.
    double delta_left_distance = delta_left_position * wheel_radius_;
    double delta_right_distance = delta_right_position * wheel_radius_;

    double delta_center = (delta_left_distance + delta_right_distance) / 2.0;
    double delta_theta = (delta_right_distance - delta_left_distance) / wheel_separation_;

    double theta_mid = theta_ + delta_theta / 2.0;

    x_ += delta_center * std::cos(theta_mid);
    y_ += delta_center * std::sin(theta_mid);
    theta_ = normalizeAngle(theta_ + delta_theta);

    double linear_velocity = delta_center / dt;
    double angular_velocity = delta_theta / dt;

    publishOdometry(current_time, linear_velocity, angular_velocity);
  }

  void publishOdometry(const rclcpp::Time & stamp, double linear_velocity, double angular_velocity)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);

    nav_msgs::msg::Odometry odom_msg;

    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = linear_velocity;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_velocity;

    odom_msg.pose.covariance[0] = 0.01;
    odom_msg.pose.covariance[7] = 0.01;
    odom_msg.pose.covariance[35] = 0.05;

    odom_msg.twist.covariance[0] = 0.01;
    odom_msg.twist.covariance[35] = 0.05;

    odom_pub_->publish(odom_msg);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;

      tf_msg.header.stamp = stamp;
      tf_msg.header.frame_id = odom_frame_id_;
      tf_msg.child_frame_id = base_frame_id_;

      tf_msg.transform.translation.x = x_;
      tf_msg.transform.translation.y = y_;
      tf_msg.transform.translation.z = 0.0;

      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(tf_msg);
    }
  }

private:
  double wheel_radius_;
  double wheel_separation_;

  std::string odom_frame_id_;
  std::string base_frame_id_;

  bool publish_tf_;

  std::vector<std::string> left_wheel_names_;
  std::vector<std::string> right_wheel_names_;

  bool initialized_ = false;

  double last_left_position_ = 0.0;
  double last_right_position_ = 0.0;

  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;

  rclcpp::Time last_time_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MobileBaseOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
