#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

struct TrajectorySample
{
  double time;
  double linear_x;
  double angular_z;
};

class MobileBaseTrajectoryPlayer : public rclcpp::Node
{
public:
  MobileBaseTrajectoryPlayer() : Node("mobile_base_trajectory_player")
  {
    input_file_path_ = this->declare_parameter<std::string>(
      "input_file",
      "/tmp/mobile_base_trajectory.csv");

    cmd_topic_ = this->declare_parameter<std::string>(
      "cmd_topic",
      "/diff_drive_base_controller/cmd_vel_unstamped");

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);

    if (!loadCsv()) {
      RCLCPP_ERROR(this->get_logger(), "Could not load trajectory file.");
      return;
    }

    start_time_ = this->now();

    timer_ = this->create_wall_timer(
      10ms,
      std::bind(&MobileBaseTrajectoryPlayer::playLoop, this));

    RCLCPP_INFO(this->get_logger(), "Trajectory player started.");
    RCLCPP_INFO(this->get_logger(), "Input file: %s", input_file_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", cmd_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Samples: %zu", samples_.size());
  }

private:
  bool loadCsv()
  {
    std::ifstream file(input_file_path_);

    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open input file: %s", input_file_path_.c_str());
      return false;
    }

    std::string line;

    std::getline(file, line);

    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string item;
      std::vector<std::string> cols;

      while (std::getline(ss, item, ',')) {
        cols.push_back(item);
      }

      if (cols.size() < 3) {
        continue;
      }

      try {
        TrajectorySample s;
        s.time = std::stod(cols[0]);
        s.linear_x = std::stod(cols[1]);
        s.angular_z = std::stod(cols[2]);
        samples_.push_back(s);
      } catch (...) {
        continue;
      }
    }

    if (samples_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "CSV file has no valid samples.");
      return false;
    }

    double t0 = samples_.front().time;
    for (auto & s : samples_) {
      s.time -= t0;
    }

    return true;
  }

  void playLoop()
  {
    if (samples_.empty()) {
      publishStop();
      return;
    }

    if (finished_) {
      publishStop();
      return;
    }

    double elapsed = (this->now() - start_time_).seconds();

    while (
      current_index_ + 1 < samples_.size() &&
      samples_[current_index_ + 1].time <= elapsed)
    {
      current_index_++;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = samples_[current_index_].linear_x;
    cmd.angular.z = samples_[current_index_].angular_z;

    publisher_->publish(cmd);

    if (elapsed >= samples_.back().time) {
      finished_ = true;
      publishStop();
      RCLCPP_INFO(this->get_logger(), "Trajectory finished. Robot stopped.");
    }
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    publisher_->publish(stop_cmd);
  }

  std::string input_file_path_;
  std::string cmd_topic_;

  std::vector<TrajectorySample> samples_;
  size_t current_index_ = 0;
  bool finished_ = false;

  rclcpp::Time start_time_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MobileBaseTrajectoryPlayer>());
  rclcpp::shutdown();
  return 0;
}
