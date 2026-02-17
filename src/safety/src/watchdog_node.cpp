// Copyright 2026 The Ackermann Rover Authors
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

class SafetyWatchdogNode : public rclcpp::Node {
public:
  SafetyWatchdogNode()
  : rclcpp::Node("safety_watchdog")
  {
    fault_topic_ = this->declare_parameter<std::string>("fault_topic", "/fault");
    cmd_topic_ = this->declare_parameter<std::string>("ackermann_cmd_topic", "/cmd_ackermann");

    pub_stop_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      cmd_topic_, 10);
    sub_fault_ = this->create_subscription<std_msgs::msg::Bool>(
      fault_topic_, 10,
      std::bind(&SafetyWatchdogNode::onFault, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Safety watchdog started. Monitoring faults on %s",
      fault_topic_.c_str());
  }

private:
  void onFault(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      ackermann_msgs::msg::AckermannDriveStamped stop;
      stop.header.stamp = this->get_clock()->now();
      stop.drive.speed = 0.0f;
      stop.drive.steering_angle = 0.0f;
      pub_stop_->publish(stop);
      RCLCPP_WARN(get_logger(), "Fault detected. Publishing STOP command.");
    }
  }

  std::string fault_topic_;
  std::string cmd_topic_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_fault_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_stop_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SafetyWatchdogNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
