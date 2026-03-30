#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rover_monitor/msg/jetson_status.hpp>
#include "rover_monitor/system_metrics_provider.hpp"

#include <memory>

namespace rover_monitor
{

class JetsonProbe : public rclcpp::Node
{
public:
  explicit JetsonProbe(const rclcpp::NodeOptions & options);

private:
  void on_timer();

  // Platform-agnostic metrics provider (Jetson hardware or x86_mock)
  std::unique_ptr<SystemMetricsProvider> provider_;

  // Publisher
  rclcpp::Publisher<rover_monitor::msg::JetsonStatus>::SharedPtr pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Callback group
  rclcpp::CallbackGroup::SharedPtr cb_group_;
};

}  // namespace rover_monitor
