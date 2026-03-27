#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rover_monitor/msg/rover_health.hpp>

#include <mqtt/async_client.h>

#include <memory>
#include <string>

namespace rover_monitor
{

class TelemetryPublisher : public rclcpp::Node
{
public:
  explicit TelemetryPublisher(const rclcpp::NodeOptions & options);
  ~TelemetryPublisher() override;

private:
  void on_health(rover_monitor::msg::RoverHealth::ConstSharedPtr msg);
  void connect_mqtt();

  // ROS subscriber
  rclcpp::Subscription<rover_monitor::msg::RoverHealth>::SharedPtr health_sub_;

  // Callback group
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  // MQTT client
  std::shared_ptr<mqtt::async_client> mqtt_client_;

  // Reconnect timer
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  // Config
  std::string broker_host_{"192.168.1.100"};
  int broker_port_{1883};
  int reconnect_delay_s_{5};
  int telemetry_qos_{0};
  int alert_qos_{1};
  std::string client_id_{"xavier_rover_01"};

  bool mqtt_connected_{false};
};

}  // namespace rover_monitor
