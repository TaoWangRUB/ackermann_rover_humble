#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rover_monitor/msg/px4_status.hpp>

#include <string>
#include <vector>

namespace rover_monitor
{

class Px4Probe : public rclcpp::Node
{
public:
  explicit Px4Probe(const rclcpp::NodeOptions & options);

private:
  void on_vehicle_status(px4_msgs::msg::VehicleStatus::ConstSharedPtr msg);
  void on_battery_status(px4_msgs::msg::BatteryStatus::ConstSharedPtr msg);
  void on_heartbeat(px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg);
  void publish_status();

  std::string nav_state_to_label(uint8_t nav_state);

  // Publisher
  rclcpp::Publisher<rover_monitor::msg::Px4Status>::SharedPtr pub_;

  // Subscribers
  std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr> vehicle_status_subs_;
  std::vector<rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr> battery_subs_;
  std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr> heartbeat_subs_;

  // Callback group
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  // Vehicle state
  bool armed_{false};
  bool armable_{false};
  uint8_t nav_state_{0};
  uint8_t nav_state_display_{0};

  // Battery state
  float battery_voltage_v_{0.0f};
  float battery_current_a_{0.0f};
  float battery_remaining_pct_{100.0f};

  // Heartbeat tracking
  rclcpp::Time last_heartbeat_stamp_;
  rclcpp::Time last_any_msg_stamp_;
  bool has_heartbeat_{false};
  bool has_any_msg_{false};

  // Config
  int heartbeat_timeout_ms_{1000};
  int xrce_disconnect_timeout_ms_{2000};
};

}  // namespace rover_monitor
