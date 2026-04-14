#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rover_monitor/msg/cam_status.hpp>
#include <rover_monitor/msg/px4_status.hpp>
#include <rover_monitor/msg/jetson_status.hpp>
#include <rover_monitor/msg/rover_health.hpp>

#include <optional>
#include <string>
#include <vector>

namespace rover_monitor
{

class Aggregator : public rclcpp::Node
{
public:
  explicit Aggregator(const rclcpp::NodeOptions & options);

private:
  void on_cam(rover_monitor::msg::CamStatus::ConstSharedPtr msg);
  void on_px4(rover_monitor::msg::Px4Status::ConstSharedPtr msg);
  void on_jetson(rover_monitor::msg::JetsonStatus::ConstSharedPtr msg);
  void on_timer();

  float compute_slam_tf_age_ms();
  std::vector<std::string> evaluate_alerts(
    const rover_monitor::msg::RoverHealth & health);
  std::string derive_overall_health(const std::vector<std::string> & alerts);

  // Publisher
  rclcpp::Publisher<rover_monitor::msg::RoverHealth>::SharedPtr pub_;

  // Subscribers
  rclcpp::Subscription<rover_monitor::msg::CamStatus>::SharedPtr cam_sub_;
  rclcpp::Subscription<rover_monitor::msg::Px4Status>::SharedPtr px4_sub_;
  rclcpp::Subscription<rover_monitor::msg::JetsonStatus>::SharedPtr jetson_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Callback group
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  // TF2 for map -> odom transform freshness
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Latest-value caches
  std::optional<rover_monitor::msg::CamStatus> cam_cache_;
  std::optional<rover_monitor::msg::Px4Status> px4_cache_;
  std::optional<rover_monitor::msg::JetsonStatus> jetson_cache_;

  rclcpp::Time cam_stamp_;
  rclcpp::Time px4_stamp_;
  rclcpp::Time jetson_stamp_;

  // Config
  int cam_stale_timeout_ms_{1000};
  int px4_stale_timeout_ms_{1000};
  int jetson_stale_timeout_ms_{4000};
  std::string slam_frame_id_{"map"};
  std::string slam_child_frame_id_{"odom"};

  // Alert thresholds
  float cam_stutter_delta_ms_{66.0f};
  float cam_depth_quality_warn_{0.5f};
  int px4_heartbeat_error_ms_{1000};
  float px4_battery_warn_pct_{40.0f};
  float px4_battery_error_pct_{20.0f};
  float jetson_cpu_temp_error_c_{80.0f};
  float jetson_gpu_temp_error_c_{83.0f};
  float jetson_ram_warn_pct_{0.90f};
  float jetson_disk_error_gb_{1.0f};
  float slam_latency_warn_ms_{100.0f};
  float wifi_signal_warn_dbm_{-75.0f};

  int32_t seq_{0};
};

}  // namespace rover_monitor
