#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rover_monitor/msg/cam_status.hpp>

#include <chrono>
#include <deque>
#include <string>

namespace rover_monitor
{

class CamProbe : public rclcpp::Node
{
public:
  explicit CamProbe(const rclcpp::NodeOptions & options);

private:
  void on_status_timer();
  void on_stream_sample(const rclcpp::Time & now);
  void on_color_image(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void on_depth_image(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void on_imu_check();
  void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void publish_status();

  // Publishers
  rclcpp::Publisher<rover_monitor::msg::CamStatus>::SharedPtr pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr status_timer_;
  // 1 Hz liveness poll for the IMU topic — replaces the prior per-message
  // subscription (which dispatched callbacks at 200 Hz × N cameras even though
  // the body only updated a timestamp).
  rclcpp::TimerBase::SharedPtr imu_check_timer_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Callback group
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  // Frame delta tracking
  rclcpp::Time last_stream_stamp_;
  rclcpp::Time last_color_stamp_;
  rclcpp::Time last_depth_stamp_;
  rclcpp::Time last_odom_stamp_;
  float frame_delta_ms_{0.0f};
  bool first_stream_sample_{true};
  std::deque<rclcpp::Time> stream_timestamps_;

  // Depth FPS rolling average (1-second window)
  std::deque<rclcpp::Time> depth_timestamps_;

  // Depth quality sampling
  int depth_frame_count_{0};
  int depth_quality_sample_interval_{10};
  float depth_quality_sampled_{1.0f};

  // IMU liveness — tracked by polling count_publishers(imu_topic_) at 1 Hz
  std::string imu_topic_;
  rclcpp::Time last_imu_stamp_;
  bool imu_active_{false};
  // imu_timeout_ms_ is no longer used for the liveness decision (imu_active_
  // is set directly by the 1 Hz poll); kept for backward-compatible param.
  int imu_timeout_ms_{2000};

  // Device state
  bool connected_{false};
  bool stream_required_{true};
  bool odom_active_{false};
  std::string camera_id_{"realsense"};

  // Thresholds
  float frame_stutter_threshold_ms_{99.0f};
  int stream_fallback_timeout_ms_{500};
  int odom_timeout_ms_{500};
};

}  // namespace rover_monitor
