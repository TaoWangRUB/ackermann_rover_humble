#pragma once

#include <rclcpp/rclcpp.hpp>
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
  void on_color_image(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void on_depth_image(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void on_imu(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void publish_status();

  // Publishers
  rclcpp::Publisher<rover_monitor::msg::CamStatus>::SharedPtr pub_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Callback group
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  // Frame delta tracking
  rclcpp::Time last_color_stamp_;
  float frame_delta_ms_{0.0f};
  bool first_color_frame_{true};
  std::deque<rclcpp::Time> color_timestamps_;

  // Depth FPS rolling average (1-second window)
  std::deque<rclcpp::Time> depth_timestamps_;

  // Depth quality sampling
  int depth_frame_count_{0};
  int depth_quality_sample_interval_{10};
  float depth_quality_sampled_{1.0f};

  // IMU liveness
  rclcpp::Time last_imu_stamp_;
  bool imu_active_{false};
  int imu_timeout_ms_{500};

  // Device state
  bool connected_{false};
  std::string camera_id_{"realsense"};

  // Thresholds
  float frame_stutter_threshold_ms_{99.0f};
};

}  // namespace rover_monitor
