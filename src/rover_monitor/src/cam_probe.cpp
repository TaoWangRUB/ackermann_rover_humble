#include "rover_monitor/cam_probe.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace rover_monitor
{

CamProbe::CamProbe(const rclcpp::NodeOptions & options)
: Node("cam_probe", options)
{
  // Callback group for deterministic scheduling
  cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Parameters
  this->declare_parameter("probes.cam.camera_id", "realsense");
  this->declare_parameter("probes.cam.color_topic", "/camera/color/image_raw");
  this->declare_parameter("probes.cam.depth_topic", "");
  this->declare_parameter("probes.cam.imu_topic", "/camera/imu");
  this->declare_parameter("probes.cam.odom_topic", "");
  this->declare_parameter("probes.cam.depth_quality_sample_interval", 10);
  this->declare_parameter("probes.cam.imu_timeout_ms", 500);
  this->declare_parameter("probes.cam.frame_stutter_threshold_ms", 99.0);
  this->declare_parameter("probes.cam.stream_fallback_timeout_ms", 500);

  camera_id_ = this->get_parameter("probes.cam.camera_id").as_string();
  depth_quality_sample_interval_ =
    this->get_parameter("probes.cam.depth_quality_sample_interval").as_int();
  imu_timeout_ms_ = this->get_parameter("probes.cam.imu_timeout_ms").as_int();
  frame_stutter_threshold_ms_ =
    this->get_parameter("probes.cam.frame_stutter_threshold_ms").as_double();
  stream_fallback_timeout_ms_ =
    this->get_parameter("probes.cam.stream_fallback_timeout_ms").as_int();

  auto color_topic = this->get_parameter("probes.cam.color_topic").as_string();
  auto depth_topic = this->get_parameter("probes.cam.depth_topic").as_string();
  auto imu_topic = this->get_parameter("probes.cam.imu_topic").as_string();
  auto odom_topic = this->get_parameter("probes.cam.odom_topic").as_string();

  // Publisher (intra-process)
  pub_ = this->create_publisher<rover_monitor::msg::CamStatus>("/monitor/cam", 1);

  // Subscriptions with callback group
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group_;

  color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    color_topic, rclcpp::SensorDataQoS(),
    std::bind(&CamProbe::on_color_image, this, std::placeholders::_1), sub_opts);

  if (!depth_topic.empty()) {
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_topic, rclcpp::SensorDataQoS(),
      std::bind(&CamProbe::on_depth_image, this, std::placeholders::_1), sub_opts);
  }

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(),
    std::bind(&CamProbe::on_imu, this, std::placeholders::_1), sub_opts);

  if (!odom_topic.empty()) {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&CamProbe::on_odom, this, std::placeholders::_1), sub_opts);
  }

  last_imu_stamp_ = this->now();

  RCLCPP_INFO(this->get_logger(), "CamProbe initialized");
}

void CamProbe::on_stream_sample(const rclcpp::Time & now)
{
  connected_ = true;

  stream_timestamps_.push_back(now);
  auto cutoff = now - rclcpp::Duration::from_seconds(1.0);
  while (!stream_timestamps_.empty() && stream_timestamps_.front() < cutoff) {
    stream_timestamps_.pop_front();
  }

  if (first_stream_sample_) {
    first_stream_sample_ = false;
  } else {
    frame_delta_ms_ = static_cast<float>((now - last_stream_stamp_).seconds() * 1000.0);
  }
  last_stream_stamp_ = now;

  publish_status();
}

void CamProbe::on_color_image(sensor_msgs::msg::Image::ConstSharedPtr /*msg*/)
{
  auto now = this->now();
  last_color_stamp_ = now;
  on_stream_sample(now);
}

void CamProbe::on_odom(nav_msgs::msg::Odometry::ConstSharedPtr /*msg*/)
{
  auto now = this->now();
  auto color_age_ms = last_color_stamp_.nanoseconds() > 0 ?
    (now - last_color_stamp_).seconds() * 1000.0 : static_cast<double>(stream_fallback_timeout_ms_ + 1);
  if (color_age_ms > stream_fallback_timeout_ms_) {
    on_stream_sample(now);
  }
}

void CamProbe::on_depth_image(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  auto now = this->now();

  // Rolling 1-second depth FPS
  depth_timestamps_.push_back(now);
  auto cutoff = now - rclcpp::Duration::from_seconds(1.0);
  while (!depth_timestamps_.empty() && depth_timestamps_.front() < cutoff) {
    depth_timestamps_.pop_front();
  }

  // Depth quality: sample 1-in-N frames
  depth_frame_count_++;
  if (depth_frame_count_ % depth_quality_sample_interval_ == 0) {
    if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
      size_t total_pixels = msg->width * msg->height;
      size_t valid_pixels = 0;

      if (msg->encoding == "16UC1") {
        const auto * data = reinterpret_cast<const uint16_t *>(msg->data.data());
        for (size_t i = 0; i < total_pixels; ++i) {
          if (data[i] > 0) { valid_pixels++; }
        }
      } else {  // 32FC1
        const auto * data = reinterpret_cast<const float *>(msg->data.data());
        for (size_t i = 0; i < total_pixels; ++i) {
          if (std::isfinite(data[i]) && data[i] > 0.0f) { valid_pixels++; }
        }
      }

      depth_quality_sampled_ = (total_pixels > 0)
        ? static_cast<float>(valid_pixels) / static_cast<float>(total_pixels)
        : 0.0f;
    }
  }
}

void CamProbe::on_imu(sensor_msgs::msg::Imu::ConstSharedPtr /*msg*/)
{
  last_imu_stamp_ = this->now();
  imu_active_ = true;
}

void CamProbe::publish_status()
{
  auto status = std::make_unique<rover_monitor::msg::CamStatus>();
  status->camera_id = camera_id_;
  status->connected = connected_;
  status->frame_delta_ms = frame_delta_ms_;
  status->stream_fps = static_cast<float>(stream_timestamps_.size());
  status->depth_fps = static_cast<float>(depth_timestamps_.size());
  status->depth_quality_sampled = depth_quality_sampled_;

  // IMU liveness check
  auto imu_age_ms = (this->now() - last_imu_stamp_).seconds() * 1000.0;
  imu_active_ = (imu_age_ms < imu_timeout_ms_);
  status->imu_active = imu_active_;

  // Error code priority: disconnected > frame stutter > depth quality > IMU lost
  if (!connected_) {
    status->error_code = 1;
    status->error_msg = "Device disconnected";
  } else if (frame_delta_ms_ > frame_stutter_threshold_ms_ && !first_stream_sample_) {
    status->error_code = 2;
    status->error_msg = "Camera frame delta exceeds 99 ms (below about 10 FPS)";
  } else if (depth_quality_sampled_ < 0.5f) {
    status->error_code = 3;
    status->error_msg = "Depth fill ratio below 50% (sampled)";
  } else if (!imu_active_) {
    status->error_code = 4;
    status->error_msg = "IMU stream lost";
  } else {
    status->error_code = 0;
    status->error_msg = "";
  }

  status->timestamp = this->now().nanoseconds() / 1000000;  // Unix ms
  pub_->publish(std::move(status));
}

}  // namespace rover_monitor

RCLCPP_COMPONENTS_REGISTER_NODE(rover_monitor::CamProbe)
