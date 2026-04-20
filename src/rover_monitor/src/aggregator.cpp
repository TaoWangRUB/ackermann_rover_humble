#include "rover_monitor/aggregator.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_ros/buffer.h>

namespace rover_monitor
{

Aggregator::Aggregator(const rclcpp::NodeOptions & options)
: Node("monitor_aggregator", options)
{
  cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Parameters
  this->declare_parameter("aggregator.publish_rate_hz", 2.0);
  this->declare_parameter("aggregator.cam_stale_timeout_ms", 1000);
  this->declare_parameter("aggregator.px4_stale_timeout_ms", 1000);
  this->declare_parameter("aggregator.jetson_stale_timeout_ms", 4000);
  this->declare_parameter("aggregator.nav2_stale_timeout_ms", 2000);
  this->declare_parameter("aggregator.slam_frame_id", "map");
  this->declare_parameter("aggregator.slam_child_frame_id", "odom");

  // Alert thresholds
  this->declare_parameter("alerts.cam_stutter_delta_ms", 99.0);
  this->declare_parameter("alerts.cam_depth_quality_warn", 0.5);
  this->declare_parameter("alerts.px4_heartbeat_error_ms", 1000);
  this->declare_parameter("alerts.px4_battery_warn_pct", 40.0);
  this->declare_parameter("alerts.px4_battery_error_pct", 20.0);
  this->declare_parameter("alerts.jetson_cpu_temp_error_c", 80.0);
  this->declare_parameter("alerts.jetson_gpu_temp_error_c", 83.0);
  this->declare_parameter("alerts.jetson_ram_warn_pct", 0.90);
  this->declare_parameter("alerts.jetson_disk_error_gb", 1.0);
  this->declare_parameter("alerts.slam_latency_warn_ms", 2000.0);
  this->declare_parameter("alerts.wifi_signal_warn_dbm", -75.0);

  double rate_hz = this->get_parameter("aggregator.publish_rate_hz").as_double();
  cam_stale_timeout_ms_ = this->get_parameter("aggregator.cam_stale_timeout_ms").as_int();
  px4_stale_timeout_ms_ = this->get_parameter("aggregator.px4_stale_timeout_ms").as_int();
  jetson_stale_timeout_ms_ =
    this->get_parameter("aggregator.jetson_stale_timeout_ms").as_int();
  nav2_stale_timeout_ms_ = this->get_parameter("aggregator.nav2_stale_timeout_ms").as_int();
  slam_frame_id_ = this->get_parameter("aggregator.slam_frame_id").as_string();
  slam_child_frame_id_ =
    this->get_parameter("aggregator.slam_child_frame_id").as_string();

  cam_stutter_delta_ms_ =
    this->get_parameter("alerts.cam_stutter_delta_ms").as_double();
  cam_depth_quality_warn_ =
    this->get_parameter("alerts.cam_depth_quality_warn").as_double();
  px4_heartbeat_error_ms_ =
    this->get_parameter("alerts.px4_heartbeat_error_ms").as_int();
  px4_battery_warn_pct_ =
    this->get_parameter("alerts.px4_battery_warn_pct").as_double();
  px4_battery_error_pct_ =
    this->get_parameter("alerts.px4_battery_error_pct").as_double();
  jetson_cpu_temp_error_c_ =
    this->get_parameter("alerts.jetson_cpu_temp_error_c").as_double();
  jetson_gpu_temp_error_c_ =
    this->get_parameter("alerts.jetson_gpu_temp_error_c").as_double();
  jetson_ram_warn_pct_ =
    this->get_parameter("alerts.jetson_ram_warn_pct").as_double();
  jetson_disk_error_gb_ =
    this->get_parameter("alerts.jetson_disk_error_gb").as_double();
  slam_latency_warn_ms_ =
    this->get_parameter("alerts.slam_latency_warn_ms").as_double();
  wifi_signal_warn_dbm_ =
    this->get_parameter("alerts.wifi_signal_warn_dbm").as_double();

  // Publisher
  pub_ = this->create_publisher<rover_monitor::msg::RoverHealth>("/monitor/health", 1);

  // TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscribers
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group_;

  cam_sub_ = this->create_subscription<rover_monitor::msg::CamStatus>(
    "/monitor/cam", 1,
    std::bind(&Aggregator::on_cam, this, std::placeholders::_1), sub_opts);

  px4_sub_ = this->create_subscription<rover_monitor::msg::Px4Status>(
    "/monitor/px4", 1,
    std::bind(&Aggregator::on_px4, this, std::placeholders::_1), sub_opts);

  jetson_sub_ = this->create_subscription<rover_monitor::msg::JetsonStatus>(
    "/monitor/jetson", 1,
    std::bind(&Aggregator::on_jetson, this, std::placeholders::_1), sub_opts);

  nav2_sub_ = this->create_subscription<rover_monitor::msg::Nav2Status>(
    "/monitor/nav2", 1,
    std::bind(&Aggregator::on_nav2, this, std::placeholders::_1), sub_opts);

  // Initialize stamps
  auto now = this->now();
  px4_stamp_ = now;
  jetson_stamp_ = now;
  nav2_stamp_ = now;

  // 2 Hz merge timer
  auto period = std::chrono::duration<double>(1.0 / rate_hz);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&Aggregator::on_timer, this), cb_group_);

  RCLCPP_INFO(this->get_logger(), "Aggregator initialized (%.1f Hz)", rate_hz);
}

void Aggregator::on_cam(rover_monitor::msg::CamStatus::ConstSharedPtr msg)
{
  cam_cache_[msg->camera_id] = *msg;
  cam_stamps_[msg->camera_id] = this->now();
}

void Aggregator::on_px4(rover_monitor::msg::Px4Status::ConstSharedPtr msg)
{
  px4_cache_ = *msg;
  px4_stamp_ = this->now();
}

void Aggregator::on_jetson(rover_monitor::msg::JetsonStatus::ConstSharedPtr msg)
{
  jetson_cache_ = *msg;
  jetson_stamp_ = this->now();
}

void Aggregator::on_nav2(rover_monitor::msg::Nav2Status::ConstSharedPtr msg)
{
  nav2_cache_ = *msg;
  nav2_stamp_ = this->now();
}

void Aggregator::on_timer()
{
  auto health = std::make_unique<rover_monitor::msg::RoverHealth>();
  health->seq = seq_++;
  health->timestamp = this->now().nanoseconds() / 1000000;

  auto now = this->now();

  // Populate cameras array from cache
  for (auto & [cam_id, cam_status] : cam_cache_) {
    auto it = cam_stamps_.find(cam_id);
    double age_ms = (it != cam_stamps_.end())
      ? (now - it->second).seconds() * 1000.0 : 9999.0;
    if (age_ms < cam_stale_timeout_ms_) {
      health->cameras.push_back(cam_status);
    } else {
      rover_monitor::msg::CamStatus stale;
      stale.camera_id = cam_id;
      stale.connected = false;
      stale.error_code = 1;
      stale.error_msg = "Camera data stale";
      health->cameras.push_back(stale);
    }
  }

  auto px4_age_ms = (now - px4_stamp_).seconds() * 1000.0;
  if (px4_cache_.has_value() && px4_age_ms < px4_stale_timeout_ms_) {
    health->px4 = px4_cache_.value();
  } else {
    health->px4.connected = false;
    health->px4.error_code = 1;
    health->px4.error_msg = "PX4 data stale";
  }

  auto jetson_age_ms = (now - jetson_stamp_).seconds() * 1000.0;
  if (jetson_cache_.has_value() && jetson_age_ms < jetson_stale_timeout_ms_) {
    health->jetson = jetson_cache_.value();
  } else {
    health->jetson.error_code = 7;
    health->jetson.error_msg = "Jetson data stale";
  }

  auto nav2_age_ms = (now - nav2_stamp_).seconds() * 1000.0;
  if (nav2_cache_.has_value() && nav2_age_ms < nav2_stale_timeout_ms_) {
    health->nav2 = nav2_cache_.value();
  } else {
    health->nav2.available = false;
    health->nav2.navigating = false;
    health->nav2.localization_active = false;
    health->nav2.goal_status_label = "STALE";
    health->nav2.feedback_status = "stale";
    health->nav2.error_code = 8;
    health->nav2.error_msg = "Nav2 status stale";
  }

  // Legacy field name retained for wire compatibility.
  // The value represents the age/freshness of the latest map -> odom TF.
  health->slam_latency_ms = compute_slam_tf_age_ms();

  // Alert engine
  health->active_alerts = evaluate_alerts(*health);
  health->overall_health = derive_overall_health(health->active_alerts);

  pub_->publish(std::move(health));
}

float Aggregator::compute_slam_tf_age_ms()
{
  try {
    auto tf = tf_buffer_->lookupTransform(
      slam_frame_id_, slam_child_frame_id_, tf2::TimePointZero);
    auto stamp = rclcpp::Time(tf.header.stamp);
    auto age = (this->now() - stamp).seconds() * 1000.0;
    return static_cast<float>(age);
  } catch (const tf2::TransformException &) {
    return -1.0f;  // map -> odom TF not available
  }
}

std::vector<std::string> Aggregator::evaluate_alerts(
  const rover_monitor::msg::RoverHealth & health)
{
  std::vector<std::string> alerts;

  // Camera alerts (check all cameras)
  for (const auto & cam : health.cameras) {
    std::string prefix = cam.camera_id.empty() ? "CAM" : cam.camera_id;
    if (!cam.connected) {
      alerts.push_back(prefix + "_DISCONNECTED");
    }
    if (cam.connected && cam.frame_delta_ms > cam_stutter_delta_ms_) {
      alerts.push_back(prefix + "_STUTTER");
    }
    if (cam.connected &&
      cam.depth_quality_sampled < cam_depth_quality_warn_ &&
      cam.depth_quality_sampled >= 0.0f)
    {
      alerts.push_back(prefix + "_DEPTH_QUALITY");
    }
  }

  // PX4 alerts
  if (health.px4.heartbeat_age_ms > px4_heartbeat_error_ms_) {
    alerts.push_back("PX4_HEARTBEAT_LOST");
  }
  if (health.px4.battery_remaining_pct < px4_battery_error_pct_) {
    alerts.push_back("PX4_BATTERY_CRITICAL");
  } else if (health.px4.battery_remaining_pct < px4_battery_warn_pct_) {
    alerts.push_back("PX4_BATTERY_LOW");
  }

  // Jetson alerts
  if (health.jetson.temp_cpu_c > jetson_cpu_temp_error_c_) {
    alerts.push_back("JETSON_CPU_HOT");
  }
  if (health.jetson.is_thermal_throttled) {
    alerts.push_back("HW_THROTTLE_THERMAL");
  }
  if (health.jetson.is_power_throttled) {
    alerts.push_back("HW_THROTTLE_POWER");
  }
  if (health.jetson.ram_total_mb > 0 &&
    static_cast<float>(health.jetson.ram_used_mb) /
    static_cast<float>(health.jetson.ram_total_mb) > jetson_ram_warn_pct_)
  {
    alerts.push_back("JETSON_RAM_HIGH");
  }
  if (health.jetson.disk_free_gb >= 0.0f &&
    health.jetson.disk_free_gb < jetson_disk_error_gb_)
  {
    alerts.push_back("JETSON_DISK_LOW");
  }

  // SLAM alert
  if (health.slam_latency_ms > slam_latency_warn_ms_ && health.slam_latency_ms >= 0.0f) {
    alerts.push_back("SLAM_LATE");
  }

  // WiFi alert
  if (health.jetson.wifi_signal_dbm < wifi_signal_warn_dbm_ &&
    health.jetson.wifi_signal_dbm != 0.0f)
  {
    alerts.push_back("NET_DROP");
  }

  if (health.nav2.navigating && health.nav2.error_code != 0) {
    alerts.push_back("NAV2_GOAL_ERROR");
  }

  return alerts;
}

std::string Aggregator::derive_overall_health(const std::vector<std::string> & alerts)
{
  if (alerts.empty()) { return "OK"; }

  // ERROR-level alerts (suffix-matched for camera alerts with dynamic prefix)
  static const std::vector<std::string> error_suffixes = {
    "_DISCONNECTED", "_HEARTBEAT_LOST", "_BATTERY_CRITICAL",
    "_CPU_HOT", "_THROTTLE_THERMAL", "_THROTTLE_POWER",
    "_DISK_LOW"
  };

  for (const auto & alert : alerts) {
    for (const auto & suffix : error_suffixes) {
      if (alert.size() >= suffix.size() &&
        alert.compare(alert.size() - suffix.size(), suffix.size(), suffix) == 0)
      {
        return "ERROR";
      }
    }
  }
  return "WARN";
}

}  // namespace rover_monitor

RCLCPP_COMPONENTS_REGISTER_NODE(rover_monitor::Aggregator)
