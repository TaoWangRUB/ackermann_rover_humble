#include "rover_monitor/jetson_probe.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace rover_monitor
{

JetsonProbe::JetsonProbe(const rclcpp::NodeOptions & options)
: Node("jetson_probe", options)
{
  cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Parameters
  this->declare_parameter("probes.jetson.publish_rate_hz", 0.5);
  this->declare_parameter("probes.jetson.metrics_provider", "");  // "" = auto-detect

  double rate_hz = this->get_parameter("probes.jetson.publish_rate_hz").as_double();
  std::string provider_override =
    this->get_parameter("probes.jetson.metrics_provider").as_string();

  // Publisher
  pub_ = this->create_publisher<rover_monitor::msg::JetsonStatus>("/monitor/jetson", 1);

  // Instantiate platform-specific metrics provider (with this node for parameter access)
  provider_ = SystemMetricsProvider::create(provider_override);

  // Timer at configured rate
  auto period = std::chrono::duration<double>(1.0 / rate_hz);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&JetsonProbe::on_timer, this), cb_group_);

  RCLCPP_INFO(this->get_logger(), "JetsonProbe initialized (%.1f Hz, SystemMetricsProvider: %s)",
    rate_hz, provider_->platform_name().c_str());
}

void JetsonProbe::on_timer()
{
  // Read all metrics from platform-agnostic provider
  auto status = std::make_unique<rover_monitor::msg::JetsonStatus>(
    provider_->read_metrics());

  // Add timestamp and publish
  status->timestamp = this->now().nanoseconds() / 1000000;
  pub_->publish(std::move(status));
}

}  // namespace rover_monitor

RCLCPP_COMPONENTS_REGISTER_NODE(rover_monitor::JetsonProbe)
