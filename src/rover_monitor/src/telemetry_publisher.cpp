#include "rover_monitor/telemetry_publisher.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include "rover_health.pb.h"

namespace rover_monitor
{

TelemetryPublisher::TelemetryPublisher(const rclcpp::NodeOptions & options)
: Node("telemetry_publisher", options)
{
  cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Parameters
  this->declare_parameter("publisher.broker_host", "192.168.1.100");
  this->declare_parameter("publisher.broker_port", 1883);
  this->declare_parameter("publisher.reconnect_delay_s", 5);
  this->declare_parameter("publisher.telemetry_qos", 0);
  this->declare_parameter("publisher.alert_qos", 1);
  this->declare_parameter("publisher.client_id", "xavier_rover_01");

  broker_host_ = this->get_parameter("publisher.broker_host").as_string();
  broker_port_ = this->get_parameter("publisher.broker_port").as_int();
  reconnect_delay_s_ = this->get_parameter("publisher.reconnect_delay_s").as_int();
  telemetry_qos_ = this->get_parameter("publisher.telemetry_qos").as_int();
  alert_qos_ = this->get_parameter("publisher.alert_qos").as_int();
  client_id_ = this->get_parameter("publisher.client_id").as_string();

  // ROS subscriber
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group_;

  health_sub_ = this->create_subscription<rover_monitor::msg::RoverHealth>(
    "/monitor/health", 1,
    std::bind(&TelemetryPublisher::on_health, this, std::placeholders::_1), sub_opts);

  // MQTT client
  std::string broker_uri = "tcp://" + broker_host_ + ":" + std::to_string(broker_port_);
  mqtt_client_ = std::make_shared<mqtt::async_client>(broker_uri, client_id_);

  // Initial connection
  connect_mqtt();

  RCLCPP_INFO(this->get_logger(), "TelemetryPublisher initialized (broker=%s)",
    broker_uri.c_str());
}

TelemetryPublisher::~TelemetryPublisher()
{
  try {
    if (mqtt_client_ && mqtt_client_->is_connected()) {
      mqtt_client_->disconnect()->wait();
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "MQTT disconnect error: %s", e.what());
  }
}

void TelemetryPublisher::connect_mqtt()
{
  try {
    auto conn_opts = mqtt::connect_options_builder()
      .clean_session(true)
      .automatic_reconnect(
        std::chrono::seconds(1),
        std::chrono::seconds(reconnect_delay_s_))
      .finalize();

    mqtt_client_->connect(conn_opts)->wait();
    mqtt_connected_ = true;
    RCLCPP_INFO(this->get_logger(), "MQTT connected");

    // Cancel reconnect timer if active
    if (reconnect_timer_) {
      reconnect_timer_->cancel();
      reconnect_timer_.reset();
    }
  } catch (const mqtt::exception & e) {
    mqtt_connected_ = false;
    RCLCPP_WARN(this->get_logger(),
      "MQTT connection failed: %s — retrying in %ds", e.what(), reconnect_delay_s_);

    // Schedule reconnect
    if (!reconnect_timer_) {
      reconnect_timer_ = this->create_wall_timer(
        std::chrono::seconds(reconnect_delay_s_),
        [this]() { connect_mqtt(); }, cb_group_);
    }
  }
}

void TelemetryPublisher::on_health(rover_monitor::msg::RoverHealth::ConstSharedPtr msg)
{
  if (!mqtt_connected_ && mqtt_client_ && !mqtt_client_->is_connected()) {
    return;
  }

  // Update connected state from paho's auto-reconnect
  mqtt_connected_ = mqtt_client_->is_connected();
  if (!mqtt_connected_) { return; }

  // Serialize to Protobuf
  rover_monitor_proto::RoverHealth pb;
  pb.set_seq(msg->seq);
  pb.set_timestamp(msg->timestamp);
  pb.set_slam_latency_ms(msg->slam_latency_ms);
  pb.set_overall_health(msg->overall_health);
  for (const auto & alert : msg->active_alerts) {
    pb.add_active_alerts(alert);
  }

  // Camera
  auto * cam = pb.mutable_camera();
  cam->set_camera_id(msg->camera.camera_id);
  cam->set_connected(msg->camera.connected);
  cam->set_frame_delta_ms(msg->camera.frame_delta_ms);
  cam->set_depth_fps(msg->camera.depth_fps);
  cam->set_depth_quality_sampled(msg->camera.depth_quality_sampled);
  cam->set_imu_active(msg->camera.imu_active);
  cam->set_error_code(msg->camera.error_code);
  cam->set_error_msg(msg->camera.error_msg);
  cam->set_timestamp(msg->camera.timestamp);

  // PX4
  auto * px4 = pb.mutable_px4();
  px4->set_connected(msg->px4.connected);
  px4->set_armed(msg->px4.armed);
  px4->set_nav_state(msg->px4.nav_state);
  px4->set_nav_state_label(msg->px4.nav_state_label);
  px4->set_battery_voltage_v(msg->px4.battery_voltage_v);
  px4->set_battery_current_a(msg->px4.battery_current_a);
  px4->set_battery_remaining_pct(msg->px4.battery_remaining_pct);
  px4->set_heartbeat_age_ms(msg->px4.heartbeat_age_ms);
  px4->set_error_code(msg->px4.error_code);
  px4->set_error_msg(msg->px4.error_msg);
  px4->set_timestamp(msg->px4.timestamp);

  // Jetson
  auto * jetson = pb.mutable_jetson();
  for (const auto & cpu : msg->jetson.cpu_usage_pct) {
    jetson->add_cpu_usage_pct(cpu);
  }
  jetson->set_gpu_usage_pct(msg->jetson.gpu_usage_pct);
  jetson->set_ram_used_mb(msg->jetson.ram_used_mb);
  jetson->set_ram_total_mb(msg->jetson.ram_total_mb);
  jetson->set_swap_used_mb(msg->jetson.swap_used_mb);
  jetson->set_disk_free_gb(msg->jetson.disk_free_gb);
  jetson->set_temp_cpu_c(msg->jetson.temp_cpu_c);
  jetson->set_temp_gpu_c(msg->jetson.temp_gpu_c);
  jetson->set_temp_board_c(msg->jetson.temp_board_c);
  jetson->set_is_thermal_throttled(msg->jetson.is_thermal_throttled);
  jetson->set_is_power_throttled(msg->jetson.is_power_throttled);
  jetson->set_power_mode(msg->jetson.power_mode);
  jetson->set_wifi_signal_dbm(msg->jetson.wifi_signal_dbm);
  jetson->set_uptime_s(msg->jetson.uptime_s);
  jetson->set_error_code(msg->jetson.error_code);
  jetson->set_error_msg(msg->jetson.error_msg);
  jetson->set_timestamp(msg->jetson.timestamp);

  // Serialize
  std::string payload;
  pb.SerializeToString(&payload);

  try {
    // Routine telemetry (QoS 0)
    mqtt_client_->publish("rover/health/overall", payload.data(), payload.size(),
      telemetry_qos_, false);

    // Alerts (QoS 1) if any active
    if (!msg->active_alerts.empty()) {
      mqtt_client_->publish("rover/alerts", payload.data(), payload.size(),
        alert_qos_, false);
    }
  } catch (const mqtt::exception & e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "MQTT publish failed: %s", e.what());
    mqtt_connected_ = false;
  }
}

}  // namespace rover_monitor

RCLCPP_COMPONENTS_REGISTER_NODE(rover_monitor::TelemetryPublisher)
