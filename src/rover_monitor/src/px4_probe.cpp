#include "rover_monitor/px4_probe.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace rover_monitor
{

Px4Probe::Px4Probe(const rclcpp::NodeOptions & options)
: Node("px4_probe", options)
{
  cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Parameters
  this->declare_parameter("probes.px4.vehicle_status_topic", "/fmu/out/vehicle_status");
  this->declare_parameter("probes.px4.battery_status_topic", "/fmu/out/battery_status");
  this->declare_parameter("probes.px4.heartbeat_topic", "/fmu/out/vehicle_odometry");
  this->declare_parameter("probes.px4.heartbeat_timeout_ms", 1000);
  this->declare_parameter("probes.px4.xrce_disconnect_timeout_ms", 2000);

  heartbeat_timeout_ms_ = this->get_parameter("probes.px4.heartbeat_timeout_ms").as_int();
  xrce_disconnect_timeout_ms_ =
    this->get_parameter("probes.px4.xrce_disconnect_timeout_ms").as_int();

  auto vehicle_status_topic =
    this->get_parameter("probes.px4.vehicle_status_topic").as_string();
  auto battery_topic =
    this->get_parameter("probes.px4.battery_status_topic").as_string();
  auto heartbeat_topic =
    this->get_parameter("probes.px4.heartbeat_topic").as_string();

  // Publisher
  pub_ = this->create_publisher<rover_monitor::msg::Px4Status>("/monitor/px4", 1);

  // QoS for PX4 topics: best-effort, keep last
  rclcpp::QoS px4_qos(10);
  px4_qos.best_effort();

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group_;

  vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
    vehicle_status_topic, px4_qos,
    std::bind(&Px4Probe::on_vehicle_status, this, std::placeholders::_1), sub_opts);

  battery_sub_ = this->create_subscription<px4_msgs::msg::BatteryStatus>(
    battery_topic, px4_qos,
    std::bind(&Px4Probe::on_battery_status, this, std::placeholders::_1), sub_opts);

  heartbeat_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    heartbeat_topic, px4_qos,
    std::bind(&Px4Probe::on_heartbeat, this, std::placeholders::_1), sub_opts);

  last_heartbeat_stamp_ = this->now();
  last_any_msg_stamp_ = this->now();

  RCLCPP_INFO(this->get_logger(), "Px4Probe initialized");
}

void Px4Probe::on_vehicle_status(px4_msgs::msg::VehicleStatus::ConstSharedPtr msg)
{
  armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
  nav_state_ = msg->nav_state;
  nav_state_display_ = msg->nav_state_display;
  last_any_msg_stamp_ = this->now();
  publish_status();
}

void Px4Probe::on_battery_status(px4_msgs::msg::BatteryStatus::ConstSharedPtr msg)
{
  battery_voltage_v_ = msg->voltage_v;
  battery_current_a_ = msg->current_a;
  battery_remaining_pct_ = msg->remaining * 100.0f;  // PX4 reports 0.0-1.0
  last_any_msg_stamp_ = this->now();
  publish_status();
}

void Px4Probe::on_heartbeat(px4_msgs::msg::VehicleOdometry::ConstSharedPtr /*msg*/)
{
  last_heartbeat_stamp_ = this->now();
  last_any_msg_stamp_ = this->now();
  has_heartbeat_ = true;
  publish_status();
}

std::string Px4Probe::nav_state_to_label(uint8_t nav_state)
{
  switch (nav_state) {
    case 0: return "MANUAL";
    case 1: return "ALTCTL";
    case 2: return "POSCTL";
    case 3: return "AUTO_MISSION";
    case 4: return "AUTO_LOITER";
    case 5: return "AUTO_RTL";
    case 14: return "OFFBOARD";
    case 17: return "AUTO_TAKEOFF";
    case 18: return "AUTO_LAND";
    case 19: return "AUTO_FOLLOW_TARGET";
    case 20: return "AUTO_PRECLAND";
    case 21: return "ORBIT";
    case 22: return "AUTO_VTOL_TAKEOFF";
    case 23: return "EXTERNAL1";
    case 24: return "EXTERNAL2";
    case 25: return "EXTERNAL3";
    case 26: return "EXTERNAL4";
    case 27: return "EXTERNAL5";
    case 28: return "EXTERNAL6";
    case 29: return "EXTERNAL7";
    case 30: return "EXTERNAL8";
    default: return "UNKNOWN(" + std::to_string(nav_state) + ")";
  }
}

void Px4Probe::publish_status()
{
  auto status = std::make_unique<rover_monitor::msg::Px4Status>();

  auto heartbeat_age_ms = static_cast<int32_t>(
    (this->now() - last_heartbeat_stamp_).seconds() * 1000.0);
  auto any_msg_age_ms = static_cast<int32_t>(
    (this->now() - last_any_msg_stamp_).seconds() * 1000.0);

  bool connected = has_heartbeat_ && (heartbeat_age_ms < heartbeat_timeout_ms_);

  status->connected = connected;
  status->armed = armed_;
  // nav_state_display is the user-visible active mode in PX4. This is the
  // field that reflects custom/external modes instead of the internal fallback.
  status->nav_state = nav_state_display_;
  status->nav_state_label = nav_state_to_label(nav_state_display_);
  status->battery_voltage_v = battery_voltage_v_;
  status->battery_current_a = battery_current_a_;
  status->battery_remaining_pct = battery_remaining_pct_;
  status->heartbeat_age_ms = heartbeat_age_ms;

  // Error code priority: heartbeat lost > XRCE disconnect > battery critical > battery low
  if (!has_heartbeat_ || heartbeat_age_ms >= heartbeat_timeout_ms_) {
    status->error_code = 1;
    status->error_msg = "PX4 uORB heartbeat lost";
  } else if (any_msg_age_ms >= xrce_disconnect_timeout_ms_) {
    status->error_code = 4;
    status->error_msg = "XRCE-DDS agent disconnect";
  } else if (battery_remaining_pct_ < 20.0f) {
    status->error_code = 2;
    status->error_msg = "Battery critical — return to base";
  } else if (battery_remaining_pct_ < 40.0f) {
    status->error_code = 3;
    status->error_msg = "Battery low";
  } else {
    status->error_code = 0;
    status->error_msg = "";
  }

  status->timestamp = this->now().nanoseconds() / 1000000;
  pub_->publish(std::move(status));
}

}  // namespace rover_monitor

RCLCPP_COMPONENTS_REGISTER_NODE(rover_monitor::Px4Probe)
