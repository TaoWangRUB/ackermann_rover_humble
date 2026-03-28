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
  this->declare_parameter("publisher.cmd_qos", 2);
  this->declare_parameter("publisher.client_id", "xavier_rover_01");
  this->declare_parameter("publisher.dedup_eviction_s", 30);

  broker_host_ = this->get_parameter("publisher.broker_host").as_string();
  broker_port_ = this->get_parameter("publisher.broker_port").as_int();
  reconnect_delay_s_ = this->get_parameter("publisher.reconnect_delay_s").as_int();
  telemetry_qos_ = this->get_parameter("publisher.telemetry_qos").as_int();
  alert_qos_ = this->get_parameter("publisher.alert_qos").as_int();
  cmd_qos_ = this->get_parameter("publisher.cmd_qos").as_int();
  client_id_ = this->get_parameter("publisher.client_id").as_string();
  dedup_eviction_s_ = this->get_parameter("publisher.dedup_eviction_s").as_int();

  // ROS subscriber for health telemetry
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group_;

  health_sub_ = this->create_subscription<rover_monitor::msg::RoverHealth>(
    "/monitor/health", 1,
    std::bind(&TelemetryPublisher::on_health, this, std::placeholders::_1), sub_opts);

  // PX4 vehicle command publisher
  vehicle_cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", 10);

  // E-stop twist publisher (zero velocity)
  twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/cmd_vel", 10);

  // Dedup eviction timer (every 10s, clean entries older than dedup_eviction_s_)
  dedup_timer_ = this->create_wall_timer(
    std::chrono::seconds(10),
    [this]() {
      std::lock_guard<std::mutex> lock(dedup_mutex_);
      auto now = std::chrono::steady_clock::now();
      for (auto it = seen_cmds_.begin(); it != seen_cmds_.end(); ) {
        auto age = std::chrono::duration_cast<std::chrono::seconds>(
          now - it->second).count();
        if (age > dedup_eviction_s_) {
          it = seen_cmds_.erase(it);
        } else {
          ++it;
        }
      }
    }, cb_group_);

  // Command drain timer — moves commands from Paho thread queue to executor thread
  cmd_drain_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    [this]() {
      std::queue<PendingAck> acks;
      std::queue<PendingCmd> cmds;
      {
        std::lock_guard<std::mutex> lock(cmd_queue_mutex_);
        std::swap(acks, ack_queue_);
        std::swap(cmds, cmd_queue_);
      }
      // Send queued ACKs first (ACK_RECEIVED before dispatch)
      while (!acks.empty()) {
        auto & a = acks.front();
        send_ack(a.cmd_id, a.cmd_type, a.status, a.message);
        acks.pop();
      }
      // Then dispatch commands
      while (!cmds.empty()) {
        auto & c = cmds.front();
        dispatch_command(c.cmd_id, c.cmd_type, c.payload);
        cmds.pop();
      }
    }, cb_group_);

  // MQTT client
  std::string broker_uri = "tcp://" + broker_host_ + ":" + std::to_string(broker_port_);
  mqtt_client_ = std::make_shared<mqtt::async_client>(broker_uri, client_id_);

  // Set MQTT message callback for inbound commands
  mqtt_client_->set_message_callback(
    [this](mqtt::const_message_ptr msg) {
      on_mqtt_message(msg);
    });

  // Set connected callback to subscribe to command topics on (re)connect
  mqtt_client_->set_connected_handler(
    [this](const std::string & /*cause*/) {
      RCLCPP_INFO(this->get_logger(), "MQTT (re)connected — subscribing to command topics");
      try {
        mqtt_client_->subscribe("rover/cmd/goal", cmd_qos_);
        mqtt_client_->subscribe("rover/cmd/arm", cmd_qos_);
        mqtt_client_->subscribe("rover/cmd/mode", cmd_qos_);
        mqtt_client_->subscribe("rover/cmd/estop", cmd_qos_);
        mqtt_client_->subscribe("rover/cmd/disarm", cmd_qos_);
        mqtt_client_->subscribe("rover/cmd/cancel_goal", cmd_qos_);
      } catch (const mqtt::exception & e) {
        RCLCPP_WARN(this->get_logger(), "MQTT subscribe failed: %s", e.what());
      }
    });

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

    // Subscribe to command topics
    try {
      mqtt_client_->subscribe("rover/cmd/goal", cmd_qos_);
      mqtt_client_->subscribe("rover/cmd/arm", cmd_qos_);
      mqtt_client_->subscribe("rover/cmd/mode", cmd_qos_);
      mqtt_client_->subscribe("rover/cmd/estop", cmd_qos_);
      mqtt_client_->subscribe("rover/cmd/disarm", cmd_qos_);
      mqtt_client_->subscribe("rover/cmd/cancel_goal", cmd_qos_);
      RCLCPP_INFO(this->get_logger(), "Subscribed to rover/cmd/* topics (QoS %d)",
        cmd_qos_);
    } catch (const mqtt::exception & e) {
      RCLCPP_WARN(this->get_logger(), "MQTT command subscribe failed: %s", e.what());
    }

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

// --- Inbound command handling ---

void TelemetryPublisher::on_mqtt_message(mqtt::const_message_ptr mqtt_msg)
{
  const auto & topic = mqtt_msg->get_topic();
  const auto & payload = mqtt_msg->get_payload_str();

  // Deserialize Protobuf RoverCommand
  rover_monitor_proto::RoverCommand cmd;
  if (!cmd.ParseFromString(payload)) {
    RCLCPP_WARN(this->get_logger(), "Malformed RoverCommand on %s — dropped",
      topic.c_str());
    return;
  }

  const auto & cmd_id = cmd.cmd_id();
  if (cmd_id.empty()) {
    RCLCPP_WARN(this->get_logger(), "RoverCommand missing cmd_id — dropped");
    return;
  }

  // Dedup check
  if (is_duplicate(cmd_id)) {
    RCLCPP_DEBUG(this->get_logger(), "Duplicate cmd_id=%s — skipped", cmd_id.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Received command: cmd_id=%s type=%d from=%s",
    cmd_id.c_str(), static_cast<int>(cmd.cmd_type()), cmd.issued_by().c_str());

  // Queue both ACK_RECEIVED and dispatch for the ROS executor thread.
  // Publishing MQTT from within the Paho message callback causes a deadlock
  // in the Paho C++ async client (the network thread blocks on itself).
  {
    std::lock_guard<std::mutex> lock(cmd_queue_mutex_);
    ack_queue_.push({cmd_id, static_cast<int>(cmd.cmd_type()),
      rover_monitor_proto::ACK_RECEIVED, "Command received"});
    cmd_queue_.push({cmd_id, static_cast<int>(cmd.cmd_type()), payload});
  }
}

bool TelemetryPublisher::is_duplicate(const std::string & cmd_id)
{
  std::lock_guard<std::mutex> lock(dedup_mutex_);
  auto it = seen_cmds_.find(cmd_id);
  if (it != seen_cmds_.end()) {
    return true;
  }
  seen_cmds_[cmd_id] = std::chrono::steady_clock::now();
  return false;
}

void TelemetryPublisher::dispatch_command(const std::string & cmd_id, int cmd_type,
  const std::string & payload)
{
  rover_monitor_proto::RoverCommand cmd;
  cmd.ParseFromString(payload);

  switch (cmd_type) {
    case rover_monitor_proto::CMD_NAV_GOAL:
      handle_nav_goal(cmd_id, payload);
      break;
    case rover_monitor_proto::CMD_ARM:
      handle_arm(cmd_id);
      break;
    case rover_monitor_proto::CMD_DISARM:
      handle_disarm(cmd_id);
      break;
    case rover_monitor_proto::CMD_SET_MODE:
      handle_set_mode(cmd_id, payload);
      break;
    case rover_monitor_proto::CMD_ESTOP:
      handle_estop(cmd_id);
      break;
    case rover_monitor_proto::CMD_CANCEL_GOAL:
      handle_cancel_goal(cmd_id);
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "Unknown command type %d for cmd_id=%s",
        cmd_type, cmd_id.c_str());
      send_ack(cmd_id, cmd_type, rover_monitor_proto::ACK_REJECTED,
        "Unknown command type");
      break;
  }
}

void TelemetryPublisher::handle_arm(const std::string & cmd_id)
{
  RCLCPP_INFO(this->get_logger(), "ARM command: cmd_id=%s", cmd_id.c_str());
  publish_vehicle_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
    1.0f);  // param1=1 -> arm
  send_ack(cmd_id, rover_monitor_proto::CMD_ARM,
    rover_monitor_proto::ACK_ACCEPTED, "Arm command sent to PX4");
}

void TelemetryPublisher::handle_disarm(const std::string & cmd_id)
{
  RCLCPP_INFO(this->get_logger(), "DISARM command: cmd_id=%s", cmd_id.c_str());
  publish_vehicle_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
    0.0f);  // param1=0 -> disarm
  send_ack(cmd_id, rover_monitor_proto::CMD_DISARM,
    rover_monitor_proto::ACK_ACCEPTED, "Disarm command sent to PX4");
}

void TelemetryPublisher::handle_estop(const std::string & cmd_id)
{
  RCLCPP_WARN(this->get_logger(), "E-STOP command: cmd_id=%s", cmd_id.c_str());

  // 1. Publish zero twist to stop movement
  auto twist = geometry_msgs::msg::TwistStamped();
  twist.header.stamp = this->now();
  twist.header.frame_id = "base_link";
  // All velocities default to 0
  twist_pub_->publish(twist);

  // 2. Disarm
  publish_vehicle_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
    0.0f);

  send_ack(cmd_id, rover_monitor_proto::CMD_ESTOP,
    rover_monitor_proto::ACK_COMPLETED, "E-stop executed: zero twist + disarm");
}

void TelemetryPublisher::handle_nav_goal(const std::string & cmd_id,
  const std::string & payload)
{
  rover_monitor_proto::RoverCommand cmd;
  cmd.ParseFromString(payload);

  if (!cmd.has_nav_goal()) {
    send_ack(cmd_id, rover_monitor_proto::CMD_NAV_GOAL,
      rover_monitor_proto::ACK_REJECTED, "Missing nav_goal field");
    return;
  }

  const auto & goal = cmd.nav_goal();
  RCLCPP_INFO(this->get_logger(),
    "NAV_GOAL command: cmd_id=%s lat=%.6f lon=%.6f alt=%.1f yaw=%.1f",
    cmd_id.c_str(), goal.latitude(), goal.longitude(),
    goal.altitude(), goal.yaw_deg());

  // TODO: Send NavigateToPose action when Nav2 integration is available
  // For now, accept the command and log it
  send_ack(cmd_id, rover_monitor_proto::CMD_NAV_GOAL,
    rover_monitor_proto::ACK_ACCEPTED,
    "Nav goal accepted (Nav2 action dispatch pending)");
}

void TelemetryPublisher::handle_set_mode(const std::string & cmd_id,
  const std::string & payload)
{
  rover_monitor_proto::RoverCommand cmd;
  cmd.ParseFromString(payload);

  if (!cmd.has_set_mode()) {
    send_ack(cmd_id, rover_monitor_proto::CMD_SET_MODE,
      rover_monitor_proto::ACK_REJECTED, "Missing set_mode field");
    return;
  }

  const auto & mode = cmd.set_mode().mode_name();
  RCLCPP_INFO(this->get_logger(), "SET_MODE command: cmd_id=%s mode=%s",
    cmd_id.c_str(), mode.c_str());

  // Use PX4 DO_SET_MODE command
  publish_vehicle_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
    1.0f,   // param1: MAV_MODE custom
    6.0f);  // param2: PX4 custom main mode (offboard=6)

  send_ack(cmd_id, rover_monitor_proto::CMD_SET_MODE,
    rover_monitor_proto::ACK_ACCEPTED, "Mode change sent to PX4: " + mode);
}

void TelemetryPublisher::handle_cancel_goal(const std::string & cmd_id)
{
  RCLCPP_INFO(this->get_logger(), "CANCEL_GOAL command: cmd_id=%s", cmd_id.c_str());

  // TODO: Cancel active Nav2 goal when Nav2 integration is available
  send_ack(cmd_id, rover_monitor_proto::CMD_CANCEL_GOAL,
    rover_monitor_proto::ACK_ACCEPTED,
    "Cancel goal accepted (Nav2 cancel dispatch pending)");
}

void TelemetryPublisher::publish_vehicle_command(uint32_t command, float param1,
  float param2)
{
  auto msg = px4_msgs::msg::VehicleCommand();
  msg.timestamp = this->now().nanoseconds() / 1000;  // PX4 uses microseconds
  msg.command = command;
  msg.param1 = param1;
  msg.param2 = param2;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  vehicle_cmd_pub_->publish(msg);
}

void TelemetryPublisher::send_ack(const std::string & cmd_id, int cmd_type,
  int status, const std::string & message)
{
  if (!mqtt_connected_ || !mqtt_client_->is_connected()) {
    return;
  }

  rover_monitor_proto::CommandAck ack;
  ack.set_cmd_id(cmd_id);
  ack.set_cmd_type(static_cast<rover_monitor_proto::CommandType>(cmd_type));
  ack.set_status(static_cast<rover_monitor_proto::AckStatus>(status));
  ack.set_message(message);
  ack.set_timestamp(this->now().nanoseconds() / 1000000);

  std::string payload;
  ack.SerializeToString(&payload);

  try {
    mqtt_client_->publish("rover/cmd/ack", payload.data(), payload.size(),
      1, false);  // QoS 1 for ACKs
  } catch (const mqtt::exception & e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "MQTT ACK publish failed: %s", e.what());
  }
}

// --- Outbound telemetry (unchanged) ---

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
