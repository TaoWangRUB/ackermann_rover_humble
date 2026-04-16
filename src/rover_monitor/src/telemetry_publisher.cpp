#include "rover_monitor/telemetry_publisher.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include "rover_health.pb.h"

#include <cmath>

namespace rover_monitor
{

namespace
{

template<typename MessageT>
std::string versioned_px4_topic(const std::string & base_topic)
{
  if constexpr (MessageT::MESSAGE_VERSION == 0) {
    return base_topic;
  }
  return base_topic + "_v" + std::to_string(MessageT::MESSAGE_VERSION);
}

double duration_to_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
    static_cast<double>(duration.nanosec) / 1e9;
}

geometry_msgs::msg::Quaternion quaternion_from_yaw_deg(double yaw_deg)
{
  geometry_msgs::msg::Quaternion quaternion;
  const double yaw_rad = yaw_deg * M_PI / 180.0;
  quaternion.z = static_cast<double>(std::sin(yaw_rad * 0.5));
  quaternion.w = static_cast<double>(std::cos(yaw_rad * 0.5));
  return quaternion;
}

}  // namespace

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
  this->declare_parameter("publisher.heartbeat_interval_s", 5.0);

  broker_host_ = this->get_parameter("publisher.broker_host").as_string();
  broker_port_ = this->get_parameter("publisher.broker_port").as_int();
  reconnect_delay_s_ = this->get_parameter("publisher.reconnect_delay_s").as_int();
  telemetry_qos_ = this->get_parameter("publisher.telemetry_qos").as_int();
  alert_qos_ = this->get_parameter("publisher.alert_qos").as_int();
  cmd_qos_ = this->get_parameter("publisher.cmd_qos").as_int();
  client_id_ = this->get_parameter("publisher.client_id").as_string();
  dedup_eviction_s_ = this->get_parameter("publisher.dedup_eviction_s").as_int();
  heartbeat_interval_s_ = this->get_parameter("publisher.heartbeat_interval_s").as_double();

  // ROS subscriber for health telemetry
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group_;

  health_sub_ = this->create_subscription<rover_monitor::msg::RoverHealth>(
    "/monitor/health", 1,
    std::bind(&TelemetryPublisher::on_health, this, std::placeholders::_1), sub_opts);

  // PX4 vehicle command publisher
  vehicle_cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
    versioned_px4_topic<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command"), 10);

  // E-stop twist publisher (zero velocity)
  twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/cmd_vel", 10);

  nav2_status_pub_ = this->create_publisher<rover_monitor::msg::Nav2Status>(
    "/monitor/nav2", 1);
  nav2_client_ = rclcpp_action::create_client<NavigateToPose>(
    this, "/navigate_to_pose", cb_group_);

  nav2_status_.goal_status_label = "IDLE";
  nav2_status_.feedback_status = "idle";

  nav2_status_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&TelemetryPublisher::on_nav2_status_timer, this), cb_group_);

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

  // Periodic heartbeat timer — publishes cached health at a low rate for dashboards
  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(heartbeat_interval_s_)),
    std::bind(&TelemetryPublisher::on_heartbeat_timer, this), cb_group_);

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
        mqtt_client_->subscribe("rover/cmd/drive", 0);  // QoS 0 for high-freq drive
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
      mqtt_client_->subscribe("rover/cmd/drive", 0);  // QoS 0 for high-freq drive
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
    case rover_monitor_proto::CMD_DRIVE:
      handle_drive(cmd_id, payload);
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
    "NAV_GOAL command: cmd_id=%s x=%.3f y=%.3f z=%.3f yaw=%.1f",
    cmd_id.c_str(), goal.latitude(), goal.longitude(),
    goal.altitude(), goal.yaw_deg());

  if (!nav2_client_->wait_for_action_server(std::chrono::seconds(1))) {
    {
      std::lock_guard<std::mutex> lock(nav2_mutex_);
      nav2_status_.available = false;
      nav2_status_.goal_status_label = "UNAVAILABLE";
      nav2_status_.feedback_status = "server_unavailable";
      nav2_status_.error_code = 1;
      nav2_status_.error_msg = "NavigateToPose action server unavailable";
    }
    publish_nav2_status();
    send_ack(cmd_id, rover_monitor_proto::CMD_NAV_GOAL,
      rover_monitor_proto::ACK_REJECTED,
      "Nav2 action server unavailable");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(nav2_mutex_);
    if (active_nav2_goal_handle_) {
      send_ack(cmd_id, rover_monitor_proto::CMD_NAV_GOAL,
        rover_monitor_proto::ACK_REJECTED,
        "Nav2 goal already active — cancel it first");
      return;
    }
  }

  NavigateToPose::Goal nav_goal;
  nav_goal.pose.header.stamp = this->now();
  nav_goal.pose.header.frame_id = "map";
  nav_goal.pose.pose.position.x = goal.latitude();
  nav_goal.pose.pose.position.y = goal.longitude();
  nav_goal.pose.pose.position.z = goal.altitude();
  nav_goal.pose.pose.orientation = quaternion_from_yaw_deg(goal.yaw_deg());

  {
    std::lock_guard<std::mutex> lock(nav2_mutex_);
    nav2_status_.available = true;
    nav2_status_.navigating = false;
    nav2_status_.goal_status_label = "PENDING";
    nav2_status_.feedback_status = "pending";
    nav2_status_.distance_remaining_m = 0.0f;
    nav2_status_.eta_seconds = 0.0f;
    nav2_status_.navigation_time_s = 0.0f;
    nav2_status_.number_of_recoveries = 0;
    nav2_status_.number_of_poses_remaining = 0;
    nav2_status_.error_code = 0;
    nav2_status_.error_msg.clear();
  }
  publish_nav2_status();

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this, cmd_id](NavigateToPoseGoalHandle::SharedPtr goal_handle) {
      if (!goal_handle) {
        {
          std::lock_guard<std::mutex> lock(nav2_mutex_);
          nav2_status_.navigating = false;
          nav2_status_.goal_status_label = "REJECTED";
          nav2_status_.feedback_status = "rejected";
          nav2_status_.error_code = 2;
          nav2_status_.error_msg = "Nav2 goal rejected";
        }
        publish_nav2_status();
        send_ack(cmd_id, rover_monitor_proto::CMD_NAV_GOAL,
          rover_monitor_proto::ACK_REJECTED, "Nav2 goal rejected");
        return;
      }

      {
        std::lock_guard<std::mutex> lock(nav2_mutex_);
        active_nav2_goal_handle_ = goal_handle;
        active_nav_goal_cmd_id_ = cmd_id;
        nav2_status_.navigating = true;
        nav2_status_.goal_status_label = "ACCEPTED";
        nav2_status_.feedback_status = "waiting_for_feedback";
        nav2_status_.error_code = 0;
        nav2_status_.error_msg.clear();
      }
      publish_nav2_status();
      send_ack(cmd_id, rover_monitor_proto::CMD_NAV_GOAL,
        rover_monitor_proto::ACK_ACCEPTED, "Nav2 goal accepted");
    };
  send_goal_options.feedback_callback =
    [this](NavigateToPoseGoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
      {
        std::lock_guard<std::mutex> lock(nav2_mutex_);
        nav2_status_.available = true;
        nav2_status_.navigating = true;
        nav2_status_.goal_status_label = "EXECUTING";
        nav2_status_.feedback_status = "active";
        nav2_status_.distance_remaining_m = feedback->distance_remaining;
        nav2_status_.eta_seconds = static_cast<float>(
          duration_to_seconds(feedback->estimated_time_remaining));
        nav2_status_.navigation_time_s = static_cast<float>(
          duration_to_seconds(feedback->navigation_time));
        nav2_status_.number_of_recoveries = feedback->number_of_recoveries;
        nav2_status_.number_of_poses_remaining = 0;
        nav2_status_.error_code = 0;
        nav2_status_.error_msg.clear();
      }
      publish_nav2_status();
    };
  send_goal_options.result_callback =
    [this, cmd_id](const NavigateToPoseGoalHandle::WrappedResult & result) {
      int ack_status = rover_monitor_proto::ACK_FAILED;
      std::string ack_message = "Nav2 goal failed";
      std::string goal_status_label = "UNKNOWN";
      std::string feedback_status = "unknown";
      int error_code = 3;

      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          ack_status = rover_monitor_proto::ACK_COMPLETED;
          ack_message = "Nav2 goal completed";
          goal_status_label = "SUCCEEDED";
          feedback_status = "succeeded";
          error_code = 0;
          break;
        case rclcpp_action::ResultCode::ABORTED:
          ack_status = rover_monitor_proto::ACK_FAILED;
          ack_message = "Nav2 goal aborted";
          goal_status_label = "ABORTED";
          feedback_status = "aborted";
          error_code = 4;
          break;
        case rclcpp_action::ResultCode::CANCELED:
          ack_status = rover_monitor_proto::ACK_COMPLETED;
          ack_message = "Nav2 goal canceled";
          goal_status_label = "CANCELED";
          feedback_status = "canceled";
          error_code = 0;
          break;
        default:
          break;
      }

      {
        std::lock_guard<std::mutex> lock(nav2_mutex_);
        active_nav2_goal_handle_.reset();
        active_nav_goal_cmd_id_.clear();
        nav2_status_.available = nav2_client_->action_server_is_ready();
        nav2_status_.navigating = false;
        nav2_status_.goal_status_label = goal_status_label;
        nav2_status_.feedback_status = feedback_status;
        nav2_status_.distance_remaining_m = 0.0f;
        nav2_status_.eta_seconds = 0.0f;
        nav2_status_.navigation_time_s = 0.0f;
        nav2_status_.number_of_poses_remaining = 0;
        nav2_status_.error_code = error_code;
        nav2_status_.error_msg = error_code == 0 ? std::string() : ack_message;
      }
      publish_nav2_status();
      send_ack(cmd_id, rover_monitor_proto::CMD_NAV_GOAL,
        ack_status, ack_message);
    };

  nav2_client_->async_send_goal(nav_goal, send_goal_options);
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

  NavigateToPoseGoalHandle::SharedPtr goal_handle;
  {
    std::lock_guard<std::mutex> lock(nav2_mutex_);
    goal_handle = active_nav2_goal_handle_;
  }

  if (!goal_handle) {
    send_ack(cmd_id, rover_monitor_proto::CMD_CANCEL_GOAL,
      rover_monitor_proto::ACK_REJECTED,
      "No active Nav2 goal to cancel");
    return;
  }

  send_ack(cmd_id, rover_monitor_proto::CMD_CANCEL_GOAL,
    rover_monitor_proto::ACK_ACCEPTED,
    "Nav2 goal cancel requested");

  nav2_client_->async_cancel_goal(goal_handle,
    [this, cmd_id](
      std::shared_future<
        typename NavigateToPoseGoalHandle::CancelResponse::SharedPtr> future) {
      const auto cancel_response = future.get();
      if (!cancel_response || cancel_response->goals_canceling.empty()) {
        send_ack(cmd_id, rover_monitor_proto::CMD_CANCEL_GOAL,
          rover_monitor_proto::ACK_FAILED,
          "Nav2 goal cancel rejected");
        return;
      }

      {
        std::lock_guard<std::mutex> lock(nav2_mutex_);
        nav2_status_.goal_status_label = "CANCELING";
        nav2_status_.feedback_status = "canceling";
      }
      publish_nav2_status();
      send_ack(cmd_id, rover_monitor_proto::CMD_CANCEL_GOAL,
        rover_monitor_proto::ACK_COMPLETED,
        "Nav2 goal cancel accepted");
    });
}

void TelemetryPublisher::handle_drive(const std::string & cmd_id,
  const std::string & payload)
{
  rover_monitor_proto::RoverCommand cmd;
  cmd.ParseFromString(payload);

  if (!cmd.has_drive()) {
    send_ack(cmd_id, rover_monitor_proto::CMD_DRIVE,
      rover_monitor_proto::ACK_REJECTED, "Missing drive field");
    return;
  }

  float speed = cmd.drive().speed_ms();
  float steering = cmd.drive().steering();

  auto twist = geometry_msgs::msg::TwistStamped();
  twist.header.stamp = this->now();
  twist.header.frame_id = "base_link";
  twist.twist.linear.x = speed;
  twist.twist.angular.z = steering;
  twist_pub_->publish(twist);

  // No ACK for drive — high-frequency, fire-and-forget
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

// --- Outbound telemetry ---

std::string TelemetryPublisher::serialise_health(
  const rover_monitor::msg::RoverHealth & msg) const
{
  rover_monitor_proto::RoverHealth pb;
  pb.set_seq(msg.seq);
  pb.set_timestamp(msg.timestamp);
  pb.set_slam_latency_ms(msg.slam_latency_ms);
  pb.set_overall_health(msg.overall_health);
  for (const auto & alert : msg.active_alerts) {
    pb.add_active_alerts(alert);
  }

  // Cameras (repeated)
  for (const auto & cam_msg : msg.cameras) {
    auto * cam = pb.add_cameras();
    cam->set_camera_id(cam_msg.camera_id);
    cam->set_connected(cam_msg.connected);
    cam->set_frame_delta_ms(cam_msg.frame_delta_ms);
    cam->set_stream_fps(cam_msg.stream_fps);
    cam->set_stream_available(cam_msg.stream_available);
    cam->set_depth_fps(cam_msg.depth_fps);
    cam->set_depth_quality_sampled(cam_msg.depth_quality_sampled);
    cam->set_imu_active(cam_msg.imu_active);
    cam->set_odom_active(cam_msg.odom_active);
    cam->set_error_code(cam_msg.error_code);
    cam->set_error_msg(cam_msg.error_msg);
    cam->set_timestamp(cam_msg.timestamp);
  }

  // PX4
  auto * px4 = pb.mutable_px4();
  px4->set_connected(msg.px4.connected);
  px4->set_armed(msg.px4.armed);
  px4->set_armable(msg.px4.armable);
  px4->set_nav_state(msg.px4.nav_state);
  px4->set_nav_state_label(msg.px4.nav_state_label);
  px4->set_battery_voltage_v(msg.px4.battery_voltage_v);
  px4->set_battery_current_a(msg.px4.battery_current_a);
  px4->set_battery_remaining_pct(msg.px4.battery_remaining_pct);
  px4->set_heartbeat_age_ms(msg.px4.heartbeat_age_ms);
  px4->set_error_code(msg.px4.error_code);
  px4->set_error_msg(msg.px4.error_msg);
  px4->set_timestamp(msg.px4.timestamp);

  // Jetson
  auto * jetson = pb.mutable_jetson();
  for (const auto & cpu : msg.jetson.cpu_usage_pct) {
    jetson->add_cpu_usage_pct(cpu);
  }
  jetson->set_gpu_usage_pct(msg.jetson.gpu_usage_pct);
  jetson->set_ram_used_mb(msg.jetson.ram_used_mb);
  jetson->set_ram_total_mb(msg.jetson.ram_total_mb);
  jetson->set_swap_used_mb(msg.jetson.swap_used_mb);
  jetson->set_disk_free_gb(msg.jetson.disk_free_gb);
  jetson->set_temp_cpu_c(msg.jetson.temp_cpu_c);
  jetson->set_temp_gpu_c(msg.jetson.temp_gpu_c);
  jetson->set_temp_board_c(msg.jetson.temp_board_c);
  jetson->set_is_thermal_throttled(msg.jetson.is_thermal_throttled);
  jetson->set_is_power_throttled(msg.jetson.is_power_throttled);
  jetson->set_power_mode(msg.jetson.power_mode);
  jetson->set_wifi_signal_dbm(msg.jetson.wifi_signal_dbm);
  jetson->set_uptime_s(msg.jetson.uptime_s);
  jetson->set_error_code(msg.jetson.error_code);
  jetson->set_error_msg(msg.jetson.error_msg);
  jetson->set_timestamp(msg.jetson.timestamp);

  auto * nav2 = pb.mutable_nav2();
  nav2->set_available(msg.nav2.available);
  nav2->set_navigating(msg.nav2.navigating);
  nav2->set_localization_active(msg.nav2.localization_active);
  nav2->set_goal_status_label(msg.nav2.goal_status_label);
  nav2->set_feedback_status(msg.nav2.feedback_status);
  nav2->set_distance_remaining_m(msg.nav2.distance_remaining_m);
  nav2->set_eta_seconds(msg.nav2.eta_seconds);
  nav2->set_navigation_time_s(msg.nav2.navigation_time_s);
  nav2->set_number_of_recoveries(msg.nav2.number_of_recoveries);
  nav2->set_number_of_poses_remaining(msg.nav2.number_of_poses_remaining);
  nav2->set_error_code(msg.nav2.error_code);
  nav2->set_error_msg(msg.nav2.error_msg);
  nav2->set_timestamp(msg.nav2.timestamp);

  std::string payload;
  pb.SerializeToString(&payload);
  return payload;
}

void TelemetryPublisher::on_health(rover_monitor::msg::RoverHealth::ConstSharedPtr msg)
{
  // Always cache the latest health for the heartbeat timer
  latest_health_ = msg;
  update_nav2_localization_flag();

  // Alert edge detection: publish immediately when alerts change
  std::vector<std::string> current_alerts(msg->active_alerts.begin(),
    msg->active_alerts.end());

  if (current_alerts != prev_alerts_) {
    prev_alerts_ = current_alerts;

    mqtt_connected_ = mqtt_client_ && mqtt_client_->is_connected();
    if (!mqtt_connected_) { return; }

    std::string payload = serialise_health(*msg);
    try {
      // Alert state changed — publish with QoS 1 for reliable delivery
      mqtt_client_->publish("rover/alerts", payload.data(), payload.size(),
        alert_qos_, false);

      RCLCPP_INFO(this->get_logger(), "Alert state changed: %zu active alert(s)",
        current_alerts.size());
    } catch (const mqtt::exception & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "MQTT alert publish failed: %s", e.what());
      mqtt_connected_ = false;
    }
  }
}

void TelemetryPublisher::on_heartbeat_timer()
{
  if (!latest_health_) { return; }

  mqtt_connected_ = mqtt_client_ && mqtt_client_->is_connected();
  if (!mqtt_connected_) { return; }

  std::string payload = serialise_health(*latest_health_);
  try {
    mqtt_client_->publish("rover/health/overall", payload.data(), payload.size(),
      telemetry_qos_, false);
  } catch (const mqtt::exception & e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "MQTT heartbeat publish failed: %s", e.what());
    mqtt_connected_ = false;
  }
}

void TelemetryPublisher::update_nav2_localization_flag()
{
  std::lock_guard<std::mutex> lock(nav2_mutex_);
  nav2_status_.localization_active = latest_health_ && latest_health_->slam_latency_ms >= 0.0f;
}

void TelemetryPublisher::publish_nav2_status()
{
  rover_monitor::msg::Nav2Status nav2_status_msg;
  {
    std::lock_guard<std::mutex> lock(nav2_mutex_);
    nav2_status_.timestamp = this->now().nanoseconds() / 1000000;
    nav2_status_msg = nav2_status_;
  }
  nav2_status_pub_->publish(nav2_status_msg);
}

void TelemetryPublisher::on_nav2_status_timer()
{
  {
    std::lock_guard<std::mutex> lock(nav2_mutex_);
    nav2_status_.available = nav2_client_->action_server_is_ready();
    if (!nav2_status_.available && !nav2_status_.navigating) {
      nav2_status_.goal_status_label = "UNAVAILABLE";
      nav2_status_.feedback_status = "server_unavailable";
    } else if (nav2_status_.available && !nav2_status_.navigating &&
      (nav2_status_.goal_status_label.empty() || nav2_status_.goal_status_label == "UNAVAILABLE"))
    {
      nav2_status_.goal_status_label = "IDLE";
      nav2_status_.feedback_status = "idle";
      nav2_status_.error_code = 0;
      nav2_status_.error_msg.clear();
    }
    nav2_status_.localization_active = latest_health_ && latest_health_->slam_latency_ms >= 0.0f;
  }
  publish_nav2_status();
}

}  // namespace rover_monitor

RCLCPP_COMPONENTS_REGISTER_NODE(rover_monitor::TelemetryPublisher)
