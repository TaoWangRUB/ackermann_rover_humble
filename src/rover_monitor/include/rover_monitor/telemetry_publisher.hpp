#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rover_monitor/msg/rover_health.hpp>
#include <rover_monitor/msg/nav2_status.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/register_ext_component_reply.hpp>
#include <std_msgs/msg/string.hpp>
#include <array>

#include <mqtt/async_client.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <sys/types.h>  // pid_t

namespace rover_monitor
{

class TelemetryPublisher : public rclcpp::Node
{
public:
  explicit TelemetryPublisher(const rclcpp::NodeOptions & options);
  ~TelemetryPublisher() override;

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateToPoseGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  // --- Outbound telemetry ---
  void on_health(rover_monitor::msg::RoverHealth::ConstSharedPtr msg);
  void connect_mqtt();
  void on_nav2_status_timer();
  void publish_nav2_status();
  void update_nav2_localization_flag();

  // Shadow path for CLI/RViz-initiated Nav2 goals (we're not the action client).
  using NavigateToPoseFeedbackMsg = nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage;
  void on_nav2_action_feedback(NavigateToPoseFeedbackMsg::ConstSharedPtr msg);
  void on_nav2_action_status(action_msgs::msg::GoalStatusArray::ConstSharedPtr msg);

  // --- Inbound command handling ---
  void on_mqtt_message(mqtt::const_message_ptr mqtt_msg);
  void dispatch_command(const std::string & cmd_id, int cmd_type,
    const std::string & payload);
  void send_ack(const std::string & cmd_id, int cmd_type, int status,
    const std::string & message);

  // Command handlers
  void handle_nav_goal(const std::string & cmd_id, const std::string & payload);
  void handle_arm(const std::string & cmd_id);
  void handle_disarm(const std::string & cmd_id);
  void handle_set_mode(const std::string & cmd_id, const std::string & payload);
  void handle_estop(const std::string & cmd_id);
  void handle_cancel_goal(const std::string & cmd_id);
  void handle_drive(const std::string & cmd_id, const std::string & payload);
  void handle_record(const std::string & cmd_id, const std::string & payload);
  void handle_register_mode(const std::string & cmd_id, const std::string & payload);
  void handle_unregister_mode(const std::string & cmd_id, const std::string & payload);

  // Dedup
  bool is_duplicate(const std::string & cmd_id);

  // Publish a VehicleCommand to PX4
  void publish_vehicle_command(uint32_t command, float param1 = 0.0f,
    float param2 = 0.0f);

  // Snoop PX4 mode registration replies to learn name → nav_state ID.
  // Each rover mode (rover_speed_steering, rover_speed_attitude, rover_manual)
  // registers itself with PX4 at startup; the FMU assigns a runtime nav_state
  // and replies on /fmu/out/register_ext_component_reply. We cache that map
  // so handle_set_mode can switch into a registered mode by name.
  void on_register_ext_component_reply(
    px4_msgs::msg::RegisterExtComponentReply::ConstSharedPtr msg);

  // Reliable secondary path: rover_*_mode nodes also announce (name, mode_id)
  // on /px4_modes/announce with RELIABLE + TRANSIENT_LOCAL after their own
  // registration succeeds. This is the source of truth for the cache because
  // the PX4 reply path (above) is fire-once + best_effort + shallow uORB
  // queue, so messages can be missed if multiple modes register together.
  void on_mode_announce(std_msgs::msg::String::ConstSharedPtr msg);

  // ROS subscriber
  rclcpp::Subscription<rover_monitor::msg::RoverHealth>::SharedPtr health_sub_;

  // Callback group
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  // MQTT client
  std::shared_ptr<mqtt::async_client> mqtt_client_;

  // Reconnect timer
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  // Dedup eviction timer
  rclcpp::TimerBase::SharedPtr dedup_timer_;

  // PX4 command publisher
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;

  // PX4 ext-component-registration snoop: name → nav_state ID
  rclcpp::Subscription<px4_msgs::msg::RegisterExtComponentReply>::SharedPtr
    register_reply_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_announce_sub_;
  std::mutex mode_id_mutex_;
  std::unordered_map<std::string, int8_t> mode_id_by_name_;

  // Mode process management: mode_name → PID of spawned mode node
  std::mutex mode_proc_mutex_;
  std::unordered_map<std::string, pid_t> mode_procs_;
  // Map mode display name → executable name
  static const std::unordered_map<std::string, std::string> kModeExecutableMap;

  // E-stop twist publisher
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

  // Record-control publisher — bridges MQTT rover/cmd/record to record_bag.sh
  // (which subscribes to /record/cmd as a String: "start" | "stop" | "toggle").
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr record_cmd_pub_;

  // Nav2 goal/status bridge
  rclcpp::Publisher<rover_monitor::msg::Nav2Status>::SharedPtr nav2_status_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
  rclcpp::TimerBase::SharedPtr nav2_status_timer_;
  std::mutex nav2_mutex_;
  rover_monitor::msg::Nav2Status nav2_status_;
  NavigateToPoseGoalHandle::SharedPtr active_nav2_goal_handle_;
  std::string active_nav_goal_cmd_id_;

  // Passive shadow subscribers — track goals sent by others (RViz / CLI).
  rclcpp::Subscription<NavigateToPoseFeedbackMsg>::SharedPtr nav2_feedback_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav2_status_sub_;
  // Last external goal UUID we're tracking (non-zero when a non-RCC goal is active).
  std::array<uint8_t, 16> external_goal_uuid_{};
  bool external_goal_active_{false};

  // Command deduplication set with timestamps
  std::mutex dedup_mutex_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> seen_cmds_;

  // Inbound command queue (Paho thread → ROS executor thread)
  // Both command dispatch and ACK sending are queued to avoid deadlock
  // when publishing MQTT/ROS from within the Paho message callback.
  struct PendingCmd { std::string cmd_id; int cmd_type; std::string payload; };
  struct PendingAck { std::string cmd_id; int cmd_type; int status; std::string message; };
  std::mutex cmd_queue_mutex_;
  std::queue<PendingCmd> cmd_queue_;
  std::queue<PendingAck> ack_queue_;
  rclcpp::TimerBase::SharedPtr cmd_drain_timer_;

  // Heartbeat timer for periodic metrics publishing
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  void on_heartbeat_timer();

  // Serialise a RoverHealth ROS message into a Protobuf string
  std::string serialise_health(const rover_monitor::msg::RoverHealth & msg) const;

  // Cached latest health message for heartbeat publishing
  rover_monitor::msg::RoverHealth::ConstSharedPtr latest_health_;

  // Alert edge detection — previous set of active alerts
  std::vector<std::string> prev_alerts_;

  // Config
  std::string broker_host_{"192.168.1.100"};
  int broker_port_{1883};
  int reconnect_delay_s_{5};
  int telemetry_qos_{0};
  int alert_qos_{1};
  int cmd_qos_{2};
  std::string client_id_{"xavier_rover_01"};
  int dedup_eviction_s_{30};
  int keep_alive_s_{15};
  int connect_timeout_s_{5};
  double heartbeat_interval_s_{5.0};

  bool mqtt_connected_{false};
};

}  // namespace rover_monitor
