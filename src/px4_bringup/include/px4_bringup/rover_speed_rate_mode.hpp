// Copyright 2026, Tao Wang. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rover/speed_rate.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Rover Speed+Rate mode — maps cmd_vel to body speed and yaw rate.
 *
 * Subscribes to a configurable cmd_vel topic (default: "/cmd_vel").
 * Accepts both geometry_msgs/Twist and geometry_msgs/TwistStamped.
 *
 * Extracts:
 *   - linear.x  → speed_body_x [m/s]
 *   - angular.z → yaw_rate [rad/s] (sign-flipped: ROS CCW+ ENU → PX4 CW+ NED)
 *
 * Unlike Speed+Steering (which converts angular.z to a normalized geometric
 * steering command) and Speed+Attitude (which integrates angular.z into a
 * heading-hold setpoint), Speed+Rate forwards the rate directly to PX4's
 * inner-loop yaw-rate controller. Suitable for teleop where the operator
 * commands a turn-rate rather than a steering angle or a heading.
 *
 * Safety features:
 *   - Before any cmd_vel is received: sends zero setpoint (vehicle stays still).
 *   - After first cmd_vel received: if no message arrives within cmd_vel_timeout
 *     seconds, the setpoint is zeroed (vehicle stops).
 *   - On deactivation (mode switch / node shutdown), a zero setpoint is sent.
 *   - Optional max_yaw_rate clamp to bound the rate command.
 *
 * Parameters:
 *   - cmd_vel_topic      (string, default: "/cmd_vel")  — topic name
 *   - use_stamped        (bool,   default: false)       — true = TwistStamped, false = Twist
 *   - max_yaw_rate       (double, default: 1.5)         — clamp |yaw_rate| [rad/s]
 *   - cmd_vel_timeout    (double, default: 2.0)         — watchdog timeout [s]
 */
class RoverSpeedRateMode : public px4_ros2::ModeBase
{
public:
  explicit RoverSpeedRateMode(rclcpp::Node & node)
  : ModeBase(node, Settings{"Rover Speed Rate"}.preventArming(false)),
    node_(node)
  {
    // See rover_manual_mode.hpp for rationale — disables the 4 s watchdog
    // so a queue-overflow burst (PX4#27271) does not crash the mode.
    disableWatchdogTimer();

    speed_rate_setpoint_ =
      std::make_shared<px4_ros2::RoverSpeedRateSetpointType>(*this);

    // ── Parameters ────────────────────────────────────────────────
    if (!node_.has_parameter("skip_message_compatibility_check")) {
      node_.declare_parameter("skip_message_compatibility_check", false);
    }
    if (node_.get_parameter("skip_message_compatibility_check").as_bool()) {
      setSkipMessageCompatibilityCheck();
    }

    if (!node_.has_parameter("cmd_vel_topic")) {
      node_.declare_parameter("cmd_vel_topic", std::string("/cmd_vel"));
    }
    const auto topic =
      node_.get_parameter("cmd_vel_topic").as_string();

    if (!node_.has_parameter("max_yaw_rate")) {
      node_.declare_parameter("max_yaw_rate", 1.5);
    }
    max_yaw_rate_ =
      static_cast<float>(node_.get_parameter("max_yaw_rate").as_double());

    if (!node_.has_parameter("cmd_vel_timeout")) {
      node_.declare_parameter("cmd_vel_timeout", 2.0);
    }
    cmd_vel_timeout_s_ =
      node_.get_parameter("cmd_vel_timeout").as_double();

    if (!node_.has_parameter("use_stamped")) {
      node_.declare_parameter("use_stamped", false);
    }
    const bool use_stamped =
      node_.get_parameter("use_stamped").as_bool();

    // ── Subscriber (one type per topic — DDS constraint) ─────────
    if (use_stamped) {
      cmd_vel_stamped_sub_ = node_.create_subscription<geometry_msgs::msg::TwistStamped>(
        topic, 10,
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
          handleTwist(msg->twist);
        });
    } else {
      cmd_vel_sub_ = node_.create_subscription<geometry_msgs::msg::Twist>(
        topic, 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          handleTwist(*msg);
        });
    }

    RCLCPP_INFO(
      node_.get_logger(),
      "RoverSpeedRateMode: subscribing to '%s' (%s), "
      "timeout=%.2f s, max_yaw_rate=%.2f rad/s",
      topic.c_str(), use_stamped ? "TwistStamped" : "Twist",
      cmd_vel_timeout_s_, static_cast<double>(max_yaw_rate_));
  }

  void onActivate() override
  {
    RCLCPP_INFO(node_.get_logger(), "RoverSpeedRateMode activated");
    {
      std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
      last_cmd_vel_ = geometry_msgs::msg::Twist{};
      cmd_vel_received_ = false;
      cmd_vel_timed_out_ = false;
    }
  }

  void onDeactivate() override
  {
    RCLCPP_INFO(node_.get_logger(), "RoverSpeedRateMode deactivated — sending zero setpoint");
    speed_rate_setpoint_->update(0.0f, 0.0f);
  }

  void updateSetpoint(float /*dt_s*/) override
  {
    geometry_msgs::msg::Twist cmd;
    bool received = false;
    bool timed_out = false;
    {
      std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
      received = cmd_vel_received_;
      if (received) {
        const double age = (node_.now() - last_cmd_vel_time_).seconds();
        timed_out = (age > cmd_vel_timeout_s_);
      }
      cmd = last_cmd_vel_;
    }

    // Before first cmd_vel: send zeros, wait quietly
    if (!received) {
      speed_rate_setpoint_->update(0.0f, 0.0f);
      return;
    }

    // After first cmd_vel: apply timeout watchdog
    if (timed_out) {
      if (!cmd_vel_timed_out_) {
        RCLCPP_WARN(
          node_.get_logger(),
          "cmd_vel timeout (>%.2f s) — zeroing setpoint (vehicle will stop)",
          cmd_vel_timeout_s_);
        cmd_vel_timed_out_ = true;
      }
      speed_rate_setpoint_->update(0.0f, 0.0f);
      return;
    }

    if (cmd_vel_timed_out_) {
      RCLCPP_INFO(node_.get_logger(), "cmd_vel resumed — restoring setpoint");
      cmd_vel_timed_out_ = false;
    }

    // Forward speed in body x [m/s]
    const float speed_body_x = static_cast<float>(cmd.linear.x);

    // Yaw rate: ROS angular.z (CCW+ about Z-up, ENU) → PX4 (CW+ about Z-down, NED).
    // Negate before sending; clamp by max_yaw_rate as a safety bound.
    float yaw_rate_ned = -static_cast<float>(cmd.angular.z);
    if (max_yaw_rate_ > 0.0f) {
      yaw_rate_ned = std::clamp(yaw_rate_ned, -max_yaw_rate_, max_yaw_rate_);
    }

    speed_rate_setpoint_->update(speed_body_x, yaw_rate_ned);
  }

private:
  /// Common handler for both Twist and TwistStamped messages
  void handleTwist(const geometry_msgs::msg::Twist & twist)
  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    last_cmd_vel_ = twist;
    last_cmd_vel_time_ = node_.now();
    cmd_vel_received_ = true;
  }

  rclcpp::Node & node_;
  std::shared_ptr<px4_ros2::RoverSpeedRateSetpointType> speed_rate_setpoint_;

  // Subscriptions — only one is active depending on use_stamped param
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_sub_;

  geometry_msgs::msg::Twist last_cmd_vel_;
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
  std::mutex cmd_vel_mutex_;

  float max_yaw_rate_{1.5f};
  double cmd_vel_timeout_s_{2.0};
  bool cmd_vel_received_{false};
  bool cmd_vel_timed_out_{false};
};
