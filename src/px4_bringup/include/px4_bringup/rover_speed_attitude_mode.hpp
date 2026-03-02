// Copyright 2026, Tao Wang. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rover/speed_attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Rover Speed+Attitude mode — maps cmd_vel to speed and yaw heading.
 *
 * Subscribes to a configurable cmd_vel topic (default: "/cmd_vel").
 * Accepts both geometry_msgs/Twist and geometry_msgs/TwistStamped.
 *
 * Extracts:
 *   - linear.x  → speed_body_x [m/s]
 *   - angular.z → yaw rate [rad/s], integrated into a heading setpoint in NED
 *
 * The current vehicle heading is read from PX4 telemetry on activation.
 * angular.z is then integrated each cycle to produce an absolute yaw target.
 * PX4's Ackermann attitude controller handles heading → steering conversion.
 *
 * Sign convention: ROS angular.z is CCW+ about Z-up (ENU).
 * PX4 yaw is CW+ about Z-down (NED). We negate the rate before integrating.
 *
 * Safety features:
 *   - Before any cmd_vel is received: sends zero speed, holds heading (vehicle stays still).
 *   - After first cmd_vel received: if no message arrives within cmd_vel_timeout
 *     seconds, speed is zeroed and heading is held.
 *   - On deactivation (mode switch / node shutdown), a zero setpoint is sent.
 *
 * Parameters:
 *   - cmd_vel_topic      (string, default: "/cmd_vel")  — topic name
 *   - use_stamped        (bool,   default: false)       — true = TwistStamped, false = Twist
 *   - cmd_vel_timeout    (double, default: 2.0)         — watchdog timeout [s]
 */
class RoverSpeedAttitudeMode : public px4_ros2::ModeBase
{
public:
  explicit RoverSpeedAttitudeMode(rclcpp::Node & node)
  : ModeBase(node, Settings{"Rover Speed Attitude"}),
    node_(node)
  {
    speed_attitude_setpoint_ =
      std::make_shared<px4_ros2::RoverSpeedAttitudeSetpointType>(*this);

    // Telemetry for reading current heading on activation
    local_position_ = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

    // ── Parameters ────────────────────────────────────────────────
    if (!node_.has_parameter("cmd_vel_topic")) {
      node_.declare_parameter("cmd_vel_topic", std::string("/cmd_vel"));
    }
    const auto topic =
      node_.get_parameter("cmd_vel_topic").as_string();

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
      "RoverSpeedAttitudeMode: subscribing to '%s' (%s), timeout=%.2f s",
      topic.c_str(), use_stamped ? "TwistStamped" : "Twist", cmd_vel_timeout_s_);
  }

  void onActivate() override
  {
    RCLCPP_INFO(node_.get_logger(), "RoverSpeedAttitudeMode activated");
    // Seed the target heading with the current vehicle heading (NED yaw)
    yaw_setpoint_ = local_position_->heading();
    RCLCPP_INFO(
      node_.get_logger(), "Initial heading: %.1f deg",
      static_cast<double>(px4_ros2::radToDeg(yaw_setpoint_)));
    {
      std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
      last_cmd_vel_ = geometry_msgs::msg::Twist{};
      cmd_vel_received_ = false;
      cmd_vel_timed_out_ = false;
    }
  }

  void onDeactivate() override
  {
    RCLCPP_INFO(node_.get_logger(), "RoverSpeedAttitudeMode deactivated — sending zero setpoint");
    speed_attitude_setpoint_->update(0.0f, yaw_setpoint_);
  }

  void updateSetpoint(float dt_s) override
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

    // Before first cmd_vel: hold heading, zero speed
    if (!received) {
      speed_attitude_setpoint_->update(0.0f, yaw_setpoint_);
      return;
    }

    // After first cmd_vel: apply timeout watchdog
    if (timed_out) {
      if (!cmd_vel_timed_out_) {
        RCLCPP_WARN(
          node_.get_logger(),
          "cmd_vel timeout (>%.2f s) — zeroing speed, holding heading",
          cmd_vel_timeout_s_);
        cmd_vel_timed_out_ = true;
      }
      speed_attitude_setpoint_->update(0.0f, yaw_setpoint_);
      return;
    }

    if (cmd_vel_timed_out_) {
      RCLCPP_INFO(node_.get_logger(), "cmd_vel resumed — restoring setpoint");
      cmd_vel_timed_out_ = false;
    }

    // Forward speed in body x [m/s]
    float speed_body_x = static_cast<float>(cmd.linear.x);

    // Integrate yaw rate into heading setpoint.
    // ROS angular.z: CCW+ about Z-up → PX4 yaw: CW+ about Z-down → negate
    float yaw_rate_ned = static_cast<float>(-cmd.angular.z);
    yaw_setpoint_ += yaw_rate_ned * dt_s;

    // Wrap to [-pi, pi]
    yaw_setpoint_ = wrapPi(yaw_setpoint_);

    speed_attitude_setpoint_->update(speed_body_x, yaw_setpoint_);
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

  /// Wrap angle to [-pi, pi]
  static float wrapPi(float angle)
  {
    while (angle > static_cast<float>(M_PI)) {
      angle -= static_cast<float>(2.0 * M_PI);
    }
    while (angle < static_cast<float>(-M_PI)) {
      angle += static_cast<float>(2.0 * M_PI);
    }
    return angle;
  }

  rclcpp::Node & node_;
  std::shared_ptr<px4_ros2::RoverSpeedAttitudeSetpointType> speed_attitude_setpoint_;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> local_position_;

  // Subscriptions — both active on the same topic; ROS2 DDS type-matches automatically
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_sub_;

  geometry_msgs::msg::Twist last_cmd_vel_;
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
  std::mutex cmd_vel_mutex_;

  float yaw_setpoint_{0.0f};
  double cmd_vel_timeout_s_{2.0};
  bool cmd_vel_received_{false};
  bool cmd_vel_timed_out_{false};
};
