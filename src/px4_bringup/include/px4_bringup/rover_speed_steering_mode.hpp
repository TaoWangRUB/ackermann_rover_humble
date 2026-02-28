// Copyright 2026, Tao Wang. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rover/speed_steering.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Rover Speed+Steering mode — maps cmd_vel to normalized speed and steering.
 *
 * Subscribes to /cmd_vel (body FLU). Extracts:
 *   - linear.x  → speed_body_x [m/s]
 *   - angular.z → normalized steering [-1 (left), 1 (right)]
 *
 * angular.z (rad/s yaw rate, CCW+) is normalized by dividing by max_steering_rate
 * and clamping to [-1, 1]. The sign is negated because ROS uses CCW+ while
 * PX4 steering uses right-positive.
 *
 * No frame conversion needed — both values are body-frame scalars.
 */
class RoverSpeedSteeringMode : public px4_ros2::ModeBase
{
public:
  explicit RoverSpeedSteeringMode(rclcpp::Node & node)
  : ModeBase(node, Settings{"Rover Speed Steering"}),
    node_(node)
  {
    speed_steering_setpoint_ =
      std::make_shared<px4_ros2::RoverSpeedSteeringSetpointType>(*this);

    cmd_vel_sub_ = node_.create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
        last_cmd_vel_ = *msg;
      });

    // Max steering rate [rad/s] used to normalize angular.z to [-1, 1].
    // Should match PX4 RA_MAX_STR_ANG or your physical steering limit.
    if (!node_.has_parameter("max_steering_rate")) {
      node_.declare_parameter("max_steering_rate", 1.0);
    }
    max_steering_rate_ =
      static_cast<float>(node_.get_parameter("max_steering_rate").as_double());
  }

  void onActivate() override
  {
    RCLCPP_INFO(node_.get_logger(), "RoverSpeedSteeringMode activated");
  }

  void onDeactivate() override
  {
    RCLCPP_INFO(node_.get_logger(), "RoverSpeedSteeringMode deactivated");
  }

  void updateSetpoint(float /*dt_s*/) override
  {
    geometry_msgs::msg::Twist cmd;
    {
      std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
      cmd = last_cmd_vel_;
    }

    // Forward speed in body x [m/s]
    float speed_body_x = static_cast<float>(cmd.linear.x);

    // Normalize yaw rate to [-1, 1] and negate (ROS CCW+ → PX4 right+)
    float normalized_steering = 0.0f;
    if (max_steering_rate_ > 0.0f) {
      normalized_steering =
        std::clamp(static_cast<float>(-cmd.angular.z) / max_steering_rate_, -1.0f, 1.0f);
    }

    speed_steering_setpoint_->update(speed_body_x, normalized_steering);
  }

private:
  rclcpp::Node & node_;
  std::shared_ptr<px4_ros2::RoverSpeedSteeringSetpointType> speed_steering_setpoint_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  geometry_msgs::msg::Twist last_cmd_vel_;
  std::mutex cmd_vel_mutex_;

  float max_steering_rate_{1.0f};
};
