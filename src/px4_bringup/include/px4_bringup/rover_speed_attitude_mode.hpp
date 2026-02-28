// Copyright 2026, Tao Wang. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rover/speed_attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Rover Speed+Attitude mode — maps cmd_vel to speed and yaw heading.
 *
 * Subscribes to /cmd_vel (body FLU). Extracts:
 *   - linear.x  → speed_body_x [m/s]
 *   - angular.z → yaw rate [rad/s], integrated into a heading setpoint in NED
 *
 * The current vehicle heading is read from PX4 telemetry on activation.
 * angular.z is then integrated each cycle to produce an absolute yaw target.
 * PX4's Ackermann attitude controller handles heading → steering conversion.
 *
 * Sign convention: ROS angular.z is CCW+ about Z-up (ENU).
 * PX4 yaw is CW+ about Z-down (NED). We negate the rate before integrating.
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

    cmd_vel_sub_ = node_.create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
        last_cmd_vel_ = *msg;
      });
  }

  void onActivate() override
  {
    RCLCPP_INFO(node_.get_logger(), "RoverSpeedAttitudeMode activated");
    // Seed the target heading with the current vehicle heading (NED yaw)
    yaw_setpoint_ = local_position_->heading();
    RCLCPP_INFO(
      node_.get_logger(), "Initial heading: %.1f deg",
      static_cast<double>(px4_ros2::radToDeg(yaw_setpoint_)));
  }

  void onDeactivate() override
  {
    RCLCPP_INFO(node_.get_logger(), "RoverSpeedAttitudeMode deactivated");
  }

  void updateSetpoint(float dt_s) override
  {
    geometry_msgs::msg::Twist cmd;
    {
      std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
      cmd = last_cmd_vel_;
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

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  geometry_msgs::msg::Twist last_cmd_vel_;
  std::mutex cmd_vel_mutex_;

  float yaw_setpoint_{0.0f};
};
