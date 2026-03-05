// Copyright 2026, Tao Wang. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/**
 * @brief Offboard trajectory mode — wraps cmd_vel into a PX4 TrajectorySetpoint.
 *
 * Subscribes to /cmd_vel (body FLU), transforms to odom (ENU) via TF2,
 * converts ENU → NED, and sends as a velocity TrajectorySetpoint.
 * The library handles the OffboardControlMode heartbeat automatically.
 */
class OffboardTrajectoryMode : public px4_ros2::ModeBase
{
public:
  explicit OffboardTrajectoryMode(rclcpp::Node & node)
  : ModeBase(node, Settings{"Offboard Trajectory"}),
    node_(node)
  {
    trajectory_setpoint_ =
      std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

    cmd_vel_sub_ = node_.create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
        last_cmd_vel_ = *msg;
      });

    // Declare parameters with defaults (will not redeclare if already set)
    if (!node_.has_parameter("base_frame")) {
      node_.declare_parameter("base_frame", "ackermann/base_link");
    }
    if (!node_.has_parameter("odom_frame")) {
      node_.declare_parameter("odom_frame", "odom");
    }

    base_frame_ = node_.get_parameter("base_frame").as_string();
    odom_frame_ = node_.get_parameter("odom_frame").as_string();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_.get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void onActivate() override
  {
    RCLCPP_INFO(node_.get_logger(), "OffboardTrajectoryMode activated");
  }

  void onDeactivate() override
  {
    RCLCPP_INFO(node_.get_logger(), "OffboardTrajectoryMode deactivated");
  }

  void updateSetpoint(float /*dt_s*/) override
  {
    geometry_msgs::msg::Twist cmd;
    {
      std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
      cmd = last_cmd_vel_;
    }

    try {
      // Look up transform from base to odom
      auto t = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, tf2::TimePointZero);

      // Create body-frame velocity vector
      geometry_msgs::msg::Vector3Stamped vel_base;
      vel_base.header.frame_id = base_frame_;
      vel_base.header.stamp = node_.get_clock()->now();
      vel_base.vector.x = cmd.linear.x;
      vel_base.vector.y = cmd.linear.y;
      vel_base.vector.z = cmd.linear.z;

      // Rotate into odom frame (ENU)
      geometry_msgs::msg::Vector3Stamped vel_odom;
      tf2::doTransform(vel_base, vel_odom, t);

      // ENU → NED: (y, x, -z)
      Eigen::Vector3f velocity_ned(
        static_cast<float>(vel_odom.vector.y),
        static_cast<float>(vel_odom.vector.x),
        static_cast<float>(-vel_odom.vector.z));

      // Yaw rate: ROS CCW+ about Z-up → PX4 CW+ about Z-down → negate
      float yawspeed_ned = static_cast<float>(-cmd.angular.z);

      trajectory_setpoint_->update(velocity_ned, {}, {}, yawspeed_ned);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(node_.get_logger(), "TF skip: %s", ex.what());
    }
  }

private:
  rclcpp::Node & node_;
  std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectory_setpoint_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  geometry_msgs::msg::Twist last_cmd_vel_;
  std::mutex cmd_vel_mutex_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string base_frame_;
  std::string odom_frame_;
};
