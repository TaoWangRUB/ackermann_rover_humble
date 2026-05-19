// Copyright 2026, Tao Wang. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <px4_ros2/components/mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <utility>

namespace px4_bringup
{

// Holds the publisher and periodic re-announce timer. Both must stay alive
// for the full session so the durability cache is maintained and late-joining
// or restarted subscribers receive the announcement.
struct ModeAnnounceHandle
{
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
  rclcpp::TimerBase::SharedPtr timer;
};

// Announce a registered PX4 mode's runtime nav_state ID on a custom topic
// so cross-process consumers (e.g. rover_monitor's telemetry_publisher) can
// reliably build a name->mode_id map.
//
// Publishes immediately and then re-publishes every 5 s so that subscribers
// which (re)start or miss the initial delivery still get the mapping.
//
// Uses RELIABLE + TRANSIENT_LOCAL QoS for guaranteed delivery and late-joiner
// cache.
template<typename ModeT>
ModeAnnounceHandle
announce_mode(const std::shared_ptr<rclcpp::Node> & node,
  const std::string & mode_name,
  ModeT & mode)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(16)).reliable().transient_local();
  auto pub = node->create_publisher<std_msgs::msg::String>(
    "/px4_modes/announce", qos);

  std_msgs::msg::String msg;
  msg.data = mode_name + ":" + std::to_string(static_cast<int>(mode.id()));
  pub->publish(msg);

  RCLCPP_INFO(node->get_logger(), "Announced mode '%s' with nav_state=%d",
    mode_name.c_str(), static_cast<int>(mode.id()));

  // Re-announce every 5 s to recover from subscriber restarts / DDS
  // discovery races.
  auto timer = node->create_wall_timer(
    std::chrono::seconds(5),
    [pub, msg]() { pub->publish(msg); });

  return {pub, timer};
}

}  // namespace px4_bringup
