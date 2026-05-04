// Copyright 2026, Tao Wang. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <px4_ros2/components/mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>

namespace px4_bringup
{

// Announce a registered PX4 mode's runtime nav_state ID on a custom topic
// so cross-process consumers (e.g. rover_monitor's telemetry_publisher) can
// reliably build a name->mode_id map.
//
// Why not just snoop /fmu/out/register_ext_component_reply: that topic is
// fire-once per registration with BEST_EFFORT QoS and a shallow PX4-side
// uORB queue (depth 2), so a late-joining or briefly-busy subscriber can
// miss an entry. RELIABLE + TRANSIENT_LOCAL on this announcement topic
// guarantees delivery and gives late joiners the cached announcement.
//
// Returns the publisher SharedPtr so the caller can keep it alive for the
// full session (drop-on-destruction otherwise empties the durability cache).
template<typename ModeT>
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
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
  return pub;
}

}  // namespace px4_bringup
