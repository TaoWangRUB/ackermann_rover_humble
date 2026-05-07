// Copyright 2026, Tao Wang. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include <px4_bringup/mode_announce.hpp>
#include <px4_bringup/rover_speed_rate_mode.hpp>
#include <px4_ros2/common/exception.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

// In-process retry loop — mirrors the other rover_*_main.cpp (ADR-004).
// The px4_ros2 HealthAndArmingChecks watchdog calls rclcpp::shutdown()
// before throwing on FMU disconnect, so a fresh rclcpp::init() is
// required per attempt.
static constexpr std::chrono::seconds kRetryDelay{5};

int main(int argc, char * argv[])
{
  while (true) {
    rclcpp::init(argc, argv);
    try {
      auto node =
        std::make_shared<px4_ros2::NodeWithMode<RoverSpeedRateMode>>(
          "rover_speed_rate_mode", true);
      auto announce_pub = px4_bringup::announce_mode(
        node, "Rover Speed Rate", node->getMode());
      rclcpp::spin(node);
      // Normal shutdown (SIGINT / rclcpp::shutdown from elsewhere)
      rclcpp::shutdown();
      break;
    } catch (const px4_ros2::Exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("rover_speed_rate_main"),
        "FMU unavailable: %s — retrying in %lds...",
        e.what(), kRetryDelay.count());
      rclcpp::shutdown();  // safe no-op if watchdog already called it
      std::this_thread::sleep_for(kRetryDelay);
    }
  }
  return 0;
}
