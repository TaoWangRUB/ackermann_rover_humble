// Copyright 2026, Tao Wang. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include <px4_bringup/rover_speed_steering_mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<px4_ros2::NodeWithMode<RoverSpeedSteeringMode>>(
      "rover_speed_steering_mode", true));
  rclcpp::shutdown();
  return 0;
}
