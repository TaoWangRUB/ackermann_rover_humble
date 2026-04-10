#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_FATAL(
    rclcpp::get_logger("realsense_camera_node"),
    "realsense_camera_bringup was built without librealsense2. "
    "Install librealsense2 so CMake can find realsense2Config.cmake under /usr/local, then rebuild.");
  rclcpp::shutdown();
  return 1;
}
