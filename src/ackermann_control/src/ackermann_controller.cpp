#include <rclcpp_lifecycle/lifecycle_node.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ackermann_controller");
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
