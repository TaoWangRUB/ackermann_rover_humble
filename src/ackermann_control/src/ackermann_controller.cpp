#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

class AckermannControllerNode : public rclcpp::Node
{
public:
  AckermannControllerNode()
  : rclcpp::Node("ackermann_controller")
  {
    wheelbase_ = this->declare_parameter<double>("wheelbase", 0.32);
    max_steering_angle_ = this->declare_parameter<double>("max_steering_angle", 0.45);
    max_speed_ = this->declare_parameter<double>("max_speed", 2.0);
    input_cmd_vel_topic_ = this->declare_parameter<std::string>("input_cmd_vel_topic", "/cmd_vel");
    output_ackermann_topic_ = this->declare_parameter<std::string>("output_ackermann_topic", "/cmd_ackermann");

    pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(output_ackermann_topic_, 10);
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      input_cmd_vel_topic_, 10,
      std::bind(&AckermannControllerNode::onTwist, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Ackermann controller started: wheelbase=%.3f, max_steering=%.3f, max_speed=%.3f",
      wheelbase_, max_steering_angle_, max_speed_);
  }

private:
  void onTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double v = msg->linear.x;
    double yaw_rate = msg->angular.z;

    // Clamp speed
    if (v > max_speed_) v = max_speed_;
    if (v < -max_speed_) v = -max_speed_;

    double steering_angle = 0.0;
    if (std::abs(v) > 1e-3) {
      double curvature = yaw_rate / v; // kappa = omega / v
      steering_angle = std::atan(wheelbase_ * curvature);
    }
    // Clamp steering
    if (steering_angle > max_steering_angle_) steering_angle = max_steering_angle_;
    if (steering_angle < -max_steering_angle_) steering_angle = -max_steering_angle_;

    ackermann_msgs::msg::AckermannDriveStamped out;
    out.header.stamp = this->get_clock()->now();
    out.drive.speed = static_cast<float>(v);
    out.drive.steering_angle = static_cast<float>(steering_angle);
    pub_->publish(out);
  }

  double wheelbase_;
  double max_steering_angle_;
  double max_speed_;
  std::string input_cmd_vel_topic_;
  std::string output_ackermann_topic_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AckermannControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
