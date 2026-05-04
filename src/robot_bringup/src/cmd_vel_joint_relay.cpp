// cmd_vel_joint_relay.cpp
//
// Derives /joint_states from /cmd_vel using inverse Ackermann kinematics.
// Used in hardware mode (use_gazebo:=false) to keep the TF tree complete and
// animate the robot model in RViz. FOR VISUALISATION ONLY.
//
// Ported from cmd_vel_joint_relay.py to C++ to avoid the rclpy executor
// overhead at the timer + subscriber callback rate. Public behaviour is
// identical: same joint names, same kinematics, same default parameters
// (except publish_rate which is 10 Hz here vs 50 Hz in the Python version —
// 10 Hz is plenty for RViz wheel-mesh animation and cuts CPU ~5x).
//
// Kinematics (per publish cycle):
//   R       = linear.x / angular.z          (signed turn radius)
//   delta_L = atan2(L,  R - T/2)            (left  / inner kingpin)
//   delta_R = atan2(L,  R + T/2)            (right / outer kingpin)
//   omega   = linear.x / wheel_radius       (single ESC channel; same all 4)
//   theta_i += omega * dt                   (integrated for mesh spin)
//
// Joint name order matches ackermann_controller.yaml + URDF.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace
{
constexpr double kSteeringLimit = 0.6;  // rad — matches URDF front_*_wheel_steering_joint limits
}

class CmdVelJointRelay : public rclcpp::Node
{
public:
  CmdVelJointRelay() : Node("cmd_vel_joint_relay")
  {
    L_ = declare_parameter<double>("wheelbase", 0.174);
    T_ = declare_parameter<double>("track", 0.174);
    r_ = declare_parameter<double>("wheel_radius", 0.0385);
    const auto ns = declare_parameter<std::string>("robot_ns", "ackermann");
    const auto hz = declare_parameter<double>("publish_rate", 10.0);

    names_ = {
      ns + "/rear_left_wheel_joint",
      ns + "/rear_right_wheel_joint",
      ns + "/front_left_wheel_joint",
      ns + "/front_right_wheel_joint",
      ns + "/front_left_wheel_steering_joint",
      ns + "/front_right_wheel_steering_joint",
    };

    last_t_ = now();

    sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        vx_ = msg->twist.linear.x;
        wz_ = msg->twist.angular.z;
      });

    pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    const auto period = std::chrono::duration<double>(1.0 / hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publish(); });

    RCLCPP_INFO(get_logger(),
      "cmd_vel_joint_relay (C++) ready — listening on /cmd_vel (TwistStamped); "
      "L=%.3f m, T=%.3f m, r=%.4f m, ns=%s, rate=%.1f Hz",
      L_, T_, r_, ns.c_str(), hz);
  }

private:
  void ackermann_angles(double v, double w, double & dl, double & dr) const
  {
    if (std::abs(w) < 1e-6) {
      dl = dr = 0.0;
      return;
    }
    if (std::abs(v) < 1e-3) {
      // Pure-rotation command: kinematically ill-defined for Ackermann.
      // Apply max steering in the commanded direction; wheels stay still.
      const double s = std::copysign(kSteeringLimit, w);
      dl = dr = s;
      return;
    }
    const double R = v / w;
    dl = std::atan2(L_, R - T_ / 2.0);
    dr = std::atan2(L_, R + T_ / 2.0);
    dl = std::clamp(dl, -kSteeringLimit, kSteeringLimit);
    dr = std::clamp(dr, -kSteeringLimit, kSteeringLimit);
  }

  void publish()
  {
    const auto t = now();
    const double dt = (t - last_t_).seconds();
    last_t_ = t;

    const double v = vx_;
    const double w = wz_;

    // Both rear wheels commanded at the same speed (single ESC channel).
    // Front wheels free-roll; approximated at rear speed for display.
    const double omega = v / r_;

    wheel_pos_[0] += omega * dt;  // rear  left
    wheel_pos_[1] += omega * dt;  // rear  right
    wheel_pos_[2] += omega * dt;  // front left
    wheel_pos_[3] += omega * dt;  // front right

    double dl = 0.0;
    double dr = 0.0;
    ackermann_angles(v, w, dl, dr);

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = t;
    msg.name = names_;
    msg.position = {
      wheel_pos_[0], wheel_pos_[1], wheel_pos_[2], wheel_pos_[3],
      dl, dr,
    };
    msg.velocity = {omega, omega, omega, omega, 0.0, 0.0};
    pub_->publish(msg);
  }

  double L_ = 0.174;
  double T_ = 0.174;
  double r_ = 0.0385;

  std::vector<std::string> names_;
  std::array<double, 4> wheel_pos_{0.0, 0.0, 0.0, 0.0};

  double vx_ = 0.0;
  double wz_ = 0.0;
  rclcpp::Time last_t_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelJointRelay>());
  rclcpp::shutdown();
  return 0;
}
