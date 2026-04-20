/**
 * px4_vision_odom_node.cpp
 *
 * C++ replacement for px4_vision_odom.py — converts ROS nav_msgs/Odometry
 * (ENU/FLU) to PX4 VehicleOdometry (NED/FRD) for the EKF2 visual-inertial
 * fusion pipeline.
 *
 * Pipeline (lazy conversion — convert only at publish time):
 *   1. Subscribe to odom topic, stash latest message (no math in callback).
 *   2. Timer (10 Hz) grabs latest message, does TF lookup + conversion, publishes.
 *   3. Watchdog timer (1 Hz) detects odom timeout.
 *
 * Conversion:
 *   Position:    ENU [E, N, U] → NED [N, E, D]
 *   Orientation: ENU/FLU quaternion → NED/FRD quaternion
 *     q_ned_frd = q_enu_to_ned ⊗ q_enu_flu ⊗ q_flu_to_frd
 *   Velocity:    body FLU → body FRD (negate Y and Z)
 *
 * Parameters:
 *   odom_topic  (string, "/odometry/filtered")  source odometry topic
 *   odom_frame  (string, "odom")                TF frame for world ENU
 *   base_frame  (string, "cubepilot_link")      TF frame for body FLU
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <string>

using nav_msgs::msg::Odometry;
using px4_msgs::msg::VehicleOdometry;

class PX4VisionOdomNode : public rclcpp::Node
{
public:
  PX4VisionOdomNode()
  : Node("vio_publisher"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ── Parameters ─────────────────────────────────────────────────────
    declare_parameter("odom_topic", "/odometry/filtered");
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "cubepilot_link");

    odom_topic_ = get_parameter("odom_topic").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    // ── Pre-computed frame-change quaternions ──────────────────────────
    // ENU→NED: 180° about (1,1,0)/√2  →  q = (√0.5, √0.5, 0, 0) in (x,y,z,w)
    q_enu_to_ned_ = Eigen::Quaterniond(0.0, std::sqrt(0.5), std::sqrt(0.5), 0.0);  // (w,x,y,z)
    q_enu_to_ned_.normalize();
    // FLU→FRD: 180° about X  →  q = (0, 1, 0, 0) in (w,x,y,z)
    q_flu_to_frd_ = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);
    q_flu_to_frd_.normalize();

    // ── QoS ────────────────────────────────────────────────────────────
    rclcpp::QoS qos(10);
    qos.best_effort();

    // ── I/O ────────────────────────────────────────────────────────────
    odom_sub_ = create_subscription<Odometry>(
      odom_topic_, qos,
      std::bind(&PX4VisionOdomNode::odom_callback, this, std::placeholders::_1));

    px4_pub_ = create_publisher<VehicleOdometry>(
      "/fmu/in/vehicle_visual_odometry", 10);

    // ── Timers ─────────────────────────────────────────────────────────
    // 10 Hz publish — avoids XRCE-DDS cycle stalls on STM32F427 (Cube Black).
    pub_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PX4VisionOdomNode::pub_px4_odom, this));

    watchdog_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&PX4VisionOdomNode::check_odom_status, this));

    RCLCPP_INFO(get_logger(),
      "VIOPublisher (C++) ready — topic: %s | odom_frame: %s | base_frame: %s",
      odom_topic_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  // ── Odom callback (trivial stash — no math) ──────────────────────────

  void odom_callback(const Odometry::SharedPtr msg)
  {
    const auto now_ns = now().nanoseconds();
    if ((now_ns - last_accept_ns_) < min_interval_ns_) {
      return;
    }
    last_accept_ns_ = now_ns;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_raw_msg_ = msg;
    }

    if (!is_publishing_) {
      RCLCPP_INFO(get_logger(), "%s → /fmu/in/vehicle_visual_odometry",
                  odom_sub_->get_topic_name());
      is_publishing_ = true;
    }
    last_odom_time_ = now();
  }

  // ── Publish timer (10 Hz) — does conversion here ─────────────────────

  void pub_px4_odom()
  {
    Odometry::SharedPtr msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!is_publishing_ || !last_raw_msg_) {
        return;
      }
      msg = last_raw_msg_;
    }

    // ── Step 1: pose from TF / message pose ────────────────────────────
    Eigen::Vector3d    pos_enu;
    Eigen::Quaterniond q_enu;

    if (msg->child_frame_id == base_frame_) {
      const auto & p = msg->pose.pose.position;
      const auto & o = msg->pose.pose.orientation;
      pos_enu = Eigen::Vector3d(p.x, p.y, p.z);
      q_enu = Eigen::Quaterniond(o.w, o.x, o.y, o.z).normalized();
    } else {
      try {
        auto tf = tf_buffer_.lookupTransform(
          odom_frame_, base_frame_, tf2::TimePointZero);
        const auto & t = tf.transform.translation;
        const auto & r = tf.transform.rotation;
        pos_enu = Eigen::Vector3d(t.x, t.y, t.z);
        q_enu = Eigen::Quaterniond(r.w, r.x, r.y, r.z).normalized();
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "TF %s → %s unavailable: %s",
          odom_frame_.c_str(), base_frame_.c_str(), ex.what());
        // Fallback: use message pose
        const auto & p = msg->pose.pose.position;
        const auto & o = msg->pose.pose.orientation;
        pos_enu = Eigen::Vector3d(p.x, p.y, p.z);
        q_enu = Eigen::Quaterniond(o.w, o.x, o.y, o.z).normalized();
      }
    }

    // ── Step 2: position ENU → NED ─────────────────────────────────────
    // ENU: x=East, y=North, z=Up  →  NED: x=North, y=East, z=Down
    float pos_ned[3] = {
      static_cast<float>(pos_enu.y()),
      static_cast<float>(pos_enu.x()),
      static_cast<float>(-pos_enu.z())
    };

    // ── Step 3: orientation ENU/FLU → NED/FRD ──────────────────────────
    // q_ned_frd = q_enu_to_ned ⊗ q_enu_flu ⊗ q_flu_to_frd
    Eigen::Quaterniond q_ned_frd = (q_enu_to_ned_ * q_enu * q_flu_to_frd_).normalized();
    // PX4 VehicleOdometry expects q = [w, x, y, z]
    float q_px4[4] = {
      static_cast<float>(q_ned_frd.w()),
      static_cast<float>(q_ned_frd.x()),
      static_cast<float>(q_ned_frd.y()),
      static_cast<float>(q_ned_frd.z())
    };

    // ── Step 4: velocity body FLU → body FRD ───────────────────────────
    // FLU→FRD: forward same, left→right(−y), up→down(−z)
    const auto & lv = msg->twist.twist.linear;
    const auto & av = msg->twist.twist.angular;
    float v_frd[3] = {
      static_cast<float>(lv.x),
      static_cast<float>(-lv.y),
      static_cast<float>(-lv.z)
    };
    float w_frd[3] = {
      static_cast<float>(av.x),
      static_cast<float>(-av.y),
      static_cast<float>(-av.z)
    };

    // ── Populate VehicleOdometry ────────────────────────────────────────
    VehicleOdometry px4_msg;
    px4_msg.timestamp = static_cast<uint64_t>(
      now().nanoseconds() / 1000);
    px4_msg.timestamp_sample = static_cast<uint64_t>(
      msg->header.stamp.sec) * 1'000'000 +
      msg->header.stamp.nanosec / 1000;

    px4_msg.pose_frame = VehicleOdometry::POSE_FRAME_NED;
    px4_msg.position[0] = pos_ned[0];
    px4_msg.position[1] = pos_ned[1];
    px4_msg.position[2] = pos_ned[2];
    px4_msg.position_variance[0] = 0.01f;
    px4_msg.position_variance[1] = 0.01f;
    px4_msg.position_variance[2] = 0.01f;

    px4_msg.q[0] = q_px4[0];
    px4_msg.q[1] = q_px4[1];
    px4_msg.q[2] = q_px4[2];
    px4_msg.q[3] = q_px4[3];
    px4_msg.orientation_variance[0] = 0.01f;
    px4_msg.orientation_variance[1] = 0.01f;
    px4_msg.orientation_variance[2] = 0.01f;

    px4_msg.velocity_frame = VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
    px4_msg.velocity[0] = v_frd[0];
    px4_msg.velocity[1] = v_frd[1];
    px4_msg.velocity[2] = v_frd[2];
    px4_msg.velocity_variance[0] = 0.01f;
    px4_msg.velocity_variance[1] = 0.01f;
    px4_msg.velocity_variance[2] = 0.01f;

    px4_msg.angular_velocity[0] = w_frd[0];
    px4_msg.angular_velocity[1] = w_frd[1];
    px4_msg.angular_velocity[2] = w_frd[2];

    px4_msg.quality = 100;

    px4_pub_->publish(px4_msg);
  }

  // ── Watchdog (1 Hz) ──────────────────────────────────────────────────

  void check_odom_status()
  {
    if (!is_publishing_) {return;}
    auto elapsed = (now() - last_odom_time_).seconds();
    if (elapsed > odom_timeout_s_) {
      RCLCPP_WARN(get_logger(), "Odom topic silent for %.1f s.", elapsed);
      is_publishing_ = false;
    }
  }

  // ── Members ──────────────────────────────────────────────────────────

  // TF
  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Parameters (cached once)
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  // Pre-computed constants
  Eigen::Quaterniond q_enu_to_ned_;
  Eigen::Quaterniond q_flu_to_frd_;

  // I/O
  rclcpp::Subscription<Odometry>::SharedPtr      odom_sub_;
  rclcpp::Publisher<VehicleOdometry>::SharedPtr   px4_pub_;
  rclcpp::TimerBase::SharedPtr                   pub_timer_;
  rclcpp::TimerBase::SharedPtr                   watchdog_timer_;

  // State
  std::mutex           mutex_;
  Odometry::SharedPtr  last_raw_msg_;
  static constexpr int64_t min_interval_ns_ = 33'333'333;  // 30 Hz
  int64_t              last_accept_ns_ = 0;
  bool                 is_publishing_ = false;
  rclcpp::Time         last_odom_time_{0, 0, RCL_ROS_TIME};
  double               odom_timeout_s_ = 5.0;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4VisionOdomNode>());
  rclcpp::shutdown();
  return 0;
}
