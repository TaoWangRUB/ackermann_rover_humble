/**
 * odom_tf_relay.cpp
 *
 * Subscribes to an odometry topic (nav_msgs/Odometry) whose child_frame_id is
 * some sensor frame (e.g. t265_pose_frame, cubepilot_link) and re-expresses it
 * so that child_frame_id becomes a configurable base_link frame (e.g.
 * ackermann/base_link).
 *
 * The rigid-body transform between the sensor frame and base_link is looked up
 * once from the static TF tree (populated by robot_state_publisher via the URDF)
 * and cached for subsequent messages.
 *
 * Pose composition (T_WC received, T_CB from TF):
 *   pos_WB = pos_WC + R(q_WC) * pos_CB
 *   q_WB   = q_WC ⊗ q_CB
 *
 * Origin alignment:
 *   The first computed (pos_WB, q_WB) is latched and subtracted from all
 *   subsequent outputs so that the odom origin coincides with base_link at
 *   t=0.  Without this, the output starts at pos_CB (the sensor→base_link
 *   mount offset), which misaligns it with other odom sources (e.g.
 *   robot_localization EKF) that define origin at base_link.
 *
 * Velocity (rigid-body, lever-arm corrected, all quaternions [x,y,z,w]):
 *   v_B  = R_BC * (v_C + ω_C × pos_CB)
 *   ω_B  = R_BC * ω_C
 *   where R_BC = rotation by conj(q_CB)
 *
 * Parameters:
 *   input_topic   (string, "/odom")              source odometry topic
 *   output_topic  (string, "/odom_base")         remapped output topic
 *   base_frame    (string, "ackermann/base_link") desired child_frame_id
 *   output_frame  (string, "")                   frame_id override (empty = keep input)
 *   tf_timeout_s  (double, 1.0)                  TF lookup timeout on first call
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <optional>
#include <string>

using nav_msgs::msg::Odometry;
using geometry_msgs::msg::TransformStamped;

class OdomTfRelay : public rclcpp::Node
{
public:
  OdomTfRelay()
  : Node("odom_tf_relay"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter("input_topic",  "/odom");
    declare_parameter("output_topic", "/odom_base");
    declare_parameter("base_frame",   "ackermann/base_link");
    declare_parameter("output_frame", "");
    declare_parameter("tf_timeout_s", 1.0);
    declare_parameter("publish_tf", false);
    declare_parameter("max_rate_hz", 0.0);

    input_topic_  = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    base_frame_   = get_parameter("base_frame").as_string();
    output_frame_ = get_parameter("output_frame").as_string();
    tf_timeout_   = get_parameter("tf_timeout_s").as_double();
    publish_tf_   = get_parameter("publish_tf").as_bool();
    max_rate_hz_  = get_parameter("max_rate_hz").as_double();
    min_interval_ns_ = (max_rate_hz_ > 0.0)
        ? static_cast<int64_t>(1.0e9 / max_rate_hz_)
        : 0;

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    pub_ = create_publisher<Odometry>(output_topic_, 10);
    sub_ = create_subscription<Odometry>(
      input_topic_, 10,
      std::bind(&OdomTfRelay::callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "odom_tf_relay ready:\n"
      "  %s  →  %s\n"
      "  child_frame: <from msg>  →  %s\n"
      "  max_rate_hz: %.1f%s",
      input_topic_.c_str(), output_topic_.c_str(), base_frame_.c_str(),
      max_rate_hz_, (max_rate_hz_ <= 0.0 ? " (unlimited)" : ""));
  }

private:
  // ── helpers ──────────────────────────────────────────────────────────────

  static Eigen::Vector3d rotate(const Eigen::Quaterniond & q, const Eigen::Vector3d & v)
  {
    return q * v;
  }

  // Rotate all four 3×3 blocks of a flat 6×6 row-major covariance array by R.
  // Layout: [pos(0..2), rot(3..5)] × [pos(0..2), rot(3..5)]
  static std::array<double, 36> rotate_covariance(
    const std::array<double, 36> & cov,
    const Eigen::Matrix3d & R)
  {
    // Extract the four 3×3 blocks: pp, pr, rp, rr
    auto get_block = [&](int r0, int c0) {
      Eigen::Matrix3d blk;
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          blk(i, j) = cov[(r0 + i) * 6 + (c0 + j)];
      return blk;
    };

    Eigen::Matrix3d pp = R * get_block(0, 0) * R.transpose();
    Eigen::Matrix3d pr = R * get_block(0, 3) * R.transpose();
    Eigen::Matrix3d rp = R * get_block(3, 0) * R.transpose();
    Eigen::Matrix3d rr = R * get_block(3, 3) * R.transpose();

    std::array<double, 36> out{};
    auto set_block = [&](const Eigen::Matrix3d & blk, int r0, int c0) {
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          out[(r0 + i) * 6 + (c0 + j)] = blk(i, j);
    };
    set_block(pp, 0, 0);
    set_block(pr, 0, 3);
    set_block(rp, 3, 0);
    set_block(rr, 3, 3);
    return out;
  }

  // ── TF cache ─────────────────────────────────────────────────────────────

  // Returns T_CB: pose of base_frame expressed in sensor_frame.
  // Cached after first successful lookup.
  bool get_tf_cb(const std::string & sensor_frame,
                 Eigen::Vector3d & pos_CB,
                 Eigen::Quaterniond & q_CB)
  {
    if (tf_cached_) {
      pos_CB = pos_CB_;
      q_CB   = q_CB_;
      return true;
    }

    try {
      rclcpp::Duration timeout = rclcpp::Duration::from_seconds(tf_timeout_);
      TransformStamped tf = tf_buffer_.lookupTransform(
        sensor_frame, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_));

      pos_CB_ = Eigen::Vector3d(
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z);

      q_CB_ = Eigen::Quaterniond(
        tf.transform.rotation.w,
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z).normalized();

      tf_cached_      = true;
      cached_sensor_  = sensor_frame;

      RCLCPP_INFO(get_logger(),
        "TF %s → %s cached: t=[%.3f %.3f %.3f]",
        sensor_frame.c_str(), base_frame_.c_str(),
        pos_CB_.x(), pos_CB_.y(), pos_CB_.z());

      pos_CB = pos_CB_;
      q_CB   = q_CB_;
      return true;
    } catch (const tf2::TransformException & ex) {
      if (!tf_warn_printed_) {
        RCLCPP_WARN(get_logger(),
          "TF %s → %s unavailable: %s — using identity until TF is ready.",
          sensor_frame.c_str(), base_frame_.c_str(), ex.what());
        tf_warn_printed_ = true;
      }
      pos_CB = Eigen::Vector3d::Zero();
      q_CB   = Eigen::Quaterniond::Identity();
      return false;
    }
  }

  // ── main callback ────────────────────────────────────────────────────────

  void callback(const Odometry::SharedPtr msg)
  {
    // Rate limiting — skip processing if below minimum interval
    if (min_interval_ns_ > 0) {
      int64_t now_ns = now().nanoseconds();
      if ((now_ns - last_pub_time_ns_) < min_interval_ns_) {
        return;
      }
      last_pub_time_ns_ = now_ns;
    }

    const std::string & sensor_frame = msg->child_frame_id;

    // If child_frame_id already is base_frame, just relay with header update.
    if (sensor_frame == base_frame_) {
      auto out = *msg;
      if (!output_frame_.empty()) {
        out.header.frame_id = output_frame_;
      }
      pub_->publish(out);
      return;
    }

    // Invalidate cache if sensor frame changed (shouldn't happen in practice).
    if (tf_cached_ && cached_sensor_ != sensor_frame) {
      RCLCPP_WARN(get_logger(),
        "child_frame_id changed from %s to %s — clearing TF cache",
        cached_sensor_.c_str(), sensor_frame.c_str());
      tf_cached_       = false;
      tf_warn_printed_ = false;
    }

    Eigen::Vector3d    pos_CB;
    Eigen::Quaterniond q_CB;
    get_tf_cb(sensor_frame, pos_CB, q_CB);   // falls back to identity on failure

    // ── Pose composition ─────────────────────────────────────────────────
    const auto & p = msg->pose.pose.position;
    const auto & o = msg->pose.pose.orientation;

    Eigen::Vector3d    pos_WC(p.x, p.y, p.z);
    Eigen::Quaterniond q_WC(o.w, o.x, o.y, o.z);
    q_WC.normalize();

    Eigen::Vector3d    pos_WB = pos_WC + rotate(q_WC, pos_CB);
    Eigen::Quaterniond q_WB   = (q_WC * q_CB).normalized();

    // ── Origin alignment: subtract first pose so odom starts at (0,0,0) ─
    if (!origin_latched_) {
      origin_pos_     = pos_WB;
      origin_q_inv_   = q_WB.conjugate();
      origin_latched_ = true;
      RCLCPP_INFO(get_logger(),
        "Origin latched: [%.3f %.3f %.3f] — subtracting from all outputs",
        origin_pos_.x(), origin_pos_.y(), origin_pos_.z());
    }
    pos_WB = origin_q_inv_ * (pos_WB - origin_pos_);
    q_WB   = (origin_q_inv_ * q_WB).normalized();

    // ── Velocity transform (rigid body) ──────────────────────────────────
    const auto & lv = msg->twist.twist.linear;
    const auto & av = msg->twist.twist.angular;

    Eigen::Vector3d v_C(lv.x, lv.y, lv.z);
    Eigen::Vector3d w_C(av.x, av.y, av.z);

    // R_BC = conj(q_CB) maps sensor-frame vectors to base-frame vectors
    Eigen::Quaterniond q_BC = q_CB.conjugate();

    Eigen::Vector3d v_B = rotate(q_BC, v_C + w_C.cross(pos_CB));
    Eigen::Vector3d w_B = rotate(q_BC, w_C);

    // ── Publish ──────────────────────────────────────────────────────────
    Odometry out;
    out.header.stamp    = msg->header.stamp;
    out.header.frame_id = output_frame_.empty() ? msg->header.frame_id : output_frame_;
    out.child_frame_id  = base_frame_;

    out.pose.pose.position.x    = pos_WB.x();
    out.pose.pose.position.y    = pos_WB.y();
    out.pose.pose.position.z    = pos_WB.z();
    out.pose.pose.orientation.x = q_WB.x();
    out.pose.pose.orientation.y = q_WB.y();
    out.pose.pose.orientation.z = q_WB.z();
    out.pose.pose.orientation.w = q_WB.w();

    out.twist.twist.linear.x  = v_B.x();
    out.twist.twist.linear.y  = v_B.y();
    out.twist.twist.linear.z  = v_B.z();
    out.twist.twist.angular.x = w_B.x();
    out.twist.twist.angular.y = w_B.y();
    out.twist.twist.angular.z = w_B.z();

    // Rotate covariance blocks into base_link frame.
    Eigen::Matrix3d R = q_BC.toRotationMatrix();
    out.pose.covariance  = rotate_covariance(msg->pose.covariance,  R);
    out.twist.covariance = rotate_covariance(msg->twist.covariance, R);

    pub_->publish(out);

    if (publish_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = out.header;
      tf.child_frame_id          = out.child_frame_id;
      tf.transform.translation.x = out.pose.pose.position.x;
      tf.transform.translation.y = out.pose.pose.position.y;
      tf.transform.translation.z = out.pose.pose.position.z;
      tf.transform.rotation      = out.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  // ── members ──────────────────────────────────────────────────────────────

  tf2_ros::Buffer           tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<Odometry>::SharedPtr    pub_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string base_frame_;
  std::string output_frame_;
  double      tf_timeout_;
  bool        publish_tf_;
  double      max_rate_hz_;
  int64_t     min_interval_ns_ = 0;
  int64_t     last_pub_time_ns_ = 0;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  bool               tf_cached_      = false;
  bool               tf_warn_printed_ = false;
  std::string        cached_sensor_;
  Eigen::Vector3d    pos_CB_        = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_CB_          = Eigen::Quaterniond::Identity();

  // Origin alignment — latch first (pos_WB, q_WB) to zero-out mount offset
  bool               origin_latched_ = false;
  Eigen::Vector3d    origin_pos_     = Eigen::Vector3d::Zero();
  Eigen::Quaterniond origin_q_inv_   = Eigen::Quaterniond::Identity();
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTfRelay>());
  rclcpp::shutdown();
  return 0;
}
