/**
 * px4_vehicle_odometry_node.cpp
 *
 * C++ replacement for px4_vehicle_odometry.py — two-stage pipeline:
 *
 * Stage 1: /fmu/out/vehicle_odometry (NED/FRD) → /px4_vehicle_odom (ENU/FLU)
 *   Converts raw PX4 VehicleOdometry to ROS Odometry in ENU/FLU frame.
 *   Rate-gated at 30 Hz (PX4 EKF2 publishes at 50-100 Hz).
 *
 * Stage 2: /px4_vehicle_odom → /px4_vehicle_odom_base
 *   Re-expresses cubepilot-frame odometry in the rover base frame via TF.
 *
 * Coordinate frames:
 *   PX4 VehicleOdometry : position NED, orientation NED/FRD, q=[w,x,y,z]
 *   Stage-1 output      : position ENU, orientation ENU/FLU, q=[x,y,z,w]
 *   Stage-2 output      : position ENU, orientation ENU/FLU, q=[x,y,z,w]
 *                          re-expressed at base_frame via rigid-body TF
 *
 * NED→ENU position : [E, N, U] = [ned[1], ned[0], -ned[2]]
 * NED/FRD→ENU/FLU q: conj(q_enu_to_ned) ⊗ q_ned_frd ⊗ conj(q_flu_to_frd)
 * FRD→FLU velocity : negate y and z
 *
 * Stage-2 pose composition (T_WC = received, T_CB from TF lookup):
 *   pos_WB = pos_WC + R(q_WC) * pos_CB
 *   q_WB   = q_WC ⊗ q_CB
 *
 * Stage-2 velocity (rigid-body, all in cubepilot body frame):
 *   v_B_in_B  = R_BC * (v_C + ω_C × pos_CB)
 *   ω_B_in_B  = R_BC * ω_C
 *
 * Parameters:
 *   frame_id        (string, "vehicle_odom")        stage-1 world frame
 *   child_frame_id  (string, "cubepilot_link")      stage-1 body frame
 *   odom_topic      (string, "/px4_vehicle_odom")   stage-2 input topic
 *   odom_frame      (string, "odom")                stage-2 world frame
 *   base_frame      (string, "ackermann/base_link") stage-2 body frame
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
#include <string>

using nav_msgs::msg::Odometry;
using px4_msgs::msg::VehicleOdometry;

class PX4VehicleOdometryNode : public rclcpp::Node
{
public:
  PX4VehicleOdometryNode()
  : Node("px4_vehicle_odometry"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ── Stage-1 parameters ─────────────────────────────────────────────
    declare_parameter("frame_id", "vehicle_odom");
    declare_parameter("child_frame_id", "cubepilot_link");

    frame_id_ = get_parameter("frame_id").as_string();
    child_frame_id_ = get_parameter("child_frame_id").as_string();

    // ── Stage-2 parameters ─────────────────────────────────────────────
    declare_parameter("odom_topic", "/px4_vehicle_odom");
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "ackermann/base_link");

    odom_topic_ = get_parameter("odom_topic").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    // ── Pre-computed frame-change quaternions ──────────────────────────
    // ENU↔NED: 180° about (1,1,0)/√2 → q = (w=0, x=√0.5, y=√0.5, z=0)
    q_enu_to_ned_ = Eigen::Quaterniond(0.0, std::sqrt(0.5), std::sqrt(0.5), 0.0).normalized();
    // FLU↔FRD: 180° about X → q = (w=0, x=1, y=0, z=0)
    q_flu_to_frd_ = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0).normalized();

    // ── QoS ────────────────────────────────────────────────────────────
    rclcpp::QoS px4_qos(10);
    px4_qos.best_effort();

    // ── Stage-1: /fmu/out/vehicle_odometry → /px4_vehicle_odom ─────────
    stage1_sub_ = create_subscription<VehicleOdometry>(
      "/fmu/out/vehicle_odometry", px4_qos,
      std::bind(&PX4VehicleOdometryNode::stage1_callback, this, std::placeholders::_1));

    stage1_pub_ = create_publisher<Odometry>("/px4_vehicle_odom", 10);

    // ── Stage-2: /px4_vehicle_odom → /px4_vehicle_odom_base ────────────
    stage2_sub_ = create_subscription<Odometry>(
      odom_topic_, 10,
      std::bind(&PX4VehicleOdometryNode::stage2_callback, this, std::placeholders::_1));

    stage2_pub_ = create_publisher<Odometry>("/px4_vehicle_odom_base", 10);

    RCLCPP_INFO(get_logger(),
      "px4_vehicle_odometry (C++) ready\n"
      "  stage-1: /fmu/out/vehicle_odometry → /px4_vehicle_odom [%s → %s]\n"
      "  stage-2: %s → /px4_vehicle_odom_base [%s → %s]",
      frame_id_.c_str(), child_frame_id_.c_str(),
      odom_topic_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  // ── Stage-1 callback ─────────────────────────────────────────────────

  void stage1_callback(const VehicleOdometry::SharedPtr msg)
  {
    // ── Frame sanity checks ────────────────────────────────────────────
    if (msg->pose_frame != VehicleOdometry::POSE_FRAME_NED) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Unexpected pose_frame=%d (expected NED=%d). Dropped.",
        msg->pose_frame, VehicleOdometry::POSE_FRAME_NED);
      return;
    }

    auto vf = msg->velocity_frame;
    if (vf != VehicleOdometry::VELOCITY_FRAME_NED &&
      vf != VehicleOdometry::VELOCITY_FRAME_BODY_FRD)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Unexpected velocity_frame=%d. Dropped.", vf);
      return;
    }

    const auto now_ns = now().nanoseconds();
    if ((now_ns - last_stage1_ns_) < min_interval_ns_) {
      return;
    }
    last_stage1_ns_ = now_ns;

    const auto stamp_ns = now().nanoseconds();

    // ── Position: NED → ENU ────────────────────────────────────────────
    Eigen::Vector3d pos_enu(msg->position[1], msg->position[0], -msg->position[2]);

    // ── Orientation: NED/FRD → ENU/FLU ─────────────────────────────────
    // PX4 q = [w, x, y, z]
    Eigen::Quaterniond q_ned_frd(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    // q_enu_flu = conj(q_enu_to_ned) ⊗ q_ned_frd ⊗ conj(q_flu_to_frd)
    // For 180° rotations, conj is equivalent (same rotation).
    Eigen::Quaterniond q_enu_flu =
      (q_enu_to_ned_.conjugate() * q_ned_frd * q_flu_to_frd_.conjugate()).normalized();

    // ── Linear velocity → body FLU ─────────────────────────────────────
    Eigen::Vector3d v_flu;
    if (vf == VehicleOdometry::VELOCITY_FRAME_BODY_FRD) {
      // Body FRD → body FLU: negate y and z
      v_flu = Eigen::Vector3d(msg->velocity[0], -msg->velocity[1], -msg->velocity[2]);
    } else {
      // VELOCITY_FRAME_NED → world ENU → body FLU
      Eigen::Vector3d v_enu(msg->velocity[1], msg->velocity[0], -msg->velocity[2]);
      v_flu = q_enu_flu.conjugate() * v_enu;
    }

    // Angular velocity is always body FRD → FLU: negate y and z
    Eigen::Vector3d w_flu(msg->angular_velocity[0],
      -msg->angular_velocity[1],
      -msg->angular_velocity[2]);

    // ── Build Odometry ─────────────────────────────────────────────────
    Odometry odom;
    odom.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1'000'000'000LL);
    odom.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1'000'000'000LL);
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;

    odom.pose.pose.position.x = pos_enu.x();
    odom.pose.pose.position.y = pos_enu.y();
    odom.pose.pose.position.z = pos_enu.z();

    odom.pose.pose.orientation.x = q_enu_flu.x();
    odom.pose.pose.orientation.y = q_enu_flu.y();
    odom.pose.pose.orientation.z = q_enu_flu.z();
    odom.pose.pose.orientation.w = q_enu_flu.w();

    odom.twist.twist.linear.x = v_flu.x();
    odom.twist.twist.linear.y = v_flu.y();
    odom.twist.twist.linear.z = v_flu.z();

    odom.twist.twist.angular.x = w_flu.x();
    odom.twist.twist.angular.y = w_flu.y();
    odom.twist.twist.angular.z = w_flu.z();

    // Covariance: diagonal from PX4 variances
    auto set_diag = [](auto & cov, int idx, float val) {
        cov[idx] = (val > 0.0f) ? static_cast<double>(val) : 0.01;
      };
    set_diag(odom.pose.covariance, 0, msg->position_variance[0]);
    set_diag(odom.pose.covariance, 7, msg->position_variance[1]);
    set_diag(odom.pose.covariance, 14, msg->position_variance[2]);
    set_diag(odom.pose.covariance, 21, msg->orientation_variance[0]);
    set_diag(odom.pose.covariance, 28, msg->orientation_variance[1]);
    set_diag(odom.pose.covariance, 35, msg->orientation_variance[2]);

    set_diag(odom.twist.covariance, 0, msg->velocity_variance[0]);
    set_diag(odom.twist.covariance, 7, msg->velocity_variance[1]);
    set_diag(odom.twist.covariance, 14, msg->velocity_variance[2]);

    stage1_pub_->publish(odom);
  }

  // ── Stage-2 callback ─────────────────────────────────────────────────

  void stage2_callback(const Odometry::SharedPtr msg)
  {
    // ── Step 1: pose of cubepilot_link (C) in vehicle_odom (W) ─────────
    const auto & p = msg->pose.pose.position;
    const auto & o = msg->pose.pose.orientation;
    Eigen::Vector3d    pos_WC(p.x, p.y, p.z);
    Eigen::Quaterniond q_WC(o.w, o.x, o.y, o.z);
    q_WC.normalize();

    // ── Step 2: TF cubepilot_link → base_frame (cached permanently) ───
    Eigen::Vector3d    pos_CB;
    Eigen::Quaterniond q_CB;
    if (!get_tf_cb(msg->child_frame_id, pos_CB, q_CB)) {
      // Identity fallback already applied by get_tf_cb
    }

    // ── Step 3: compose pose ───────────────────────────────────────────
    Eigen::Vector3d    pos_WB = pos_WC + q_WC * pos_CB;
    Eigen::Quaterniond q_WB = (q_WC * q_CB).normalized();

    // ── Step 4: rigid-body velocity transform ──────────────────────────
    const auto & lv = msg->twist.twist.linear;
    const auto & av = msg->twist.twist.angular;
    Eigen::Vector3d v_C(lv.x, lv.y, lv.z);
    Eigen::Vector3d w_C(av.x, av.y, av.z);

    Eigen::Vector3d v_B_in_C = v_C + w_C.cross(pos_CB);
    Eigen::Quaterniond q_BC = q_CB.conjugate();
    Eigen::Vector3d v_B = q_BC * v_B_in_C;
    Eigen::Vector3d w_B = q_BC * w_C;

    // ── Step 5: publish ────────────────────────────────────────────────
    Odometry out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = odom_frame_;
    out.child_frame_id = base_frame_;

    out.pose.pose.position.x = pos_WB.x();
    out.pose.pose.position.y = pos_WB.y();
    out.pose.pose.position.z = pos_WB.z();
    out.pose.pose.orientation.x = q_WB.x();
    out.pose.pose.orientation.y = q_WB.y();
    out.pose.pose.orientation.z = q_WB.z();
    out.pose.pose.orientation.w = q_WB.w();

    out.twist.twist.linear.x = v_B.x();
    out.twist.twist.linear.y = v_B.y();
    out.twist.twist.linear.z = v_B.z();
    out.twist.twist.angular.x = w_B.x();
    out.twist.twist.angular.y = w_B.y();
    out.twist.twist.angular.z = w_B.z();

    out.pose.covariance = msg->pose.covariance;
    out.twist.covariance = msg->twist.covariance;

    stage2_pub_->publish(out);
  }

  // ── TF cache (permanent — static URDF transform) ────────────────────

  bool get_tf_cb(
    const std::string & sensor_frame,
    Eigen::Vector3d & pos_CB,
    Eigen::Quaterniond & q_CB)
  {
    if (tf_cached_ && cached_sensor_frame_ == sensor_frame) {
      pos_CB = cached_pos_CB_;
      q_CB = cached_q_CB_;
      return true;
    }

    try {
      auto tf = tf_buffer_.lookupTransform(
        sensor_frame, base_frame_, tf2::TimePointZero);
      const auto & t = tf.transform.translation;
      const auto & r = tf.transform.rotation;
      cached_pos_CB_ = Eigen::Vector3d(t.x, t.y, t.z);
      cached_q_CB_ = Eigen::Quaterniond(r.w, r.x, r.y, r.z).normalized();
      cached_sensor_frame_ = sensor_frame;
      tf_cached_ = true;

      RCLCPP_INFO(get_logger(),
        "TF %s → %s cached: t=[%.3f %.3f %.3f]",
        sensor_frame.c_str(), base_frame_.c_str(),
        cached_pos_CB_.x(), cached_pos_CB_.y(), cached_pos_CB_.z());

      pos_CB = cached_pos_CB_;
      q_CB = cached_q_CB_;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF %s → %s unavailable: %s",
        sensor_frame.c_str(), base_frame_.c_str(), ex.what());
      pos_CB = Eigen::Vector3d::Zero();
      q_CB = Eigen::Quaterniond::Identity();
      return false;
    }
  }

  // ── Members ──────────────────────────────────────────────────────────

  // TF
  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  bool               tf_cached_ = false;
  Eigen::Vector3d    cached_pos_CB_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond cached_q_CB_ = Eigen::Quaterniond::Identity();
  std::string        cached_sensor_frame_;

  // Parameters (cached once)
  std::string frame_id_;
  std::string child_frame_id_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  // Pre-computed constants
  Eigen::Quaterniond q_enu_to_ned_;
  Eigen::Quaterniond q_flu_to_frd_;

  // Stage-1 I/O + rate limiting
  rclcpp::Subscription<VehicleOdometry>::SharedPtr stage1_sub_;
  rclcpp::Publisher<Odometry>::SharedPtr           stage1_pub_;
  static constexpr int64_t min_interval_ns_ = 33'333'333;  // 30 Hz
  int64_t last_stage1_ns_ = 0;

  // Stage-2 I/O
  rclcpp::Subscription<Odometry>::SharedPtr stage2_sub_;
  rclcpp::Publisher<Odometry>::SharedPtr    stage2_pub_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4VehicleOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
