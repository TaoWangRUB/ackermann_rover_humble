// Copyright 2026 The Ackermann Rover Authors
// SPDX-License-Identifier: Apache-2.0

/**
 * cuvslam_rgbd_node.cpp
 *
 * Thin ROS 2 wrapper around cuvslam::Odometry for RGB-D visual-inertial
 * odometry using the Intel RealSense D435i.
 */

#include <cstdint>
#include <cstdlib>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cuvslam/cuvslam2.h>

namespace cuvslam_bringup
{

namespace
{

using nav_msgs::msg::Odometry;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::Imu;

using RgbdSyncPolicy =
  message_filters::sync_policies::ApproximateTime<Image, Image>;
using RgbdSynchronizer = message_filters::Synchronizer<RgbdSyncPolicy>;

int64_t stampToNs(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(rclcpp::Time(stamp).nanoseconds());
}

cuvslam::Pose transformToCuvslamPose(const geometry_msgs::msg::TransformStamped & tf_msg)
{
  cuvslam::Pose pose;
  pose.translation = {
    static_cast<float>(tf_msg.transform.translation.x),
    static_cast<float>(tf_msg.transform.translation.y),
    static_cast<float>(tf_msg.transform.translation.z),
  };
  pose.rotation = {
    static_cast<float>(tf_msg.transform.rotation.x),
    static_cast<float>(tf_msg.transform.rotation.y),
    static_cast<float>(tf_msg.transform.rotation.z),
    static_cast<float>(tf_msg.transform.rotation.w),
  };
  return pose;
}

bool fillDistortion(const CameraInfo & info, cuvslam::Camera & cam, rclcpp::Logger logger)
{
  const std::string & model = info.distortion_model;
  if (model == "plumb_bob" && info.d.size() >= 5) {
    cam.distortion.model = cuvslam::Distortion::Model::Brown;
    cam.distortion.parameters = {
      static_cast<float>(info.d[0]),
      static_cast<float>(info.d[1]),
      static_cast<float>(info.d[2]),
      static_cast<float>(info.d[3]),
      static_cast<float>(info.d[4]),
    };
    return true;
  }
  cam.distortion.model = cuvslam::Distortion::Model::Pinhole;
  cam.distortion.parameters.clear();
  return true;
}

cuvslam::Camera buildCuvslamCamera(
  const CameraInfo & info,
  const cuvslam::Pose & rig_from_camera,
  rclcpp::Logger logger)
{
  cuvslam::Camera cam;
  cam.size = {
    static_cast<int32_t>(info.width),
    static_cast<int32_t>(info.height),
  };
  cam.focal = {
    static_cast<float>(info.k[0]),
    static_cast<float>(info.k[4]),
  };
  cam.principal = {
    static_cast<float>(info.k[2]),
    static_cast<float>(info.k[5]),
  };
  cam.rig_from_camera = rig_from_camera;
  fillDistortion(info, cam, logger);
  return cam;
}

}  // namespace

class CuvslamRgbdNode : public rclcpp::Node
{
public:
  CuvslamRgbdNode()
  : Node("cuvslam_rgbd_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    rgb_image_topic_ = declare_parameter<std::string>(
      "rgb_image_topic", "/d435i/color/image_raw");
    depth_image_topic_ = declare_parameter<std::string>(
      "depth_image_topic", "/d435i/depth/image_rect_raw");
    rgb_camera_info_topic_ = declare_parameter<std::string>(
      "rgb_camera_info_topic", "/d435i/color/camera_info");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/d435i/imu");
    raw_odom_topic_ = declare_parameter<std::string>(
      "raw_odom_topic", "/cuvslam/raw_odometry");

    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    rig_frame_id_ = declare_parameter<std::string>(
      "rig_frame_id", "ackermann/base_link");
    imu_frame_id_param_ = declare_parameter<std::string>("imu_frame_id", "");

    bool requested_imu = declare_parameter<bool>("use_imu", true);
    if (requested_imu) {
      RCLCPP_WARN(
        get_logger(),
        "cuVSLAM OdometryMode::RGBD does NOT support IMU inertial fusion! "
        "Disabling IMU interface. (Rely on EKF for IMU yaw fusion instead).");
    }
    use_imu_ = false;
    sync_queue_size_ = declare_parameter<int>("sync_queue_size", 10);
    sync_slop_s_ = declare_parameter<double>("sync_slop_s", 0.05);
    rig_init_timeout_s_ = declare_parameter<double>("rig_init_timeout_s", 10.0);
    debug_ = declare_parameter<bool>("debug", false);

    published_child_frame_id_ = rig_frame_id_;

    cuvslam::SetVerbosity(debug_ ? 1 : 0);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    const auto info_qos = rclcpp::QoS(10).reliable();

    rgb_info_sub_ = create_subscription<CameraInfo>(
      rgb_camera_info_topic_, info_qos,
      std::bind(&CuvslamRgbdNode::rgbInfoCallback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<Imu>(
      imu_topic_, sensor_qos,
      std::bind(&CuvslamRgbdNode::imuCallback, this, std::placeholders::_1));

    rgb_image_sub_.subscribe(this, rgb_image_topic_, sensor_qos.get_rmw_qos_profile());
    depth_image_sub_.subscribe(this, depth_image_topic_, sensor_qos.get_rmw_qos_profile());

    sync_ = std::make_shared<RgbdSynchronizer>(
      RgbdSyncPolicy(sync_queue_size_), rgb_image_sub_, depth_image_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_s_));
    sync_->registerCallback(
      std::bind(&CuvslamRgbdNode::rgbdCallback, this, std::placeholders::_1, std::placeholders::_2));

    odom_pub_ = create_publisher<Odometry>(raw_odom_topic_, rclcpp::QoS(10));

    init_start_ = now();
    RCLCPP_INFO(get_logger(), "cuvslam_rgbd_node starting");
  }

private:
  void rgbInfoCallback(const CameraInfo::SharedPtr msg)
  {
    rgb_info_ = *msg;
    tryInitRig();
  }

  void imuCallback(const Imu::SharedPtr msg)
  {
    if (!use_imu_) {
      return;
    }
    const int64_t ts_ns = stampToNs(msg->header.stamp);
    if (ts_ns <= last_imu_received_ts_ns_ || ts_ns <= last_track_ts_ns_) {
      return;
    }
    last_imu_received_ts_ns_ = ts_ns;

    cuvslam::ImuMeasurement meas;
    meas.timestamp_ns = ts_ns;
    meas.linear_accelerations = {
      static_cast<float>(msg->linear_acceleration.x),
      static_cast<float>(msg->linear_acceleration.y),
      static_cast<float>(msg->linear_acceleration.z),
    };
    meas.angular_velocities = {
      static_cast<float>(msg->angular_velocity.x),
      static_cast<float>(msg->angular_velocity.y),
      static_cast<float>(msg->angular_velocity.z),
    };
    pending_imu_.push_back(meas);
  }

  void rgbdCallback(const Image::ConstSharedPtr & rgb_msg, const Image::ConstSharedPtr & depth_msg)
  {
    if (!tracker_) {
      return;
    }

    const int64_t rgb_ts_ns = stampToNs(rgb_msg->header.stamp);
    if (last_track_ts_ns_ != 0 && rgb_ts_ns <= last_track_ts_ns_) {
      return;
    }

    cv_bridge::CvImageConstPtr rgb_cv;
    cv_bridge::CvImageConstPtr depth_cv;
    try {
      rgb_cv = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::MONO8); // cuvslam accepts MONO for RGBD tracking easily 
      depth_cv = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge failed: %s", e.what());
      return;
    }

    if (use_imu_) {
      flushImuMeasurements(rgb_ts_ns);
    }

    cuvslam::Image rgb_image;
    rgb_image.pixels = rgb_cv->image.data;
    rgb_image.width = rgb_cv->image.cols;
    rgb_image.height = rgb_cv->image.rows;
    rgb_image.pitch = static_cast<int32_t>(rgb_cv->image.step);
    rgb_image.encoding = cuvslam::ImageData::Encoding::MONO;
    rgb_image.data_type = cuvslam::ImageData::DataType::UINT8;
    rgb_image.is_gpu_mem = false;
    rgb_image.timestamp_ns = rgb_ts_ns;
    rgb_image.camera_index = 0;

    cuvslam::Image depth_image = rgb_image;
    depth_image.pixels = depth_cv->image.data;
    depth_image.width = depth_cv->image.cols;
    depth_image.height = depth_cv->image.rows;
    depth_image.pitch = static_cast<int32_t>(depth_cv->image.step);
    depth_image.encoding = cuvslam::ImageData::Encoding::MONO;
    depth_image.data_type = cuvslam::ImageData::DataType::UINT16;

    cuvslam::Odometry::ImageSet images = {rgb_image};
    cuvslam::Odometry::ImageSet depths = {depth_image};

    cuvslam::PoseEstimate estimate;
    try {
      estimate = tracker_->Track(images, {}, depths);
      last_track_ts_ns_ = rgb_ts_ns;
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "cuVSLAM Track() failed: %s", e.what());
      return;
    }

    if (!estimate.world_from_rig.has_value()) {
      return;
    }

    publishOdometry(*estimate.world_from_rig, rgb_msg->header.stamp);
  }

  void tryInitRig()
  {
    if (tracker_ || !rgb_info_.has_value()) {
      return;
    }

    cuvslam::Pose rgb_rig_from_cam;
    cuvslam::Pose imu_rig_from_imu;

    std::string err;
    try {
      if (use_imu_) {
        std::string imu_frame = imu_frame_id_param_;
        if (imu_frame.empty()) {
            imu_frame = "d435i_imu_optical_frame";
        }
        auto t_imu = tf_buffer_.lookupTransform(
          rig_frame_id_, imu_frame, tf2::TimePointZero);
        imu_rig_from_imu = transformToCuvslamPose(t_imu);
      }
      auto t_rgb = tf_buffer_.lookupTransform(
        rig_frame_id_, rgb_info_->header.frame_id, tf2::TimePointZero);
      rgb_rig_from_cam = transformToCuvslamPose(t_rgb);
    } catch (const tf2::TransformException & ex) {
      return;
    }

    cuvslam::Camera rgb_camera = buildCuvslamCamera(*rgb_info_, rgb_rig_from_cam, get_logger());
    /* wait we can just push it directly */

    cuvslam::Rig rig;
    rig.cameras.push_back(rgb_camera);

    if (use_imu_) {
      cuvslam::ImuCalibration imu;
      imu.rig_from_imu = imu_rig_from_imu;
      // You should add noise densities here too if needed, but for now we skip or add 0s
      rig.imus.push_back(imu);
    }

    auto config = cuvslam::Odometry::GetDefaultConfig();
    config.max_frame_delta_s = 0.5f;
    config.odometry_mode = cuvslam::Odometry::OdometryMode::RGBD;
    config.rgbd_settings.depth_camera_id = 0;
    config.rgbd_settings.depth_scale_factor = 0.001f;

    try {
      cuvslam::WarmUpGPU();
      tracker_ = std::make_unique<cuvslam::Odometry>(rig, config);
      RCLCPP_INFO(get_logger(), "cuVSLAM RGBD rig initialized!");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Tracker init failed: %s", e.what());
    }
  }

  void flushImuMeasurements(int64_t track_ts_ns)
  {
    if (!tracker_ || pending_imu_.empty()) {
      return;
    }

    if (last_track_ts_ns_ == 0) {
      while (!pending_imu_.empty() && pending_imu_.front().timestamp_ns < track_ts_ns) {
        pending_imu_.pop_front();
      }
      return;
    }

    while (!pending_imu_.empty() && pending_imu_.front().timestamp_ns < track_ts_ns) {
      const auto & m = pending_imu_.front();
      if (m.timestamp_ns > last_track_ts_ns_) {
        try {
          tracker_->RegisterImuMeasurement(0, m);
        } catch (const std::exception & e) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "ProvideImu failed: %s", e.what());
        }
      }
      pending_imu_.pop_front();
    }
  }

  void publishOdometry(const cuvslam::PoseWithCovariance & pose_with_covariance, const builtin_interfaces::msg::Time & stamp)
  {
    const cuvslam::Pose & world_from_rig = pose_with_covariance.pose;
    Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = odom_frame_id_;
    msg.child_frame_id = published_child_frame_id_;

    tf2::Matrix3x3 R_ros_cv(
      0,  0,  1,
     -1,  0,  0,
      0, -1,  0
    );
    tf2::Vector3 p_cv(
      world_from_rig.translation[0],
      world_from_rig.translation[1],
      world_from_rig.translation[2]);
    tf2::Vector3 p_ros = R_ros_cv * p_cv;

    tf2::Quaternion q_cv(
      world_from_rig.rotation[0],
      world_from_rig.rotation[1],
      world_from_rig.rotation[2],
      world_from_rig.rotation[3]);

    tf2::Matrix3x3 R_cv_quat(q_cv);
    tf2::Matrix3x3 R_ros = R_ros_cv * R_cv_quat;
    tf2::Quaternion q_ros;
    R_ros.getRotation(q_ros);

    if (!origin_latched_) {
      origin_q_inv_ = q_ros.inverse();
      origin_p_ = p_ros;
      origin_latched_ = true;
    }

    tf2::Vector3 aligned_p = tf2::quatRotate(origin_q_inv_, p_ros - origin_p_);
    tf2::Quaternion aligned_q = origin_q_inv_ * q_ros;

    msg.pose.pose.position.x = aligned_p.x();
    msg.pose.pose.position.y = aligned_p.y();
    msg.pose.pose.position.z = aligned_p.z();

    msg.pose.pose.orientation.x = aligned_q.x();
    msg.pose.pose.orientation.y = aligned_q.y();
    msg.pose.pose.orientation.z = aligned_q.z();
    msg.pose.pose.orientation.w = aligned_q.w();

    odom_pub_->publish(msg);
  }

  std::string rgb_image_topic_;
  std::string depth_image_topic_;
  std::string rgb_camera_info_topic_;
  std::string imu_topic_;
  std::string raw_odom_topic_;
  std::string odom_frame_id_;
  std::string rig_frame_id_;
  std::string imu_frame_id_param_;
  std::string published_child_frame_id_;

  bool use_imu_;
  int sync_queue_size_;
  double sync_slop_s_;
  double rig_init_timeout_s_;
  bool debug_;

  rclcpp::Time init_start_;
  std::optional<CameraInfo> rgb_info_;

  std::unique_ptr<cuvslam::Odometry> tracker_;

  rclcpp::Subscription<CameraInfo>::SharedPtr rgb_info_sub_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  message_filters::Subscriber<Image> rgb_image_sub_;
  message_filters::Subscriber<Image> depth_image_sub_;
  std::shared_ptr<RgbdSynchronizer> sync_;

  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;

  int64_t last_imu_received_ts_ns_{0};
  int64_t last_track_ts_ns_{0};
  std::deque<cuvslam::ImuMeasurement> pending_imu_;

  bool origin_latched_{false};
  tf2::Vector3 origin_p_;
  tf2::Quaternion origin_q_inv_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace cuvslam_bringup

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cuvslam_bringup::CuvslamRgbdNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
