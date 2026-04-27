// Copyright 2026 The Ackermann Rover Authors
// SPDX-License-Identifier: Apache-2.0

/**
 * cuvslam_odom_node.cpp
 *
 * Thin ROS 2 wrapper around cuvslam::Odometry for stereo visual-inertial
 * odometry using the Intel RealSense T265 fisheye stereo pair and IMU.
 *
 * Responsibilities:
 *   1. Initialize a cuvslam::Rig from the live CameraInfo topics and the
 *      static TF tree.
 *   2. Synchronize stereo fisheye images with message_filters ApproximateTime.
 *   3. Buffer IMU measurements and forward them to cuVSLAM in strict
 *      timestamp order between Track() calls.
 *   4. Publish raw odometry (nav_msgs/Odometry) suitable for consumption by
 *      odom_tf_relay -> EKF -> RTAB-Map / Nav2 / PX4.
 *
 * Frame conventions:
 *   - In the normal T265 path the rig frame is "t265_pose_frame", matching
 *     the raw built-in T265 odometry contract.
 *   - odom_tf_relay adapts that raw sensor-frame odometry into
 *     ackermann/base_link, latches the origin, and optionally publishes the
 *     final odom -> ackermann/base_link TF edge.
 *   - If TF lookup for rig extrinsics times out, the node falls back to
 *     rig = left camera optical frame. In that fallback mode the published
 *     child_frame_id becomes the left fisheye optical frame so odom_tf_relay
 *     can still compose the pose into ackermann/base_link via the static TF
 *     tree.
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

using StereoSyncPolicy =
  message_filters::sync_policies::ApproximateTime<Image, Image>;
using StereoSynchronizer = message_filters::Synchronizer<StereoSyncPolicy>;

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
  if (model == "equidistant" && info.d.size() >= 4) {
    cam.distortion.model = cuvslam::Distortion::Model::Fisheye;
    cam.distortion.parameters = {
      static_cast<float>(info.d[0]),
      static_cast<float>(info.d[1]),
      static_cast<float>(info.d[2]),
      static_cast<float>(info.d[3]),
    };
    return true;
  }

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

  if (model == "rational_polynomial" && info.d.size() >= 8) {
    cam.distortion.model = cuvslam::Distortion::Model::Polynomial;
    cam.distortion.parameters.clear();
    cam.distortion.parameters.reserve(8);
    for (size_t i = 0; i < 8; ++i) {
      cam.distortion.parameters.push_back(static_cast<float>(info.d[i]));
    }
    return true;
  }

  if (model.empty() || model == "none") {
    cam.distortion.model = cuvslam::Distortion::Model::Pinhole;
    cam.distortion.parameters.clear();
    return true;
  }

  RCLCPP_WARN(
    logger,
    "Unknown CameraInfo distortion_model '%s' (d.size()=%zu); falling back to Pinhole",
    model.c_str(), info.d.size());
  cam.distortion.model = cuvslam::Distortion::Model::Pinhole;
  cam.distortion.parameters.clear();
  return false;
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

class CuvslamOdomNode : public rclcpp::Node
{
public:
  CuvslamOdomNode()
  : Node("cuvslam_odom_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    left_image_topic_ = declare_parameter<std::string>(
      "left_image_topic", "/t265/fisheye1/image_raw");
    right_image_topic_ = declare_parameter<std::string>(
      "right_image_topic", "/t265/fisheye2/image_raw");
    left_camera_info_topic_ = declare_parameter<std::string>(
      "left_camera_info_topic", "/t265/fisheye1/camera_info");
    right_camera_info_topic_ = declare_parameter<std::string>(
      "right_camera_info_topic", "/t265/fisheye2/camera_info");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/t265/imu");
    raw_odom_topic_ = declare_parameter<std::string>(
      "raw_odom_topic", "/cuvslam/raw_odometry");

    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    rig_frame_id_ = declare_parameter<std::string>(
      "rig_frame_id", "t265_pose_frame");
    imu_frame_id_param_ = declare_parameter<std::string>("imu_frame_id", "");

    use_imu_ = declare_parameter<bool>("use_imu", true);
    enable_observations_ = declare_parameter<bool>("enable_observations", false);
    enable_landmarks_ = declare_parameter<bool>("enable_landmarks", false);
    async_sba_ = declare_parameter<bool>("async_sba", true);
    use_motion_model_ = declare_parameter<bool>("use_motion_model", true);
    use_denoising_ = declare_parameter<bool>("use_denoising", false);
    max_frame_delta_s_ = declare_parameter<double>("max_frame_delta_s", 0.5);

    sync_queue_size_ = declare_parameter<int>("sync_queue_size", 10);
    sync_slop_s_ = declare_parameter<double>("sync_slop_s", 0.01);
    rig_init_timeout_s_ = declare_parameter<double>("rig_init_timeout_s", 10.0);

    debug_ = declare_parameter<bool>("debug", false);

    imu_gyro_noise_density_ = declare_parameter<double>(
      "imu_gyroscope_noise_density", 1.7e-4);
    imu_gyro_random_walk_ = declare_parameter<double>(
      "imu_gyroscope_random_walk", 2.0e-5);
    imu_accel_noise_density_ = declare_parameter<double>(
      "imu_accelerometer_noise_density", 2.0e-3);
    imu_accel_random_walk_ = declare_parameter<double>(
      "imu_accelerometer_random_walk", 3.0e-3);
    imu_frequency_hz_ = declare_parameter<double>("imu_frequency_hz", 200.0);
    fallback_baseline_m_ = declare_parameter<double>(
      "fallback_stereo_baseline_m", 0.0642);

    published_child_frame_id_ = rig_frame_id_;

    cuvslam::SetVerbosity(debug_ ? 1 : 0);

    const auto version = std::string(cuvslam::GetVersion(nullptr, nullptr, nullptr));
    RCLCPP_INFO(
      get_logger(),
      "cuvslam_odom_node starting - cuVSLAM version %s",
      version.c_str());

    const auto sensor_qos = rclcpp::SensorDataQoS();
    const auto info_qos = rclcpp::QoS(10).reliable();

    left_info_sub_ = create_subscription<CameraInfo>(
      left_camera_info_topic_, info_qos,
      std::bind(&CuvslamOdomNode::leftInfoCallback, this, std::placeholders::_1));
    right_info_sub_ = create_subscription<CameraInfo>(
      right_camera_info_topic_, info_qos,
      std::bind(&CuvslamOdomNode::rightInfoCallback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<Imu>(
      imu_topic_, sensor_qos,
      std::bind(&CuvslamOdomNode::imuCallback, this, std::placeholders::_1));

    left_image_sub_.subscribe(this, left_image_topic_, sensor_qos.get_rmw_qos_profile());
    right_image_sub_.subscribe(this, right_image_topic_, sensor_qos.get_rmw_qos_profile());

    sync_ = std::make_shared<StereoSynchronizer>(
      StereoSyncPolicy(sync_queue_size_), left_image_sub_, right_image_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_s_));
    sync_->registerCallback(
      std::bind(&CuvslamOdomNode::stereoCallback, this, std::placeholders::_1, std::placeholders::_2));

    odom_pub_ = create_publisher<Odometry>(raw_odom_topic_, rclcpp::QoS(10));

    init_start_ = now();
    RCLCPP_INFO(
      get_logger(),
      "Subscribed:\n"
      "  left  image    : %s\n"
      "  right image    : %s\n"
      "  left  info     : %s\n"
      "  right info     : %s\n"
      "  imu            : %s\n"
      "Publishing raw odometry on: %s\n"
      "Rig frame (target)    : %s\n"
      "Odom frame_id         : %s",
      left_image_topic_.c_str(),
      right_image_topic_.c_str(),
      left_camera_info_topic_.c_str(),
      right_camera_info_topic_.c_str(),
      imu_topic_.c_str(),
      raw_odom_topic_.c_str(),
      rig_frame_id_.c_str(),
      odom_frame_id_.c_str());
  }

private:
  void leftInfoCallback(const CameraInfo::SharedPtr msg)
  {
    left_info_ = *msg;
    if (!left_info_logged_) {
      RCLCPP_INFO(
        get_logger(),
        "Received left CameraInfo (%ux%u, frame_id=%s, distortion=%s)",
        msg->width, msg->height, msg->header.frame_id.c_str(),
        msg->distortion_model.c_str());
      left_info_logged_ = true;
    }
    tryInitRig();
  }

  void rightInfoCallback(const CameraInfo::SharedPtr msg)
  {
    right_info_ = *msg;
    if (!right_info_logged_) {
      RCLCPP_INFO(
        get_logger(),
        "Received right CameraInfo (%ux%u, frame_id=%s, distortion=%s)",
        msg->width, msg->height, msg->header.frame_id.c_str(),
        msg->distortion_model.c_str());
      right_info_logged_ = true;
    }
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

  void stereoCallback(const Image::ConstSharedPtr & left_msg, const Image::ConstSharedPtr & right_msg)
  {
    if (!tracker_) {
      dropped_pre_init_++;
      if (dropped_pre_init_ % 30 == 1) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping stereo frame: cuVSLAM rig not initialized yet (waiting on CameraInfo + TF)");
      }
      return;
    }

    const int64_t left_ts_ns = stampToNs(left_msg->header.stamp);
    const int64_t right_ts_ns = stampToNs(right_msg->header.stamp);
    if (std::llabs(left_ts_ns - right_ts_ns) > 1000000LL) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Stereo timestamps differ by %.3f ms; cuVSLAM expects nearly synchronized images",
        static_cast<double>(std::llabs(left_ts_ns - right_ts_ns)) / 1.0e6);
    }

    // cuVSLAM requires strictly ascending frame timestamps across Track()
    // calls. On real hardware the upstream driver can occasionally re-emit
    // the same stereo pair (e.g. the librealsense pipeline sync module
    // delivering a frameset alongside a fresh IMU/pose sample), which would
    // otherwise trip an "Frame timestamps must be strictly increasing"
    // exception inside Track(). Drop any non-monotonic pair before it
    // reaches the tracker as a defensive fallback; the upstream camera node
    // should already deduplicate.
    if (last_track_ts_ns_ != 0 && left_ts_ns <= last_track_ts_ns_) {
      dropped_non_monotonic_++;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Dropped non-monotonic stereo pair (total dropped: %zu)",
        dropped_non_monotonic_);
      return;
    }

    cv_bridge::CvImageConstPtr left_cv;
    cv_bridge::CvImageConstPtr right_cv;
    try {
      left_cv = cv_bridge::toCvShare(left_msg, sensor_msgs::image_encodings::MONO8);
      right_cv = cv_bridge::toCvShare(right_msg, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "cv_bridge conversion failed: %s", e.what());
      return;
    }

    if (use_imu_) {
      flushImuMeasurements(left_ts_ns);
    }

    cuvslam::Image left_image;
    left_image.pixels = left_cv->image.data;
    left_image.width = left_cv->image.cols;
    left_image.height = left_cv->image.rows;
    left_image.pitch = static_cast<int32_t>(left_cv->image.step);
    left_image.encoding = cuvslam::ImageData::Encoding::MONO;
    left_image.data_type = cuvslam::ImageData::DataType::UINT8;
    left_image.is_gpu_mem = false;
    left_image.timestamp_ns = left_ts_ns;
    left_image.camera_index = 0;

    cuvslam::Image right_image = left_image;
    right_image.pixels = right_cv->image.data;
    right_image.width = right_cv->image.cols;
    right_image.height = right_cv->image.rows;
    right_image.pitch = static_cast<int32_t>(right_cv->image.step);
    right_image.timestamp_ns = right_ts_ns;
    right_image.camera_index = 1;

    cuvslam::Odometry::ImageSet images = {left_image, right_image};

    cuvslam::PoseEstimate estimate;
    try {
      estimate = tracker_->Track(images);
      last_track_ts_ns_ = left_ts_ns;
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "cuVSLAM Track() failed: %s", e.what());
      return;
    }

    if (!estimate.world_from_rig.has_value()) {
      tracking_lost_count_++;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "cuVSLAM tracking lost (total lost frames: %zu)",
        tracking_lost_count_);
      return;
    }

    publishOdometry(*estimate.world_from_rig, left_msg->header.stamp);
  }

  void tryInitRig()
  {
    if (tracker_ || !left_info_.has_value() || !right_info_.has_value()) {
      return;
    }

    cuvslam::Pose left_rig_from_cam;
    cuvslam::Pose right_rig_from_cam;
    cuvslam::Pose imu_rig_from_imu;
    const bool tf_ok = lookupExtrinsicsFromTf(
      left_rig_from_cam, right_rig_from_cam, imu_rig_from_imu);

    if (!tf_ok) {
      const double waited = (now() - init_start_).seconds();
      if (waited < rig_init_timeout_s_) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for static TF between '%s' and the fisheye/IMU frames (waited %.1fs / %.1fs timeout)",
          rig_frame_id_.c_str(), waited, rig_init_timeout_s_);
        return;
      }

      RCLCPP_WARN(
        get_logger(),
        "TF lookup for cuVSLAM rig extrinsics timed out after %.1fs; falling back to rig = left camera optical frame with %.4f m baseline",
        waited, fallback_baseline_m_);
      fillFallbackExtrinsics(left_rig_from_cam, right_rig_from_cam, imu_rig_from_imu);
      published_child_frame_id_ = left_info_->header.frame_id;
    } else {
      published_child_frame_id_ = rig_frame_id_;
    }

    cuvslam::Rig rig;
    rig.cameras.push_back(
      buildCuvslamCamera(*left_info_, left_rig_from_cam, get_logger()));
    rig.cameras.push_back(
      buildCuvslamCamera(*right_info_, right_rig_from_cam, get_logger()));

    if (use_imu_) {
      cuvslam::ImuCalibration imu;
      imu.rig_from_imu = imu_rig_from_imu;
      imu.gyroscope_noise_density = static_cast<float>(imu_gyro_noise_density_);
      imu.gyroscope_random_walk = static_cast<float>(imu_gyro_random_walk_);
      imu.accelerometer_noise_density = static_cast<float>(imu_accel_noise_density_);
      imu.accelerometer_random_walk = static_cast<float>(imu_accel_random_walk_);
      imu.frequency = static_cast<float>(imu_frequency_hz_);
      rig.imus.push_back(imu);
    }

    auto cfg = cuvslam::Odometry::GetDefaultConfig();
    cfg.odometry_mode = use_imu_ ?
      cuvslam::Odometry::OdometryMode::Inertial :
      cuvslam::Odometry::OdometryMode::Multicamera;
    cfg.multicam_mode = cuvslam::Odometry::MulticameraMode::Precision;
    cfg.use_gpu = true;
    cfg.async_sba = async_sba_;
    cfg.use_motion_model = use_motion_model_;
    cfg.use_denoising = use_denoising_;
    cfg.enable_observations_export = enable_observations_;
    cfg.enable_landmarks_export = enable_landmarks_;
    cfg.max_frame_delta_s = static_cast<float>(max_frame_delta_s_);

    try {
      cuvslam::WarmUpGPU();
      tracker_ = std::make_unique<cuvslam::Odometry>(rig, cfg);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_logger(),
        "cuVSLAM Odometry construction failed: %s (will retry on next CameraInfo)",
        e.what());
      tracker_.reset();
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "cuVSLAM rig initialized: 2 cameras%s (rig frame='%s', child_frame_id='%s', mode=%s)",
      use_imu_ ? " + IMU" : "",
      rig_frame_id_.c_str(),
      published_child_frame_id_.c_str(),
      use_imu_ ? "Inertial" : "Multicamera");
  }

  bool lookupExtrinsicsFromTf(
    cuvslam::Pose & left_pose,
    cuvslam::Pose & right_pose,
    cuvslam::Pose & imu_pose)
  {
    const std::string & left_frame = left_info_->header.frame_id;
    const std::string & right_frame = right_info_->header.frame_id;
    if (left_frame.empty() || right_frame.empty()) {
      return false;
    }

    try {
      const auto left_tf = tf_buffer_.lookupTransform(
        rig_frame_id_, left_frame, tf2::TimePointZero, tf2::durationFromSec(0.0));
      const auto right_tf = tf_buffer_.lookupTransform(
        rig_frame_id_, right_frame, tf2::TimePointZero, tf2::durationFromSec(0.0));
      left_pose = transformToCuvslamPose(left_tf);
      right_pose = transformToCuvslamPose(right_tf);
    } catch (const tf2::TransformException &) {
      return false;
    }

    if (!use_imu_) {
      return true;
    }

    std::string imu_frame = imu_frame_id_param_;
    if (imu_frame.empty()) {
      imu_frame = left_frame;
      const auto pos = imu_frame.find("fisheye1");
      if (pos != std::string::npos) {
        imu_frame.replace(pos, 8, "imu");
      }
    }

    try {
      const auto imu_tf = tf_buffer_.lookupTransform(
        rig_frame_id_, imu_frame, tf2::TimePointZero, tf2::durationFromSec(0.0));
      imu_pose = transformToCuvslamPose(imu_tf);
    } catch (const tf2::TransformException &) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU frame TF lookup failed (rig='%s', imu='%s')",
        rig_frame_id_.c_str(), imu_frame.c_str());
      return false;
    }

    return true;
  }

  void fillFallbackExtrinsics(
    cuvslam::Pose & left_pose,
    cuvslam::Pose & right_pose,
    cuvslam::Pose & imu_pose)
  {
    left_pose = cuvslam::Pose{};
    right_pose = cuvslam::Pose{};
    right_pose.translation = {
      static_cast<float>(fallback_baseline_m_),
      0.0f,
      0.0f,
    };
    imu_pose = cuvslam::Pose{};
  }

  void flushImuMeasurements(int64_t frame_ts_ns)
  {
    if (!tracker_ || pending_imu_.empty()) {
      return;
    }

    if (last_track_ts_ns_ == 0) {
      while (!pending_imu_.empty() && pending_imu_.front().timestamp_ns < frame_ts_ns) {
        pending_imu_.pop_front();
      }
      return;
    }

    while (!pending_imu_.empty() && pending_imu_.front().timestamp_ns < frame_ts_ns) {
      const auto meas = pending_imu_.front();
      pending_imu_.pop_front();
      if (meas.timestamp_ns <= last_track_ts_ns_) {
        continue;
      }

      try {
        tracker_->RegisterImuMeasurement(0, meas);
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "cuVSLAM RegisterImuMeasurement failed: %s", e.what());
        return;
      }
    }
  }

  void publishOdometry(
    const cuvslam::PoseWithCovariance & pose_with_covariance,
    const builtin_interfaces::msg::Time & stamp)
  {
    Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = published_child_frame_id_;

    tf2::Matrix3x3 R_ros_cv(
      0,  0,  1,
     -1,  0,  0,
      0, -1,  0
    );

    tf2::Vector3 pos_cv(
      pose_with_covariance.pose.translation[0],
      pose_with_covariance.pose.translation[1],
      pose_with_covariance.pose.translation[2]
    );

    tf2::Vector3 pos_ros = R_ros_cv * pos_cv;

    tf2::Quaternion q_cv(
      pose_with_covariance.pose.rotation[0],
      pose_with_covariance.pose.rotation[1],
      pose_with_covariance.pose.rotation[2],
      pose_with_covariance.pose.rotation[3]
    );

    tf2::Quaternion q_ros_cv;
    R_ros_cv.getRotation(q_ros_cv);

    tf2::Quaternion q_ros = q_ros_cv * q_cv;
    q_ros.normalize();

    tf2::Transform current_tf(q_ros, pos_ros);

    if (!origin_latched_) {
      origin_inv_ = current_tf.inverse();
      origin_latched_ = true;
    }

    tf2::Transform aligned_tf = origin_inv_ * current_tf;

    odom.pose.pose.position.x = aligned_tf.getOrigin().x();
    odom.pose.pose.position.y = aligned_tf.getOrigin().y();
    odom.pose.pose.position.z = aligned_tf.getOrigin().z();

    odom.pose.pose.orientation.x = aligned_tf.getRotation().x();
    odom.pose.pose.orientation.y = aligned_tf.getRotation().y();
    odom.pose.pose.orientation.z = aligned_tf.getRotation().z();
    odom.pose.pose.orientation.w = aligned_tf.getRotation().w();

    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        const int src_i = (i < 3) ? i + 3 : i - 3;
        const int src_j = (j < 3) ? j + 3 : j - 3;
        odom.pose.covariance[i * 6 + j] =
          pose_with_covariance.covariance[src_i * 6 + src_j];
      }
    }

    for (int i = 0; i < 6; ++i) {
      odom.twist.covariance[i * 6 + i] = -1.0;
    }

    odom_pub_->publish(odom);
    ++published_frames_;
  }

  std::string left_image_topic_;
  std::string right_image_topic_;
  std::string left_camera_info_topic_;
  std::string right_camera_info_topic_;
  std::string imu_topic_;
  std::string raw_odom_topic_;
  std::string odom_frame_id_;
  std::string rig_frame_id_;
  std::string imu_frame_id_param_;
  std::string published_child_frame_id_;

  bool use_imu_{true};
  bool enable_observations_{false};
  bool enable_landmarks_{false};
  bool async_sba_{true};
  bool use_motion_model_{true};
  bool use_denoising_{false};
  bool debug_{false};

  int sync_queue_size_{10};
  double sync_slop_s_{0.01};
  double rig_init_timeout_s_{10.0};
  double max_frame_delta_s_{0.5};

  double imu_gyro_noise_density_{0.0};
  double imu_gyro_random_walk_{0.0};
  double imu_accel_noise_density_{0.0};
  double imu_accel_random_walk_{0.0};
  double imu_frequency_hz_{200.0};
  double fallback_baseline_m_{0.0642};

  rclcpp::Time init_start_;
  std::optional<CameraInfo> left_info_;
  std::optional<CameraInfo> right_info_;
  bool left_info_logged_{false};
  bool right_info_logged_{false};
  std::unique_ptr<cuvslam::Odometry> tracker_;
  std::deque<cuvslam::ImuMeasurement> pending_imu_;
  int64_t last_imu_received_ts_ns_{0};
  int64_t last_track_ts_ns_{0};
  size_t dropped_pre_init_{0};
  size_t dropped_non_monotonic_{0};
  size_t tracking_lost_count_{0};
  size_t published_frames_{0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<CameraInfo>::SharedPtr left_info_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr right_info_sub_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  message_filters::Subscriber<Image> left_image_sub_;
  message_filters::Subscriber<Image> right_image_sub_;
  std::shared_ptr<StereoSynchronizer> sync_;
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;

  bool origin_latched_{false};
  tf2::Transform origin_inv_;
};

}  // namespace cuvslam_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cuvslam_bringup::CuvslamOdomNode>());
  rclcpp::shutdown();
  return 0;
}
