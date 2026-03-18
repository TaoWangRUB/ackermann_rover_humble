#include "realsense_camera_bingup/realsense_camera_node.hpp"

#include <sensor_msgs/image_encodings.hpp>

namespace realsense_camera_bingup
{

// ============================================================================
// Construction / destruction
// ============================================================================

RealsenseCameraNode::RealsenseCameraNode(const rclcpp::NodeOptions & options)
: Node("realsense_camera_node", options)
{
  declare_parameters();
  create_publishers();
  start_pipeline();
}

RealsenseCameraNode::~RealsenseCameraNode()
{
  running_ = false;
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  try { pipe_.stop(); } catch (...) {}
}

// ============================================================================
// Parameter declaration
// ============================================================================

void RealsenseCameraNode::declare_parameters()
{
  camera_name_ = declare_parameter<std::string>("camera_name", "camera");
  serial_no_   = declare_parameter<std::string>("serial_no", "");

  const std::string model_str = declare_parameter<std::string>("camera_model", "d435i");
  if (model_str == "l515") {
    camera_model_ = CameraModel::L515;
  } else if (model_str == "t265") {
    camera_model_ = CameraModel::T265;
  } else {
    camera_model_ = CameraModel::D435I;
    if (model_str != "d435i") {
      RCLCPP_WARN(get_logger(),
        "Unknown camera_model '%s', defaulting to d435i", model_str.c_str());
    }
  }

  // D435i / L515 parameters
  enable_color_         = declare_parameter<bool>("enable_color", true);
  enable_depth_         = declare_parameter<bool>("enable_depth", true);
  enable_imu_           = declare_parameter<bool>("enable_imu", false);
  color_width_          = declare_parameter<int>("color_width", 640);
  color_height_         = declare_parameter<int>("color_height", 480);
  color_fps_            = declare_parameter<int>("color_fps", 30);
  depth_width_          = declare_parameter<int>("depth_width", 640);
  depth_height_         = declare_parameter<int>("depth_height", 480);
  depth_fps_            = declare_parameter<int>("depth_fps", 30);
  align_depth_to_color_ = declare_parameter<bool>("align_depth_to_color", false);

  // T265 parameters
  fisheye_fps_ = declare_parameter<int>("fisheye_fps", 30);

  // T265 has no color/depth streams
  if (camera_model_ == CameraModel::T265) {
    enable_color_ = false;
    enable_depth_ = false;
    enable_imu_   = true;  // always needed for tracking
  }
}

// ============================================================================
// Publisher creation
// ============================================================================

void RealsenseCameraNode::create_publishers()
{
  const auto qos = rclcpp::SensorDataQoS();

  if (camera_model_ == CameraModel::T265) {
    fisheye1_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      camera_name_ + "/fisheye1/image_raw", qos);
    fisheye1_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      camera_name_ + "/fisheye1/camera_info", qos);
    fisheye2_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      camera_name_ + "/fisheye2/image_raw", qos);
    fisheye2_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      camera_name_ + "/fisheye2/camera_info", qos);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      camera_name_ + "/odom", qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      camera_name_ + "/imu", qos);
    return;
  }

  // D435i / L515
  if (enable_color_) {
    color_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      camera_name_ + "/color/image_raw", qos);
    color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      camera_name_ + "/color/camera_info", qos);
  }
  if (enable_depth_) {
    depth_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      camera_name_ + "/depth/image_rect_raw", qos);
    depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      camera_name_ + "/depth/camera_info", qos);
  }
  if (enable_imu_) {
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      camera_name_ + "/imu", qos);
  }
}

// ============================================================================
// Pipeline start
// ============================================================================

void RealsenseCameraNode::start_pipeline()
{
  rs2::config cfg;
  if (!serial_no_.empty()) {
    cfg.enable_device(serial_no_);
  }

  if (camera_model_ == CameraModel::T265) {
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO,  RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_POSE,  RS2_FORMAT_6DOF);
  } else {
    // D435i / L515
    if (enable_color_) {
      cfg.enable_stream(
        RS2_STREAM_COLOR, color_width_, color_height_, RS2_FORMAT_BGR8, color_fps_);
    }
    if (enable_depth_) {
      cfg.enable_stream(
        RS2_STREAM_DEPTH, depth_width_, depth_height_, RS2_FORMAT_Z16, depth_fps_);
    }
    if (enable_imu_) {
      cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
      cfg.enable_stream(RS2_STREAM_GYRO,  RS2_FORMAT_MOTION_XYZ32F);
    }
  }

  rs2::pipeline_profile profile = pipe_.start(cfg);

  // Cache camera_info from intrinsics (done once at startup)
  if (camera_model_ == CameraModel::T265) {
    auto fe1 = profile.get_stream(RS2_STREAM_FISHEYE, 1).as<rs2::video_stream_profile>();
    auto fe2 = profile.get_stream(RS2_STREAM_FISHEYE, 2).as<rs2::video_stream_profile>();
    fisheye1_info_msg_ = build_camera_info(fe1, camera_name_ + "_fisheye1_optical_frame");
    fisheye2_info_msg_ = build_camera_info(fe2, camera_name_ + "_fisheye2_optical_frame");
  } else {
    if (enable_color_) {
      auto vsp = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
      color_info_msg_ = build_camera_info(vsp, camera_name_ + "_color_optical_frame");
    }
    if (enable_depth_) {
      auto vsp = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
      depth_info_msg_ = build_camera_info(vsp, camera_name_ + "_depth_optical_frame");
    }
  }

  RCLCPP_INFO(get_logger(),
    "RealSense pipeline started — model=%s serial=%s",
    (camera_model_ == CameraModel::T265 ? "t265" :
     camera_model_ == CameraModel::L515 ? "l515" : "d435i"),
    serial_no_.empty() ? "(auto)" : serial_no_.c_str());

  running_ = true;
  capture_thread_ = std::thread(&RealsenseCameraNode::capture_loop, this);
}

// ============================================================================
// Capture loop
// ============================================================================

void RealsenseCameraNode::capture_loop()
{
  while (running_ && rclcpp::ok()) {
    try {
      rs2::frameset frames = pipe_.wait_for_frames(5000 /*ms*/);
      if (camera_model_ == CameraModel::T265) {
        process_t265_frames(frames);
      } else {
        process_depth_camera_frames(frames);
      }
    } catch (const rs2::error & e) {
      if (running_) {
        RCLCPP_WARN(get_logger(), "RealSense error: %s", e.what());
      }
    } catch (const std::exception & e) {
      if (running_) {
        RCLCPP_WARN(get_logger(), "Capture error: %s", e.what());
      }
    }
  }
}

// ============================================================================
// Per-model frame processing
// ============================================================================

void RealsenseCameraNode::process_depth_camera_frames(const rs2::frameset & frames_in)
{
  rs2::frameset frames = frames_in;

  if (align_depth_to_color_ && enable_depth_ && enable_color_) {
    frames = align_to_color_.process(frames);
  }

  if (enable_color_) {
    if (auto f = frames.get_color_frame()) {
      publish_color_frame(f);
    }
  }
  if (enable_depth_) {
    if (auto f = frames.get_depth_frame()) {
      publish_depth_frame(f);
    }
  }
  if (enable_imu_) {
    // Motion frames arrive in separate framesets; accumulate accel and publish on gyro
    static rs2::motion_frame last_accel;
    if (auto f = frames.first_or_default(RS2_STREAM_ACCEL)) {
      last_accel = f.as<rs2::motion_frame>();
    }
    if (auto f = frames.first_or_default(RS2_STREAM_GYRO)) {
      if (last_accel) {
        publish_imu_frame(last_accel, f.as<rs2::motion_frame>());
      }
    }
  }
}

void RealsenseCameraNode::process_t265_frames(const rs2::frameset & frames)
{
  // Pose (odometry) — arrives at ~200 Hz
  if (auto f = frames.first_or_default(RS2_STREAM_POSE)) {
    publish_pose_frame(f.as<rs2::pose_frame>());
  }

  // Fisheye images — arrive at fisheye_fps_ (default 30 Hz)
  if (auto f = frames.get_fisheye_frame(1)) {
    publish_fisheye_frame(f, 1, fisheye1_info_msg_);
  }
  if (auto f = frames.get_fisheye_frame(2)) {
    publish_fisheye_frame(f, 2, fisheye2_info_msg_);
  }

  // IMU — accel ~62.5 Hz, gyro ~200 Hz
  static rs2::motion_frame last_accel;
  if (auto f = frames.first_or_default(RS2_STREAM_ACCEL)) {
    last_accel = f.as<rs2::motion_frame>();
  }
  if (auto f = frames.first_or_default(RS2_STREAM_GYRO)) {
    if (last_accel) {
      publish_imu_frame(last_accel, f.as<rs2::motion_frame>());
    }
  }
}

// ============================================================================
// camera_info builder
// ============================================================================

sensor_msgs::msg::CameraInfo RealsenseCameraNode::build_camera_info(
  const rs2::video_stream_profile & profile,
  const std::string & frame_id) const
{
  rs2_intrinsics intr = profile.get_intrinsics();

  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = frame_id;
  info.width  = static_cast<uint32_t>(intr.width);
  info.height = static_cast<uint32_t>(intr.height);

  // T265 fisheye uses Kannala-Brandt (4 coefficients); all others use plumb_bob (5)
  if (intr.model == RS2_DISTORTION_KANNALA_BRANDT4) {
    info.distortion_model = "equidistant";
    info.d = {intr.coeffs[0], intr.coeffs[1], intr.coeffs[2], intr.coeffs[3]};
  } else {
    info.distortion_model = "plumb_bob";
    info.d = {
      intr.coeffs[0], intr.coeffs[1],
      intr.coeffs[2], intr.coeffs[3],
      intr.coeffs[4]
    };
  }

  info.k = {
    intr.fx,  0.0,      intr.ppx,
    0.0,      intr.fy,  intr.ppy,
    0.0,      0.0,      1.0
  };
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {
    intr.fx,  0.0,      intr.ppx,  0.0,
    0.0,      intr.fy,  intr.ppy,  0.0,
    0.0,      0.0,      1.0,       0.0
  };

  return info;
}

// ============================================================================
// Publish helpers
// ============================================================================

void RealsenseCameraNode::publish_color_frame(const rs2::video_frame & frame)
{
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header.stamp    = now();
  msg->header.frame_id = camera_name_ + "_color_optical_frame";
  msg->height   = static_cast<uint32_t>(frame.get_height());
  msg->width    = static_cast<uint32_t>(frame.get_width());
  msg->encoding = sensor_msgs::image_encodings::BGR8;
  msg->step     = static_cast<uint32_t>(frame.get_stride_in_bytes());
  msg->is_bigendian = false;
  const auto * data = static_cast<const uint8_t *>(frame.get_data());
  msg->data.assign(data, data + msg->step * msg->height);

  auto info = color_info_msg_;
  info.header = msg->header;
  color_image_pub_->publish(std::move(msg));
  color_info_pub_->publish(info);
}

void RealsenseCameraNode::publish_depth_frame(const rs2::depth_frame & frame)
{
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header.stamp    = now();
  msg->header.frame_id = camera_name_ + "_depth_optical_frame";
  msg->height   = static_cast<uint32_t>(frame.get_height());
  msg->width    = static_cast<uint32_t>(frame.get_width());
  msg->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  msg->step     = static_cast<uint32_t>(frame.get_stride_in_bytes());
  msg->is_bigendian = false;
  const auto * data = static_cast<const uint8_t *>(frame.get_data());
  msg->data.assign(data, data + msg->step * msg->height);

  auto info = depth_info_msg_;
  info.header = msg->header;
  depth_image_pub_->publish(std::move(msg));
  depth_info_pub_->publish(info);
}

void RealsenseCameraNode::publish_fisheye_frame(
  const rs2::video_frame & frame, int index,
  const sensor_msgs::msg::CameraInfo & info_template)
{
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header.stamp    = now();
  msg->header.frame_id = camera_name_ + "_fisheye" + std::to_string(index) + "_optical_frame";
  msg->height   = static_cast<uint32_t>(frame.get_height());
  msg->width    = static_cast<uint32_t>(frame.get_width());
  msg->encoding = sensor_msgs::image_encodings::MONO8;
  msg->step     = static_cast<uint32_t>(frame.get_stride_in_bytes());
  msg->is_bigendian = false;
  const auto * data = static_cast<const uint8_t *>(frame.get_data());
  msg->data.assign(data, data + msg->step * msg->height);

  auto info = info_template;
  info.header = msg->header;

  if (index == 1) {
    fisheye1_image_pub_->publish(std::move(msg));
    fisheye1_info_pub_->publish(info);
  } else {
    fisheye2_image_pub_->publish(std::move(msg));
    fisheye2_info_pub_->publish(info);
  }
}

void RealsenseCameraNode::publish_pose_frame(const rs2::pose_frame & frame)
{
  rs2_pose pose = frame.get_pose_data();

  auto msg = std::make_unique<nav_msgs::msg::Odometry>();
  msg->header.stamp    = now();
  msg->header.frame_id = camera_name_ + "_odom_frame";
  msg->child_frame_id  = camera_name_ + "_pose_frame";

  msg->pose.pose.position.x = pose.translation.x;
  msg->pose.pose.position.y = pose.translation.y;
  msg->pose.pose.position.z = pose.translation.z;

  msg->pose.pose.orientation.x = pose.rotation.x;
  msg->pose.pose.orientation.y = pose.rotation.y;
  msg->pose.pose.orientation.z = pose.rotation.z;
  msg->pose.pose.orientation.w = pose.rotation.w;

  msg->twist.twist.linear.x  = pose.velocity.x;
  msg->twist.twist.linear.y  = pose.velocity.y;
  msg->twist.twist.linear.z  = pose.velocity.z;

  msg->twist.twist.angular.x = pose.angular_velocity.x;
  msg->twist.twist.angular.y = pose.angular_velocity.y;
  msg->twist.twist.angular.z = pose.angular_velocity.z;

  odom_pub_->publish(std::move(msg));
}

void RealsenseCameraNode::publish_imu_frame(
  const rs2::motion_frame & accel, const rs2::motion_frame & gyro)
{
  if (!accel || !gyro) {return;}

  auto a = accel.get_motion_data();
  auto g = gyro.get_motion_data();

  auto msg = std::make_unique<sensor_msgs::msg::Imu>();
  msg->header.stamp    = now();
  msg->header.frame_id = camera_name_ + "_imu_optical_frame";

  msg->angular_velocity.x = g.x;
  msg->angular_velocity.y = g.y;
  msg->angular_velocity.z = g.z;

  msg->linear_acceleration.x = a.x;
  msg->linear_acceleration.y = a.y;
  msg->linear_acceleration.z = a.z;

  // Covariance unknown
  msg->orientation_covariance[0]         = -1.0;
  msg->angular_velocity_covariance[0]    = -1.0;
  msg->linear_acceleration_covariance[0] = -1.0;

  imu_pub_->publish(std::move(msg));
}

}  // namespace realsense_camera_bingup

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<realsense_camera_bingup::RealsenseCameraNode>());
  rclcpp::shutdown();
  return 0;
}
