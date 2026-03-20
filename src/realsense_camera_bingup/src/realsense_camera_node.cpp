#include "realsense_camera_bingup/realsense_camera_node.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <chrono>
#include <thread>

namespace realsense_camera_bingup
{

RealsenseCameraNode::RealsenseCameraNode(const rclcpp::NodeOptions & options)
: Node("realsense_camera_node", options), pipe_(ctx_)
{
  declare_parameters();
  create_publishers();
  try {
    reset_device();
    start_pipeline();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Failed to initialise camera: %s", e.what());
    throw;
  }
}

RealsenseCameraNode::~RealsenseCameraNode()
{
  RCLCPP_INFO(get_logger(), "Shutting down — stopping pipeline...");
  running_ = false;

  // Watchdog: if shutdown blocks for more than 5 s, force-exit the process.
  std::thread watchdog([]{
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::_Exit(0);
  });
  watchdog.detach();

  // Stop IMU sensor first (independent of pipeline; avoids callback races).
  if (imu_sensor_) {
    try { imu_sensor_.stop(); imu_sensor_.close(); } catch (...) {}
  }
  // Then stop the video pipeline.
  try { pipe_.stop(); } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Pipeline stop error: %s", e.what());
  }
  RCLCPP_INFO(get_logger(), "Shutdown complete.");
}

void RealsenseCameraNode::declare_parameters()
{
  camera_name_ = declare_parameter<std::string>("camera_name", "camera");
  serial_no_   = declare_parameter<std::string>("serial_no", "");
  const std::string model_str = declare_parameter<std::string>("camera_model", "d435i");
  
  if (model_str == "l515") camera_model_ = CameraModel::L515;
  else if (model_str == "t265") camera_model_ = CameraModel::T265;
  else camera_model_ = CameraModel::D435I;

  enable_color_ = declare_parameter<bool>("enable_color", true);
  enable_depth_ = declare_parameter<bool>("enable_depth", true);
  enable_imu_   = declare_parameter<bool>("enable_imu", false);
  unite_imu_method_ = declare_parameter<int>("unite_imu_method", 2); // Recommended for SLAM

  color_width_  = declare_parameter<int>("color_width",  640);
  color_height_ = declare_parameter<int>("color_height", 480);
  color_fps_    = declare_parameter<int>("color_fps",    30);
  depth_width_  = declare_parameter<int>("depth_width",  640);
  depth_height_ = declare_parameter<int>("depth_height", 480);
  depth_fps_    = declare_parameter<int>("depth_fps",    30);

  // Declared for launch-file compatibility; not yet used in pipeline
  declare_parameter<bool>("align_depth_to_color", false);
  declare_parameter<int>("fisheye_fps", 30);
}

void RealsenseCameraNode::create_publishers()
{
  const auto qos = rclcpp::QoS(10);
  if (enable_color_) {
    color_image_pub_ = create_publisher<sensor_msgs::msg::Image>(camera_name_ + "/color/image_raw", qos);
    color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_ + "/color/camera_info", qos);
  }
  if (enable_depth_) {
    depth_image_pub_ = create_publisher<sensor_msgs::msg::Image>(camera_name_ + "/depth/image_rect_raw", qos);
    depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_ + "/depth/camera_info", qos);
  }
  if (enable_imu_) {
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(camera_name_ + "/imu", qos);
  }
  if (camera_model_ == CameraModel::T265) {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(camera_name_ + "/odom", qos);
  }
}

void RealsenseCameraNode::reset_device()
{
  RCLCPP_INFO(get_logger(), "Resetting RealSense device to clear stuck states...");

  try {
    // rs2::context enumerates USB devices asynchronously — give it time to complete
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    auto devices = ctx_.query_devices();
    if (devices.size() == 0) {
      RCLCPP_WARN(get_logger(), "No RealSense device found — skipping reset.");
      return;
    }

    std::string expected_model;
    switch (camera_model_) {
      case CameraModel::D435I: expected_model = "D435"; break;
      case CameraModel::L515:  expected_model = "L515"; break;
      case CameraModel::T265:  expected_model = "T265"; break;
    }

    rs2::device dev;
    for (auto d : devices) {
      try {
        const std::string serial = d.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        const std::string name   = d.get_info(RS2_CAMERA_INFO_NAME);
        if (!serial_no_.empty()) {
          if (serial == serial_no_) { dev = d; break; }
          continue;
        }
        if (name.find(expected_model) != std::string::npos) { dev = d; break; }
      } catch (const rs2::error &) { continue; }  // device in bad state — skip
    }

    if (!dev) {
      RCLCPP_WARN(get_logger(), "No RealSense device matching '%s' found — skipping reset.",
        serial_no_.empty() ? expected_model.c_str() : serial_no_.c_str());
      return;
    }

    const std::string reset_serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    RCLCPP_INFO(get_logger(), "Issuing hardware_reset() to device [%s]...", reset_serial.c_str());
    dev.hardware_reset();

    // Poll until THIS specific device reappears (not just any RealSense)
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      for (auto d : ctx_.query_devices()) {
        try {
          if (d.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == reset_serial) {
            RCLCPP_INFO(get_logger(), "Device USB reconnected — waiting for sensors to initialise...");
            std::this_thread::sleep_for(std::chrono::seconds(3));
            RCLCPP_INFO(get_logger(), "Device ready.");
            return;
          }
        } catch (const rs2::error &) { continue; }
      }
    }
    RCLCPP_WARN(get_logger(), "Device did not reappear within 10 s after reset — proceeding anyway.");

  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Device reset failed (%s) — proceeding without reset.", e.what());
  }
}

void RealsenseCameraNode::start_pipeline()
{
  if (!serial_no_.empty()) cfg_.enable_device(serial_no_);

  if (camera_model_ == CameraModel::T265) {
    cfg_.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg_.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
    cfg_.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
  } else {
    if (enable_color_) cfg_.enable_stream(RS2_STREAM_COLOR, color_width_, color_height_, RS2_FORMAT_RGB8, color_fps_);
    if (enable_depth_) cfg_.enable_stream(RS2_STREAM_DEPTH, depth_width_, depth_height_, RS2_FORMAT_Z16, depth_fps_);
    // IMU is intentionally NOT added to pipeline config — opened via direct sensor API
    // below to avoid the pipeline sync module holding video frames waiting for IMU timestamps
  }

  // Start video pipeline with callback.
  // If the exact requested config isn't supported by this device (e.g. L515 has no 640x480 color),
  // fall back to auto-profile so librealsense picks the closest valid resolution/fps.
  running_ = true;
  rs2::pipeline_profile profile;
  try {
    profile = pipe_.start(cfg_, [this](rs2::frame f) { on_frame(f); });
  } catch (const rs2::error & e) {
    RCLCPP_WARN(get_logger(),
      "Pipeline start failed with requested config (%s) — retrying with auto profiles.", e.what());
    cfg_ = rs2::config();
    if (!serial_no_.empty()) cfg_.enable_device(serial_no_);
    if (camera_model_ == CameraModel::T265) {
      cfg_.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
      cfg_.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
      cfg_.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    } else {
      if (enable_color_) cfg_.enable_stream(RS2_STREAM_COLOR, -1, 0, 0, RS2_FORMAT_RGB8, 0);
      if (enable_depth_) cfg_.enable_stream(RS2_STREAM_DEPTH, -1, 0, 0, RS2_FORMAT_Z16, 0);
    }
    try {
      profile = pipe_.start(cfg_, [this](rs2::frame f) { on_frame(f); });
    } catch (const rs2::error & e2) {
      RCLCPP_ERROR(get_logger(), "Auto-profile fallback also failed: %s", e2.what());
      throw std::runtime_error(std::string("Pipeline cannot start: ") + e2.what());
    }
  }

  if (camera_model_ != CameraModel::T265) {
    if (enable_color_) {
      auto cp = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
      color_info_msg_ = build_camera_info(cp, camera_name_ + "_color_optical_frame");
      RCLCPP_INFO(get_logger(), "Color stream: %dx%d @ %dfps  format=%d",
        cp.width(), cp.height(), cp.fps(), static_cast<int>(cp.format()));
    }
    if (enable_depth_) {
      auto dp = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
      depth_info_msg_ = build_camera_info(dp, camera_name_ + "_depth_optical_frame");
      RCLCPP_INFO(get_logger(), "Depth stream: %dx%d @ %dfps  format=%d",
        dp.width(), dp.height(), dp.fps(), static_cast<int>(dp.format()));
    }
  }

  // Log USB connection type; warn if USB 2.x (bandwidth may be insufficient for full resolution)
  auto dev = profile.get_device();
  if (dev.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)) {
    std::string usb_type = dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
    RCLCPP_INFO(get_logger(), "USB connection type: %s", usb_type.c_str());
    if (usb_type.find("2.") != std::string::npos) {
      RCLCPP_WARN(get_logger(),
        "Device connected via %s — reduced bandwidth. Consider lowering color/depth fps or resolution.",
        usb_type.c_str());
    }
  }

  if (enable_imu_) start_imu_sensor(dev);
  RCLCPP_INFO(get_logger(), "Pipeline started (callback mode).");
}

void RealsenseCameraNode::start_imu_sensor(rs2::device device)
{
  // Quick pre-check: does this device have ANY motion-capable sensor at all?
  // Devices without a Motion Module (e.g. L515) have no accel/gyro profiles — skip immediately.
  bool has_motion = false;
  for (auto sensor : device.query_sensors()) {
    for (auto & p : sensor.get_stream_profiles()) {
      if (p.stream_type() == RS2_STREAM_ACCEL || p.stream_type() == RS2_STREAM_GYRO) {
        has_motion = true; break;
      }
    }
    if (has_motion) break;
  }
  if (!has_motion) {
    RCLCPP_WARN(get_logger(), "Device has no Motion Module — IMU disabled.");
    return;
  }

  // Motion Module (HID sensor) may take extra time to enumerate after a USB reset.
  // Retry for up to 5 s before giving up.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);

  while (std::chrono::steady_clock::now() < deadline) {
    for (auto sensor : device.query_sensors()) {
      auto profiles = sensor.get_stream_profiles();
      rs2::stream_profile accel_profile, gyro_profile;

      for (auto & p : profiles) {
        // Match official realsense-ros: prefer the device's default profile fps.
        // Fallback: pick lowest fps among MOTION_XYZ32F profiles (63Hz accel, 200Hz gyro on D435i)
        // to stay within USB bandwidth and avoid overloading downstream consumers.
        if (p.stream_type() == RS2_STREAM_ACCEL && p.format() == RS2_FORMAT_MOTION_XYZ32F) {
          if (!accel_profile || p.fps() < accel_profile.fps()) accel_profile = p;
        } else if (p.stream_type() == RS2_STREAM_GYRO && p.format() == RS2_FORMAT_MOTION_XYZ32F) {
          if (!gyro_profile || p.fps() < gyro_profile.fps()) gyro_profile = p;
        }
      }

      if (!accel_profile || !gyro_profile) continue;

      sensor.open({accel_profile, gyro_profile});
      sensor.start([this](rs2::frame f) { on_frame(f); });
      imu_sensor_ = sensor;
      RCLCPP_INFO(get_logger(), "IMU sensor opened (accel@%dfps gyro@%dfps).",
        accel_profile.fps(), gyro_profile.fps());
      return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    RCLCPP_DEBUG(get_logger(), "Motion Module not yet enumerated, retrying...");
  }
  RCLCPP_WARN(get_logger(), "No motion sensor found after 5 s — IMU disabled.");
}

void RealsenseCameraNode::on_frame(rs2::frame frame)
{
  if (!running_) return;

  try {
    // Pipeline sync module delivers color+depth as a composite frameset — unpack explicitly
    if (auto fs = frame.as<rs2::frameset>()) {
      if (enable_color_) {
        if (auto color = fs.get_color_frame())
          publish_video_frame(color, color_image_pub_, color_info_pub_, color_info_msg_);
      }
      if (enable_depth_) {
        if (auto depth = fs.get_depth_frame())
          publish_video_frame(depth, depth_image_pub_, depth_info_pub_, depth_info_msg_);
      }
      if (camera_model_ == CameraModel::T265) {
        fs.foreach_rs([this](rs2::frame f) {
          if (auto vf = f.as<rs2::video_frame>()) {
            if (vf.get_profile().stream_type() == RS2_STREAM_FISHEYE) {
              int idx = vf.get_profile().stream_index();
              if (idx == 1) publish_video_frame(vf, fisheye1_image_pub_, fisheye1_info_pub_, fisheye1_info_msg_);
              else           publish_video_frame(vf, fisheye2_image_pub_, fisheye2_info_pub_, fisheye2_info_msg_);
            }
          }
        });
      }
      return;
    }

    if (auto vf = frame.as<rs2::video_frame>()) {
      switch (vf.get_profile().stream_type()) {
        case RS2_STREAM_COLOR:
          publish_video_frame(vf, color_image_pub_, color_info_pub_, color_info_msg_);
          break;
        case RS2_STREAM_DEPTH:
          publish_video_frame(vf, depth_image_pub_, depth_info_pub_, depth_info_msg_);
          break;
        case RS2_STREAM_FISHEYE: {
          int idx = vf.get_profile().stream_index();
          if (idx == 1) publish_video_frame(vf, fisheye1_image_pub_, fisheye1_info_pub_, fisheye1_info_msg_);
          else           publish_video_frame(vf, fisheye2_image_pub_, fisheye2_info_pub_, fisheye2_info_msg_);
          break;
        }
        default: break;
      }
    } else if (auto mf = frame.as<rs2::motion_frame>()) {
      // Cache accel; publish IMU when gyro arrives (copy method)
      if (mf.get_profile().stream_type() == RS2_STREAM_ACCEL) {
        last_accel_frame_ = mf;
      } else if (mf.get_profile().stream_type() == RS2_STREAM_GYRO && last_accel_frame_) {
        publish_imu_data(last_accel_frame_.as<rs2::motion_frame>(), mf);
      }
    } else if (auto pf = frame.as<rs2::pose_frame>()) {
      publish_pose_frame(pf);
    }
  } catch (const std::exception & e) {
    if (running_) RCLCPP_WARN(get_logger(), "Frame callback error: %s", e.what());
  }
}

void RealsenseCameraNode::publish_imu_data(const rs2::motion_frame& accel, const rs2::motion_frame& gyro)
{
  auto msg = std::make_unique<sensor_msgs::msg::Imu>();
  msg->header.stamp = now();
  msg->header.frame_id = camera_name_ + "_imu_optical_frame";

  auto g = gyro.get_motion_data();
  auto a = accel.get_motion_data();

  msg->angular_velocity.x = g.x;
  msg->angular_velocity.y = g.y;
  msg->angular_velocity.z = g.z;
  msg->linear_acceleration.x = a.x;
  msg->linear_acceleration.y = a.y;
  msg->linear_acceleration.z = a.z;

  imu_pub_->publish(std::move(msg));
}

sensor_msgs::msg::CameraInfo RealsenseCameraNode::build_camera_info(
  const rs2::video_stream_profile & profile, const std::string & frame_id) const
{
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = frame_id;
  info.width = profile.width();
  info.height = profile.height();

  auto i = profile.get_intrinsics();
  info.distortion_model = "plumb_bob";
  info.d.assign(i.coeffs, i.coeffs + 5);

  info.k = {i.fx, 0.0, i.ppx,
             0.0, i.fy, i.ppy,
             0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0};
  info.p = {i.fx, 0.0, i.ppx, 0.0,
             0.0, i.fy, i.ppy, 0.0,
             0.0, 0.0, 1.0,   0.0};
  return info;
}

void RealsenseCameraNode::publish_video_frame(
  const rs2::video_frame & frame,
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub,
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub,
  const sensor_msgs::msg::CameraInfo & info_template)
{
  if (!pub || !info_pub) return;

  auto img = std::make_unique<sensor_msgs::msg::Image>();
  img->header.stamp = now();
  img->header.frame_id = info_template.header.frame_id;
  img->width = frame.get_width();
  img->height = frame.get_height();
  img->step = frame.get_stride_in_bytes();

  switch (frame.get_profile().format()) {
    case RS2_FORMAT_RGB8:  img->encoding = sensor_msgs::image_encodings::RGB8; break;
    case RS2_FORMAT_Z16:   img->encoding = sensor_msgs::image_encodings::TYPE_16UC1; break;
    case RS2_FORMAT_Y8:    img->encoding = sensor_msgs::image_encodings::MONO8; break;
    default:               img->encoding = sensor_msgs::image_encodings::RGB8; break;
  }

  const auto * data = reinterpret_cast<const uint8_t *>(frame.get_data());
  img->data.assign(data, data + img->height * img->step);

  auto info = std::make_unique<sensor_msgs::msg::CameraInfo>(info_template);
  info->header = img->header;

  pub->publish(std::move(img));
  info_pub->publish(std::move(info));
}

void RealsenseCameraNode::publish_pose_frame(const rs2::pose_frame & frame)
{
  if (!odom_pub_) return;

  auto pose = frame.get_pose_data();
  auto msg = std::make_unique<nav_msgs::msg::Odometry>();
  msg->header.stamp = now();
  msg->header.frame_id = camera_name_ + "_odom_frame";
  msg->child_frame_id = camera_name_ + "_pose_frame";

  msg->pose.pose.position.x = pose.translation.x;
  msg->pose.pose.position.y = pose.translation.y;
  msg->pose.pose.position.z = pose.translation.z;
  msg->pose.pose.orientation.x = pose.rotation.x;
  msg->pose.pose.orientation.y = pose.rotation.y;
  msg->pose.pose.orientation.z = pose.rotation.z;
  msg->pose.pose.orientation.w = pose.rotation.w;

  msg->twist.twist.linear.x = pose.velocity.x;
  msg->twist.twist.linear.y = pose.velocity.y;
  msg->twist.twist.linear.z = pose.velocity.z;
  msg->twist.twist.angular.x = pose.angular_velocity.x;
  msg->twist.twist.angular.y = pose.angular_velocity.y;
  msg->twist.twist.angular.z = pose.angular_velocity.z;

  odom_pub_->publish(std::move(msg));
}

} // namespace realsense_camera_bingup

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<realsense_camera_bingup::RealsenseCameraNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}