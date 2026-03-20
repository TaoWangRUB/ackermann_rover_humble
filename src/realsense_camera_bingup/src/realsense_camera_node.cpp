#include "realsense_camera_bingup/realsense_camera_node.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <chrono>
#include <thread>
#include <cstdio>

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

// ---------------------------------------------------------------------------
// Profile string parser: "WxHxFPS" e.g. "640x480x30"
// ---------------------------------------------------------------------------
bool RealsenseCameraNode::parse_profile_string(
  const std::string & profile_str, int & width, int & height, int & fps)
{
  if (profile_str.empty()) return false;
  return std::sscanf(profile_str.c_str(), "%dx%dx%d", &width, &height, &fps) == 3;
}

// ---------------------------------------------------------------------------
// declare_parameters
// ---------------------------------------------------------------------------
void RealsenseCameraNode::declare_parameters()
{
  // --- Identity ---
  camera_name_      = declare_parameter<std::string>("camera_name", "camera");
  serial_no_        = declare_parameter<std::string>("serial_no", "");
  camera_namespace_ = declare_parameter<std::string>("camera_namespace", "");
  device_type_      = declare_parameter<std::string>("device_type", "");

  const std::string model_str = declare_parameter<std::string>("camera_model", "d435i");
  if (model_str == "l515") camera_model_ = CameraModel::L515;
  else if (model_str == "t265") camera_model_ = CameraModel::T265;
  else camera_model_ = CameraModel::D435I;

  // --- Stream enables ---
  enable_color_ = declare_parameter<bool>("enable_color", true);
  enable_depth_ = declare_parameter<bool>("enable_depth", true);

  // IMU: enable_imu is a convenience flag that enables both accel+gyro.
  // enable_imu=true forces both on; individual flags can also enable independently.
  enable_imu_   = declare_parameter<bool>("enable_imu", false);
  enable_accel_ = declare_parameter<bool>("enable_accel", false);
  enable_gyro_  = declare_parameter<bool>("enable_gyro", false);
  if (enable_imu_) { enable_accel_ = true; enable_gyro_ = true; }
  // Reconcile: if either granular flag is true, we need the motion sensor
  enable_imu_ = enable_accel_ || enable_gyro_;

  unite_imu_method_ = declare_parameter<int>("unite_imu_method", 2);

  enable_infra1_ = declare_parameter<bool>("enable_infra1", false);
  enable_infra2_ = declare_parameter<bool>("enable_infra2", false);

  // T265 fisheye (default off — odom-only mode)
  enable_fisheye_ = declare_parameter<bool>("enable_fisheye", false);

  // --- Color stream resolution ---
  color_width_  = declare_parameter<int>("color_width",  640);
  color_height_ = declare_parameter<int>("color_height", 480);
  color_fps_    = declare_parameter<int>("color_fps",    30);

  // --- Depth stream resolution ---
  depth_width_  = declare_parameter<int>("depth_width",  640);
  depth_height_ = declare_parameter<int>("depth_height", 480);
  depth_fps_    = declare_parameter<int>("depth_fps",    30);

  // --- Profile string overrides (WxHxFPS) ---
  rgb_color_profile_  = declare_parameter<std::string>("rgb_camera.color_profile", "");
  depth_profile_str_  = declare_parameter<std::string>("depth_module.depth_profile", "");

  int pw, ph, pf;
  if (parse_profile_string(rgb_color_profile_, pw, ph, pf)) {
    color_width_ = pw; color_height_ = ph; color_fps_ = pf;
    RCLCPP_INFO(get_logger(), "RGB profile override: %dx%d@%dfps", pw, ph, pf);
  }
  if (parse_profile_string(depth_profile_str_, pw, ph, pf)) {
    depth_width_ = pw; depth_height_ = ph; depth_fps_ = pf;
    RCLCPP_INFO(get_logger(), "Depth profile override: %dx%d@%dfps", pw, ph, pf);
  }

  // --- RGB sensor controls ---
  rgb_auto_exposure_ = declare_parameter<bool>("rgb_camera.enable_auto_exposure", true);
  rgb_exposure_      = declare_parameter<int>("rgb_camera.exposure", 0);
  rgb_gain_          = declare_parameter<int>("rgb_camera.gain", 0);

  // --- Depth sensor controls ---
  depth_auto_exposure_ = declare_parameter<bool>("depth_module.enable_auto_exposure", true);
  depth_exposure_      = declare_parameter<int>("depth_module.exposure", 0);
  depth_gain_          = declare_parameter<int>("depth_module.gain", 0);

  // --- Sync / alignment ---
  enable_sync_ = declare_parameter<bool>("enable_sync", false);
  // Support both new dotted name and old flat name for backward compat
  align_depth_enable_ = declare_parameter<bool>("align_depth.enable", false);
  bool align_old = declare_parameter<bool>("align_depth_to_color", false);
  if (align_old && !align_depth_enable_) align_depth_enable_ = true;

  // --- Stubbed parameters (declared for compatibility, not yet functional) ---
  if (declare_parameter<bool>("publish_tf", false))
    RCLCPP_WARN(get_logger(), "publish_tf is not yet implemented — TF frames will not be broadcast.");
  if (declare_parameter<bool>("enable_rgbd", false))
    RCLCPP_WARN(get_logger(), "enable_rgbd is not yet implemented.");
  if (declare_parameter<bool>("pointcloud.enable", false))
    RCLCPP_WARN(get_logger(), "pointcloud is not yet implemented.");
  if (enable_sync_)
    RCLCPP_INFO(get_logger(), "enable_sync=true — pipeline sync is always active (default behavior).");

  // T265 fisheye
  declare_parameter<int>("fisheye_fps", 30);
}

// ---------------------------------------------------------------------------
// create_publishers
// ---------------------------------------------------------------------------
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
  if (enable_infra1_) {
    infra1_image_pub_ = create_publisher<sensor_msgs::msg::Image>(camera_name_ + "/infra1/image_rect_raw", qos);
    infra1_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_ + "/infra1/camera_info", qos);
  }
  if (enable_infra2_) {
    infra2_image_pub_ = create_publisher<sensor_msgs::msg::Image>(camera_name_ + "/infra2/image_rect_raw", qos);
    infra2_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_ + "/infra2/camera_info", qos);
  }
  if (enable_imu_) {
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(camera_name_ + "/imu", qos);
  }
  if (camera_model_ == CameraModel::T265) {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(camera_name_ + "/odom", qos);
    if (enable_fisheye_) {
      fisheye1_image_pub_ = create_publisher<sensor_msgs::msg::Image>(camera_name_ + "/fisheye1/image_raw", qos);
      fisheye1_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_ + "/fisheye1/camera_info", qos);
      fisheye2_image_pub_ = create_publisher<sensor_msgs::msg::Image>(camera_name_ + "/fisheye2/image_raw", qos);
      fisheye2_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_ + "/fisheye2/camera_info", qos);
    }
  }
}

// ---------------------------------------------------------------------------
// reset_device — throws on mismatch instead of warning
// ---------------------------------------------------------------------------
void RealsenseCameraNode::reset_device()
{
  RCLCPP_INFO(get_logger(), "Resetting RealSense device to clear stuck states...");

  // rs2::context enumerates USB devices asynchronously — give it time to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  auto devices = ctx_.query_devices();
  if (devices.size() == 0) {
    throw std::runtime_error("No RealSense device found.");
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

      // Match by serial number if specified
      if (!serial_no_.empty()) {
        if (serial == serial_no_) { dev = d; break; }
        continue;
      }
      // Match by device_type if specified (substring match on device name)
      if (!device_type_.empty()) {
        if (name.find(device_type_) != std::string::npos) { dev = d; break; }
        continue;
      }
      // Default: match by camera_model
      if (name.find(expected_model) != std::string::npos) { dev = d; break; }
    } catch (const rs2::error &) { continue; }  // device in bad state — skip
  }

  if (!dev) {
    std::string match_key = serial_no_.empty()
      ? (device_type_.empty() ? expected_model : device_type_)
      : serial_no_;
    throw std::runtime_error(
      std::string("No RealSense device matching '") + match_key + "' found.");
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
}

// ---------------------------------------------------------------------------
// start_pipeline
// ---------------------------------------------------------------------------
void RealsenseCameraNode::start_pipeline()
{
  if (!serial_no_.empty()) cfg_.enable_device(serial_no_);

  if (camera_model_ == CameraModel::T265) {
    if (enable_fisheye_) {
      cfg_.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
      cfg_.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
    }
    cfg_.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
  } else {
    if (enable_color_) cfg_.enable_stream(RS2_STREAM_COLOR, color_width_, color_height_, RS2_FORMAT_RGB8, color_fps_);
    if (enable_depth_) cfg_.enable_stream(RS2_STREAM_DEPTH, depth_width_, depth_height_, RS2_FORMAT_Z16, depth_fps_);
    if (enable_infra1_) cfg_.enable_stream(RS2_STREAM_INFRARED, 1, depth_width_, depth_height_, RS2_FORMAT_Y8, depth_fps_);
    if (enable_infra2_) cfg_.enable_stream(RS2_STREAM_INFRARED, 2, depth_width_, depth_height_, RS2_FORMAT_Y8, depth_fps_);
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
      if (enable_fisheye_) {
        cfg_.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
        cfg_.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
      }
      cfg_.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    } else {
      if (enable_color_) cfg_.enable_stream(RS2_STREAM_COLOR, -1, 0, 0, RS2_FORMAT_RGB8, 0);
      if (enable_depth_) cfg_.enable_stream(RS2_STREAM_DEPTH, -1, 0, 0, RS2_FORMAT_Z16, 0);
      if (enable_infra1_) cfg_.enable_stream(RS2_STREAM_INFRARED, 1, 0, 0, RS2_FORMAT_Y8, 0);
      if (enable_infra2_) cfg_.enable_stream(RS2_STREAM_INFRARED, 2, 0, 0, RS2_FORMAT_Y8, 0);
    }
    try {
      profile = pipe_.start(cfg_, [this](rs2::frame f) { on_frame(f); });
    } catch (const rs2::error & e2) {
      RCLCPP_ERROR(get_logger(), "Auto-profile fallback also failed: %s", e2.what());
      throw std::runtime_error(std::string("Pipeline cannot start: ") + e2.what());
    }
  }

  // Build CameraInfo from active stream profiles
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
    if (enable_infra1_) {
      auto ip = profile.get_stream(RS2_STREAM_INFRARED, 1).as<rs2::video_stream_profile>();
      infra1_info_msg_ = build_camera_info(ip, camera_name_ + "_infra1_optical_frame");
      RCLCPP_INFO(get_logger(), "Infra1 stream: %dx%d @ %dfps", ip.width(), ip.height(), ip.fps());
    }
    if (enable_infra2_) {
      auto ip = profile.get_stream(RS2_STREAM_INFRARED, 2).as<rs2::video_stream_profile>();
      infra2_info_msg_ = build_camera_info(ip, camera_name_ + "_infra2_optical_frame");
      RCLCPP_INFO(get_logger(), "Infra2 stream: %dx%d @ %dfps", ip.width(), ip.height(), ip.fps());
    }
  }

  // Depth alignment
  if (align_depth_enable_ && enable_depth_ && enable_color_) {
    align_to_color_ = std::make_shared<rs2::align>(RS2_STREAM_COLOR);
    const auto qos = rclcpp::QoS(10);
    aligned_depth_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      camera_name_ + "/aligned_depth_to_color/image_raw", qos);
    aligned_depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      camera_name_ + "/aligned_depth_to_color/camera_info", qos);
    RCLCPP_INFO(get_logger(), "Depth alignment to color enabled.");
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

  // Apply sensor options (exposure, gain, auto_exposure)
  apply_sensor_options(profile);

  if (enable_imu_) start_imu_sensor(dev);
  RCLCPP_INFO(get_logger(), "Pipeline started (callback mode).");
}

// ---------------------------------------------------------------------------
// apply_sensor_options — exposure / gain / auto_exposure per sensor
// ---------------------------------------------------------------------------
void RealsenseCameraNode::apply_sensor_options(rs2::pipeline_profile & profile)
{
  auto dev = profile.get_device();
  for (auto & sensor : dev.query_sensors()) {
    bool is_color = false, is_depth = false;
    for (auto & p : sensor.get_stream_profiles()) {
      if (p.stream_type() == RS2_STREAM_COLOR) is_color = true;
      if (p.stream_type() == RS2_STREAM_DEPTH || p.stream_type() == RS2_STREAM_INFRARED) is_depth = true;
    }

    // --- RGB sensor ---
    if (is_color && enable_color_) {
      try {
        if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
          sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, rgb_auto_exposure_ ? 1.0f : 0.0f);
          RCLCPP_INFO(get_logger(), "RGB auto_exposure=%s", rgb_auto_exposure_ ? "true" : "false");
        }
        if (!rgb_auto_exposure_) {
          if (rgb_exposure_ > 0 && sensor.supports(RS2_OPTION_EXPOSURE)) {
            sensor.set_option(RS2_OPTION_EXPOSURE, static_cast<float>(rgb_exposure_));
            RCLCPP_INFO(get_logger(), "RGB exposure=%d", rgb_exposure_);
          }
          if (rgb_gain_ > 0 && sensor.supports(RS2_OPTION_GAIN)) {
            sensor.set_option(RS2_OPTION_GAIN, static_cast<float>(rgb_gain_));
            RCLCPP_INFO(get_logger(), "RGB gain=%d", rgb_gain_);
          }
        }
      } catch (const rs2::error & e) {
        RCLCPP_WARN(get_logger(), "Failed to set RGB sensor option: %s", e.what());
      }
    }

    // --- Depth sensor ---
    if (is_depth && enable_depth_) {
      try {
        if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
          sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, depth_auto_exposure_ ? 1.0f : 0.0f);
          RCLCPP_INFO(get_logger(), "Depth auto_exposure=%s", depth_auto_exposure_ ? "true" : "false");
        }
        if (!depth_auto_exposure_) {
          if (depth_exposure_ > 0 && sensor.supports(RS2_OPTION_EXPOSURE)) {
            sensor.set_option(RS2_OPTION_EXPOSURE, static_cast<float>(depth_exposure_));
            RCLCPP_INFO(get_logger(), "Depth exposure=%d", depth_exposure_);
          }
          if (depth_gain_ > 0 && sensor.supports(RS2_OPTION_GAIN)) {
            sensor.set_option(RS2_OPTION_GAIN, static_cast<float>(depth_gain_));
            RCLCPP_INFO(get_logger(), "Depth gain=%d", depth_gain_);
          }
        }
      } catch (const rs2::error & e) {
        RCLCPP_WARN(get_logger(), "Failed to set depth sensor option: %s", e.what());
      }
    }
  }
}

// ---------------------------------------------------------------------------
// start_imu_sensor — respects enable_accel_ / enable_gyro_ independently
// ---------------------------------------------------------------------------
void RealsenseCameraNode::start_imu_sensor(rs2::device device)
{
  // Quick pre-check: does this device have ANY motion-capable sensor at all?
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
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);

  while (std::chrono::steady_clock::now() < deadline) {
    for (auto sensor : device.query_sensors()) {
      auto profiles = sensor.get_stream_profiles();
      rs2::stream_profile accel_profile, gyro_profile;

      for (auto & p : profiles) {
        if (p.stream_type() == RS2_STREAM_ACCEL && p.format() == RS2_FORMAT_MOTION_XYZ32F) {
          if (!accel_profile || p.fps() < accel_profile.fps()) accel_profile = p;
        } else if (p.stream_type() == RS2_STREAM_GYRO && p.format() == RS2_FORMAT_MOTION_XYZ32F) {
          if (!gyro_profile || p.fps() < gyro_profile.fps()) gyro_profile = p;
        }
      }

      // Build the list of profiles to open based on granular flags
      std::vector<rs2::stream_profile> open_profiles;
      if (enable_accel_ && accel_profile) open_profiles.push_back(accel_profile);
      if (enable_gyro_ && gyro_profile) open_profiles.push_back(gyro_profile);

      if (open_profiles.empty()) continue;

      sensor.open(open_profiles);
      sensor.start([this](rs2::frame f) { on_frame(f); });
      imu_sensor_ = sensor;

      std::string opened;
      if (enable_accel_ && accel_profile) opened += "accel@" + std::to_string(accel_profile.fps()) + "fps ";
      if (enable_gyro_ && gyro_profile) opened += "gyro@" + std::to_string(gyro_profile.fps()) + "fps";
      RCLCPP_INFO(get_logger(), "IMU sensor opened (%s).", opened.c_str());
      return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    RCLCPP_DEBUG(get_logger(), "Motion Module not yet enumerated, retrying...");
  }
  RCLCPP_WARN(get_logger(), "No motion sensor found after 5 s — IMU disabled.");
}

// ---------------------------------------------------------------------------
// on_frame — handles alignment + infrared dispatch
// ---------------------------------------------------------------------------
void RealsenseCameraNode::on_frame(rs2::frame frame)
{
  if (!running_) return;

  try {
    // Pipeline sync module delivers color+depth as a composite frameset — unpack explicitly
    if (auto fs = frame.as<rs2::frameset>()) {
      // Apply depth alignment if enabled
      rs2::frameset processed = fs;
      if (align_to_color_) {
        processed = align_to_color_->process(fs);
      }

      if (enable_color_) {
        if (auto color = processed.get_color_frame())
          publish_video_frame(color, color_image_pub_, color_info_pub_, color_info_msg_);
      }
      if (enable_depth_) {
        // Publish raw depth from original frameset
        if (auto depth = fs.get_depth_frame())
          publish_video_frame(depth, depth_image_pub_, depth_info_pub_, depth_info_msg_);
        // Publish aligned depth (uses color intrinsics)
        if (align_to_color_) {
          if (auto aligned = processed.get_depth_frame())
            publish_video_frame(aligned, aligned_depth_image_pub_, aligned_depth_info_pub_, color_info_msg_);
        }
      }

      // Infrared frames
      if (enable_infra1_ || enable_infra2_) {
        fs.foreach_rs([this](rs2::frame f) {
          if (auto vf = f.as<rs2::video_frame>()) {
            if (vf.get_profile().stream_type() == RS2_STREAM_INFRARED) {
              int idx = vf.get_profile().stream_index();
              if (idx == 1 && enable_infra1_)
                publish_video_frame(vf, infra1_image_pub_, infra1_info_pub_, infra1_info_msg_);
              else if (idx == 2 && enable_infra2_)
                publish_video_frame(vf, infra2_image_pub_, infra2_info_pub_, infra2_info_msg_);
            }
          }
        });
      }

      // T265 fisheye (only when enabled)
      if (camera_model_ == CameraModel::T265 && enable_fisheye_) {
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
        case RS2_STREAM_INFRARED: {
          int idx = vf.get_profile().stream_index();
          if (idx == 1 && enable_infra1_) publish_video_frame(vf, infra1_image_pub_, infra1_info_pub_, infra1_info_msg_);
          else if (idx == 2 && enable_infra2_) publish_video_frame(vf, infra2_image_pub_, infra2_info_pub_, infra2_info_msg_);
          break;
        }
        case RS2_STREAM_FISHEYE: {
          if (enable_fisheye_) {
            int idx = vf.get_profile().stream_index();
            if (idx == 1) publish_video_frame(vf, fisheye1_image_pub_, fisheye1_info_pub_, fisheye1_info_msg_);
            else           publish_video_frame(vf, fisheye2_image_pub_, fisheye2_info_pub_, fisheye2_info_msg_);
          }
          break;
        }
        default: break;
      }
    } else if (auto mf = frame.as<rs2::motion_frame>()) {
      // Cache accel; publish fused IMU when gyro arrives (copy method)
      if (mf.get_profile().stream_type() == RS2_STREAM_ACCEL) {
        last_accel_frame_ = mf;
      } else if (mf.get_profile().stream_type() == RS2_STREAM_GYRO && last_accel_frame_) {
        // Only publish fused IMU if both accel and gyro are enabled
        if (enable_accel_ && enable_gyro_) {
          publish_imu_data(last_accel_frame_.as<rs2::motion_frame>(), mf);
        }
      }
    } else if (auto pf = frame.as<rs2::pose_frame>()) {
      publish_pose_frame(pf);
    }
  } catch (const std::exception & e) {
    if (running_) RCLCPP_WARN(get_logger(), "Frame callback error: %s", e.what());
  }
}

// ---------------------------------------------------------------------------
// publish_imu_data
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// build_camera_info
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// publish_video_frame
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// publish_pose_frame
// ---------------------------------------------------------------------------
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
