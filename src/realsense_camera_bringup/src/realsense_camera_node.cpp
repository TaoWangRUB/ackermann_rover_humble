#include "realsense_camera_bringup/realsense_camera_node.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <chrono>
#include <thread>
#include <cstdio>
#include <cstring>

namespace realsense_camera_bringup
{

RealsenseCameraNode::RealsenseCameraNode(const rclcpp::NodeOptions & options)
: Node("realsense_camera_node", options), pipe_(ctx_)
{
  declare_parameters();
  create_publishers();

  // Retry loop — handles two classes of transient USB/firmware errors:
  //   1. "failed to set power state" / "Unable to open device" — enumeration race
  //      on shared hubs; just wait and retry (no extra reset needed).
  //   2. "Couldn't resolve requests" / "device is streaming" — firmware stuck in
  //      streaming state from a previously killed process; force a hardware reset
  //      to release the device before retrying.
  constexpr int MAX_RETRIES = 5;
  // T265 needs longer waits — its Movidius VPU takes 15-20s to re-enumerate
  // after a hardware reset or USB reconnect.
  const double retry_delay_s = (camera_model_ == CameraModel::T265) ? 10.0 : 5.0;
  bool did_hardware_reset = false;

  for (int attempt = 1; attempt <= MAX_RETRIES; ++attempt) {
    try {
      if (enable_hardware_reset_ && !did_hardware_reset) {
        reset_device();
        did_hardware_reset = true;
      }
      start_pipeline();
      start_data_flow_watchdog();
      return;   // success
    } catch (const std::exception & e) {
      const std::string what = e.what();
      const bool is_stuck  = what.find("Couldn't resolve requests") != std::string::npos ||
                             what.find("device is streaming")       != std::string::npos;
      const bool retriable = is_stuck ||
                             what.find("Unable to open device")     != std::string::npos ||
                             what.find("failed to set power")       != std::string::npos ||
                             what.find("No device connected")       != std::string::npos;

      if (retriable && attempt < MAX_RETRIES) {
        RCLCPP_WARN(get_logger(),
          "Camera init attempt %d/%d failed (%s) — retrying in %.0fs...",
          attempt, MAX_RETRIES, what.c_str(), retry_delay_s);

        // For T265: forced reset on "stuck" worsens USB state — just wait.
        // For D435i/L515: a forced reset can clear stuck streaming.
        if (is_stuck && !did_hardware_reset && camera_model_ != CameraModel::T265) {
          RCLCPP_WARN(get_logger(), "Device appears stuck — issuing forced hardware reset.");
          try { reset_device(); did_hardware_reset = true; } catch (const std::exception & re) {
            RCLCPP_WARN(get_logger(), "Forced reset failed: %s — will retry anyway.", re.what());
          }
        }

        std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>(retry_delay_s * 1000)));

        // Recreate pipeline + config to avoid stale state from failed start.
        // After pipe_.start() fails, the pipeline object may hold a dead USB
        // handle; creating a fresh one forces librealsense to re-enumerate.
        ctx_ = rs2::context();
        pipe_ = rs2::pipeline(ctx_);
        cfg_ = rs2::config();
      } else {
        RCLCPP_FATAL(get_logger(), "Failed to initialise camera: %s", what.c_str());
        throw;
      }
    }
  }
}

RealsenseCameraNode::~RealsenseCameraNode()
{
  RCLCPP_INFO(get_logger(), "Shutting down — stopping pipeline...");
  running_ = false;

  // Cancel the data-flow watchdog to prevent it from firing during shutdown.
  if (data_flow_watchdog_timer_) {
    data_flow_watchdog_timer_->cancel();
  }

  // Wake and join the alignment worker before stopping the pipeline so it
  // doesn't try to process frames after pipe_.stop() frees them.
  align_cv_.notify_all();
  if (align_thread_.joinable()) align_thread_.join();

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
  // --- Startup delay (stagger multi-camera inits to avoid USB race) ---
  const double startup_delay_s = declare_parameter<double>("startup_delay_s", 0.0);
  if (startup_delay_s > 0.0) {
    RCLCPP_INFO(get_logger(), "Startup delay %.1fs — waiting for other cameras to settle...",
      startup_delay_s);
    std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(startup_delay_s * 1000)));
  }

  // --- Hardware reset ---
  // Default false — matches official realsense-ros (initial_reset=false).
  // Set true to clear firmware stuck-streaming states on startup.
  enable_hardware_reset_ = declare_parameter<bool>("enable_hardware_reset", false);

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

  // --- TF publishing ---
  publish_tf_ = declare_parameter<bool>("publish_tf", false);

  // --- Stubbed parameters (declared for compatibility, not yet functional) ---
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
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
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

  if (!enable_hardware_reset_) {
    RCLCPP_INFO(get_logger(), "Device found — skipping hardware reset (enable_hardware_reset=false).");
    return;
  }

  const std::string reset_serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

  // T265: unload the tracking module before reset so firmware releases its
  // USB handle cleanly (mirrors the official realsense-ros wrapper behaviour).
  if (camera_model_ == CameraModel::T265) {
    try { ctx_.unload_tracking_module(); } catch (...) {}
  }

  RCLCPP_INFO(get_logger(), "Issuing hardware_reset() to device [%s]...", reset_serial.c_str());
  try {
    dev.hardware_reset();
  } catch (const rs2::error & e) {
    RCLCPP_WARN(get_logger(), "hardware_reset() failed: %s — continuing without reset.", e.what());
    return;
  }

  // Poll until THIS specific device reappears (not just any RealSense).
  // T265 Movidius VPU takes 15-20s to re-enumerate after reset.
  const int poll_timeout_s = (camera_model_ == CameraModel::T265) ? 25 : 10;
  const int settle_time_s  = (camera_model_ == CameraModel::T265) ? 5  : 3;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(poll_timeout_s);
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // Fresh context to avoid cached stale device handles
    rs2::context poll_ctx;
    for (auto d : poll_ctx.query_devices()) {
      try {
        if (d.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == reset_serial) {
          RCLCPP_INFO(get_logger(), "Device USB reconnected — waiting %ds for sensors to initialise...",
            settle_time_s);
          std::this_thread::sleep_for(std::chrono::seconds(settle_time_s));
          // Refresh main context so start_pipeline sees the new device
          ctx_ = rs2::context();
          pipe_ = rs2::pipeline(ctx_);
          RCLCPP_INFO(get_logger(), "Device ready.");
          return;
        }
      } catch (const rs2::error &) { continue; }
    }
  }
  RCLCPP_WARN(get_logger(), "Device did not reappear within %d s after reset — proceeding anyway.",
    poll_timeout_s);
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
    // T265 IMU is delivered through the pipeline (not direct sensor API)
    if (enable_accel_) cfg_.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    if (enable_gyro_) cfg_.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
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
      if (enable_accel_) cfg_.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
      if (enable_gyro_) cfg_.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
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
  } else if (enable_fisheye_) {
    auto fp1 = profile.get_stream(RS2_STREAM_FISHEYE, 1).as<rs2::video_stream_profile>();
    auto fp2 = profile.get_stream(RS2_STREAM_FISHEYE, 2).as<rs2::video_stream_profile>();
    fisheye1_info_msg_ = build_camera_info(fp1, camera_name_ + "_fisheye1_optical_frame");
    fisheye2_info_msg_ = build_camera_info(fp2, camera_name_ + "_fisheye2_optical_frame");
    RCLCPP_INFO(get_logger(), "Fisheye1 stream: %dx%d @ %dfps", fp1.width(), fp1.height(), fp1.fps());
    RCLCPP_INFO(get_logger(), "Fisheye2 stream: %dx%d @ %dfps", fp2.width(), fp2.height(), fp2.fps());
  }

  // Depth alignment — rs2::align::process() is CPU-intensive (especially on ARM).
  // When CUDA is available, use GPU-accelerated alignment (~1-3ms vs ~250ms on Jetson).
  // Otherwise fall back to rs2::align on a worker thread.
  if (align_depth_enable_ && enable_depth_ && enable_color_) {
    const auto qos = rclcpp::QoS(10);
    aligned_depth_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      camera_name_ + "/aligned_depth_to_color/image_raw", qos);
    aligned_depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      camera_name_ + "/aligned_depth_to_color/camera_info", qos);

#ifdef HAVE_CUDA
    // Extract intrinsics and extrinsics from active pipeline profiles
    auto depth_profile = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_profile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto di = depth_profile.get_intrinsics();
    auto ci = color_profile.get_intrinsics();
    auto ext = depth_profile.get_extrinsics_to(color_profile);

    CameraIntrinsics d_intr{di.fx, di.fy, di.ppx, di.ppy, di.width, di.height};
    CameraIntrinsics c_intr{ci.fx, ci.fy, ci.ppx, ci.ppy, ci.width, ci.height};
    DepthColorExtrinsics d2c;
    std::copy(std::begin(ext.rotation), std::end(ext.rotation), std::begin(d2c.rotation));
    std::copy(std::begin(ext.translation), std::end(ext.translation), std::begin(d2c.translation));

    auto depth_sensor = profile.get_device().first<rs2::depth_sensor>();
    float depth_scale = depth_sensor.get_depth_scale();

    try {
      cuda_aligner_ = std::make_unique<CudaAligner>(d_intr, c_intr, d2c, depth_scale);
      aligned_depth_buf_.resize(static_cast<size_t>(ci.width) * ci.height, 0);
      RCLCPP_INFO(get_logger(), "Depth alignment: CUDA GPU-accelerated (depth_scale=%.6f)", depth_scale);
    } catch (const std::exception & e) {
      enable_cpu_alignment_fallback(e.what());
    }
#else
    align_to_color_ = std::make_shared<rs2::align>(RS2_STREAM_COLOR);
    RCLCPP_INFO(get_logger(), "Depth alignment: CPU rs2::align (async worker thread)");
#endif

    // Guard: only spawn a new alignment thread if none is running
    // (handles pipeline restart after watchdog trigger).
    if (!align_thread_.joinable()) {
      align_thread_ = std::thread(&RealsenseCameraNode::align_worker, this);
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

  // Apply sensor options (exposure, gain, auto_exposure)
  apply_sensor_options(profile);

  // T265 streams IMU through the pipeline (pose+motion are fused internally).
  // Only D435i/L515 need the direct sensor API for IMU.
  if (enable_imu_ && camera_model_ != CameraModel::T265) start_imu_sensor(dev);
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

#ifdef HAVE_CUDA
void RealsenseCameraNode::enable_cpu_alignment_fallback(const std::string & reason)
{
  if (!align_to_color_) {
    align_to_color_ = std::make_shared<rs2::align>(RS2_STREAM_COLOR);
  }

  const bool was_using_cuda = static_cast<bool>(cuda_aligner_);
  cuda_aligner_.reset();
  aligned_depth_buf_.clear();
  aligned_depth_buf_.shrink_to_fit();

  if (reason.empty()) {
    if (was_using_cuda) {
      RCLCPP_WARN(
        get_logger(),
        "CUDA depth alignment disabled at runtime. Falling back to CPU rs2::align.");
    } else {
      RCLCPP_INFO(get_logger(), "Depth alignment: CPU rs2::align (async worker thread)");
    }
    return;
  }

  RCLCPP_WARN(
    get_logger(),
    "CUDA depth alignment unavailable (%s). Falling back to CPU rs2::align.",
    reason.c_str());
}
#endif

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

  // Signal the data-flow watchdog that we're receiving data.
  if (!first_frame_received_.load(std::memory_order_relaxed)) {
    first_frame_received_.store(true, std::memory_order_relaxed);
  }

  try {
    // Pipeline sync module delivers color+depth as a composite frameset — unpack explicitly
    if (auto fs = frame.as<rs2::frameset>()) {
      // Publish raw color and depth immediately so they run at full sensor fps
      // regardless of how long alignment takes (alignment is offloaded to align_worker).
      // Compute color stamp once — the same stamp is forwarded to align_worker
      // so aligned depth is guaranteed to carry an identical header.stamp.
      rclcpp::Time color_stamp;
      bool have_color_stamp = false;
      if (enable_color_) {
        if (auto color = fs.get_color_frame()) {
          color_stamp = frame_ros_time(color);
          have_color_stamp = true;
          publish_video_frame(color, color_image_pub_, color_info_pub_, color_info_msg_, color_stamp);
        }
      }
      if (enable_depth_) {
        if (auto depth = fs.get_depth_frame())
          publish_video_frame(depth, depth_image_pub_, depth_info_pub_, depth_info_msg_);
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
      //
      // librealsense's pipeline sync module re-emits a frameset every time
      // any configured stream (pose / accel / gyro) has a new sample, and
      // each such frameset contains the *most recent* fisheye pair — not
      // necessarily a new one. Without deduplication, the fisheye topics get
      // published at the IMU rate (~200 Hz) instead of the shutter rate
      // (30 Hz), and downstream stereo synchronizers (e.g. cuvslam) see pairs
      // with identical timestamps, tripping cuVSLAM's strictly-increasing
      // frame-timestamp guard. Deduplicate by frame number.
      if (camera_model_ == CameraModel::T265 && enable_fisheye_) {
        fs.foreach_rs([this](rs2::frame f) {
          if (auto vf = f.as<rs2::video_frame>()) {
            if (vf.get_profile().stream_type() == RS2_STREAM_FISHEYE) {
              int idx = vf.get_profile().stream_index();
              const auto fn = vf.get_frame_number();
              if (idx == 1) {
                if (fn != last_fisheye1_frame_number_) {
                  last_fisheye1_frame_number_ = fn;
                  publish_video_frame(vf, fisheye1_image_pub_, fisheye1_info_pub_, fisheye1_info_msg_);
                }
              } else if (idx == 2) {
                if (fn != last_fisheye2_frame_number_) {
                  last_fisheye2_frame_number_ = fn;
                  publish_video_frame(vf, fisheye2_image_pub_, fisheye2_info_pub_, fisheye2_info_msg_);
                }
              }
            }
          }
        });
      }

      // Hand off to async alignment worker (if enabled). Drop oldest frame if
      // the worker is behind to bound queue memory usage.
      // Pass the pre-computed color stamp (or a fallback depth stamp) so the
      // aligned depth image carries exactly the same header.stamp as the raw
      // color image already published above — no re-derivation needed.
      if (align_depth_enable_) {
        rclcpp::Time align_stamp = have_color_stamp
            ? color_stamp
            : (fs.get_depth_frame()
                ? frame_ros_time(fs.get_depth_frame())
                : rclcpp::Time(0, 0, RCL_SYSTEM_TIME));
        std::lock_guard<std::mutex> lock(align_mutex_);
        if (align_queue_.size() >= ALIGN_QUEUE_MAX) {
          align_queue_.pop();
        }
        align_queue_.push({fs, align_stamp});
        align_cv_.notify_one();
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
// align_worker — runs on a dedicated thread to keep depth alignment off the
// pipeline callback. With HAVE_CUDA, uses GPU-accelerated alignment (~1-3ms
// on Jetson). Without CUDA, falls back to rs2::align (~250ms on ARM).
// ---------------------------------------------------------------------------
void RealsenseCameraNode::align_worker()
{
  while (true) {
    rs2::frameset fs;
    rclcpp::Time aligned_stamp;
    {
      std::unique_lock<std::mutex> lock(align_mutex_);
      align_cv_.wait(lock, [this] { return !align_queue_.empty() || !running_; });
      if (!running_ && align_queue_.empty()) break;
      auto & front = align_queue_.front();
      fs = front.first;
      aligned_stamp = front.second;
      align_queue_.pop();
    }
    try {
#ifdef HAVE_CUDA
      if (cuda_aligner_) {
        auto depth = fs.get_depth_frame();
        if (!depth) continue;

        try {
          const auto * src = reinterpret_cast<const uint16_t *>(depth.get_data());
          cuda_aligner_->align(src, aligned_depth_buf_.data());
          publish_aligned_depth(aligned_depth_buf_.data(),
                                cuda_aligner_->color_width(),
                                cuda_aligner_->color_height(),
                                aligned_stamp);
          continue;
        } catch (const std::exception & e) {
          enable_cpu_alignment_fallback(e.what());
        }
      }

      if (align_to_color_) {
        rs2::frameset processed = align_to_color_->process(fs);
        if (auto aligned = processed.get_depth_frame())
          publish_video_frame(
            aligned,
            aligned_depth_image_pub_,
            aligned_depth_info_pub_,
            color_info_msg_,
            aligned_stamp);
      }
#else
      rs2::frameset processed = align_to_color_->process(fs);
      if (auto aligned = processed.get_depth_frame())
        publish_video_frame(
          aligned,
          aligned_depth_image_pub_,
          aligned_depth_info_pub_,
          color_info_msg_,
          aligned_stamp);
#endif
    } catch (const std::exception & e) {
      if (running_) RCLCPP_WARN(get_logger(), "Align worker error: %s", e.what());
    }
  }
}

// ---------------------------------------------------------------------------
// publish_imu_data
// ---------------------------------------------------------------------------
void RealsenseCameraNode::publish_imu_data(const rs2::motion_frame& accel, const rs2::motion_frame& gyro)
{
  auto msg = std::make_unique<sensor_msgs::msg::Imu>();
  msg->header.stamp = frame_ros_time(gyro);
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
// sensor_time_to_ros / frame_ros_time
// ---------------------------------------------------------------------------
rclcpp::Time RealsenseCameraNode::sensor_time_to_ros(
  double frame_time_ms, rs2_timestamp_domain time_domain)
{
  if (frame_time_ms <= 0.0) {
    return now();
  }

  if (time_domain == RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME ||
      time_domain == RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME)
  {
    return rclcpp::Time(
      static_cast<int64_t>(frame_time_ms * 1000000.0),
      RCL_SYSTEM_TIME);
  }

  const auto ros_now = now();
  std::lock_guard<std::mutex> lock(timestamp_base_mutex_);
  if (!timestamp_base_initialized_ ||
      time_domain != timestamp_domain_ ||
      frame_time_ms < camera_time_base_ms_)
  {
    camera_time_base_ms_ = frame_time_ms;
    ros_time_base_ = ros_now;
    timestamp_domain_ = time_domain;
    timestamp_base_initialized_ = true;
  }

  const int64_t delta_ns =
    static_cast<int64_t>((frame_time_ms - camera_time_base_ms_) * 1000000.0);
  return ros_time_base_ + rclcpp::Duration::from_nanoseconds(delta_ns);
}

rclcpp::Time RealsenseCameraNode::frame_ros_time(const rs2::frame & frame)
{
  if (!frame) {
    return now();
  }

  return sensor_time_to_ros(frame.get_timestamp(), frame.get_frame_timestamp_domain());
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
  switch (i.model) {
    case RS2_DISTORTION_KANNALA_BRANDT4:
    case RS2_DISTORTION_FTHETA:
      info.distortion_model = "equidistant";
      info.d.assign(i.coeffs, i.coeffs + 4);
      break;
    case RS2_DISTORTION_NONE:
      info.distortion_model = "plumb_bob";
      info.d.assign(5, 0.0);
      break;
    default:
      info.distortion_model = "plumb_bob";
      info.d.assign(i.coeffs, i.coeffs + 5);
      break;
  }

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
  publish_video_frame(frame, pub, info_pub, info_template, frame_ros_time(frame));
}

void RealsenseCameraNode::publish_video_frame(
  const rs2::video_frame & frame,
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub,
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub,
  const sensor_msgs::msg::CameraInfo & info_template,
  const rclcpp::Time & stamp)
{
  if (!pub || !info_pub) return;

  auto img = std::make_unique<sensor_msgs::msg::Image>();
  img->header.stamp = stamp;
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
// publish_aligned_depth — publishes raw uint16_t buffer as Z16 Image
// Used by CUDA alignment path (no rs2::video_frame available).
// ---------------------------------------------------------------------------
void RealsenseCameraNode::publish_aligned_depth(
  const uint16_t * data, int width, int height, const rclcpp::Time & stamp)
{
  if (!aligned_depth_image_pub_ || !aligned_depth_info_pub_) return;

  auto img = std::make_unique<sensor_msgs::msg::Image>();
  img->header.stamp = stamp;
  img->header.frame_id = color_info_msg_.header.frame_id;
  img->width = static_cast<uint32_t>(width);
  img->height = static_cast<uint32_t>(height);
  img->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  img->step = static_cast<uint32_t>(width) * sizeof(uint16_t);

  const auto * raw = reinterpret_cast<const uint8_t *>(data);
  img->data.assign(raw, raw + img->height * img->step);

  auto info = std::make_unique<sensor_msgs::msg::CameraInfo>(color_info_msg_);
  info->header = img->header;

  aligned_depth_image_pub_->publish(std::move(img));
  aligned_depth_info_pub_->publish(std::move(info));
}

// ---------------------------------------------------------------------------
// publish_pose_frame
// ---------------------------------------------------------------------------
void RealsenseCameraNode::publish_pose_frame(const rs2::pose_frame & frame)
{
  if (!odom_pub_) return;

  auto pose = frame.get_pose_data();
  auto msg = std::make_unique<nav_msgs::msg::Odometry>();
  msg->header.stamp = frame_ros_time(frame);
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

  // Broadcast TF: odom_frame → pose_frame
  if (tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header = msg->header;
    tf.child_frame_id = msg->child_frame_id;
    tf.transform.translation.x = pose.translation.x;
    tf.transform.translation.y = pose.translation.y;
    tf.transform.translation.z = pose.translation.z;
    tf.transform.rotation.x = pose.rotation.x;
    tf.transform.rotation.y = pose.rotation.y;
    tf.transform.rotation.z = pose.rotation.z;
    tf.transform.rotation.w = pose.rotation.w;
    tf_broadcaster_->sendTransform(tf);
  }

  odom_pub_->publish(std::move(msg));
}

// ---------------------------------------------------------------------------
// Data-flow watchdog — detects frozen T265 (or any camera) pipeline
// ---------------------------------------------------------------------------
void RealsenseCameraNode::start_data_flow_watchdog()
{
  // Only arm the watchdog when hardware_reset is enabled (i.e. we can actually
  // recover) and we haven't already exhausted restart attempts.
  if (!enable_hardware_reset_ || pipeline_restart_attempts_ >= MAX_PIPELINE_RESTARTS) {
    return;
  }

  first_frame_received_.store(false, std::memory_order_relaxed);
  data_flow_watchdog_timer_ = create_wall_timer(
    std::chrono::seconds(DATA_FLOW_WATCHDOG_S),
    [this]() {
      // One-shot: cancel immediately so it doesn't re-fire.
      data_flow_watchdog_timer_->cancel();

      if (first_frame_received_.load(std::memory_order_relaxed)) {
        RCLCPP_INFO(get_logger(), "Data-flow watchdog: first frame received — all good.");
        return;
      }

      // No data arrived within the timeout window.
      ++pipeline_restart_attempts_;
      RCLCPP_WARN(get_logger(),
        "Data-flow watchdog: no frames received in %d s — restarting pipeline "
        "(attempt %d/%d)...",
        DATA_FLOW_WATCHDOG_S, pipeline_restart_attempts_, MAX_PIPELINE_RESTARTS);

      restart_pipeline_with_reset();
    });
}

void RealsenseCameraNode::restart_pipeline_with_reset()
{
  // Stop current pipeline
  running_ = false;

  // Wake and join the alignment worker so it doesn't race with the new pipeline.
  align_cv_.notify_all();
  if (align_thread_.joinable()) {
    align_thread_.join();
  }

  // Drain the alignment queue
  {
    std::lock_guard<std::mutex> lock(align_mutex_);
    decltype(align_queue_) empty;
    std::swap(align_queue_, empty);
  }

  // Stop IMU sensor if running
  if (imu_sensor_) {
    try { imu_sensor_.stop(); imu_sensor_.close(); } catch (...) {}
    imu_sensor_ = rs2::sensor();
  }

  // Stop video pipeline
  try { pipe_.stop(); } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Pipeline stop during restart: %s", e.what());
  }

  // Reset timestamp base so next start picks up fresh timestamps
  {
    std::lock_guard<std::mutex> lock(timestamp_base_mutex_);
    timestamp_base_initialized_ = false;
  }

  // Perform hardware reset + re-enumeration
  try {
    ctx_ = rs2::context();
    pipe_ = rs2::pipeline(ctx_);
    cfg_ = rs2::config();
    reset_device();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Hardware reset during restart failed: %s", e.what());
    return;
  }

  // Restart pipeline
  try {
    start_pipeline();
    RCLCPP_INFO(get_logger(), "Pipeline restarted successfully after watchdog trigger.");
    // Re-arm watchdog for the new pipeline
    start_data_flow_watchdog();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Pipeline restart failed: %s — node needs manual intervention.", e.what());
  }
}

} // namespace realsense_camera_bringup

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<realsense_camera_bringup::RealsenseCameraNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
