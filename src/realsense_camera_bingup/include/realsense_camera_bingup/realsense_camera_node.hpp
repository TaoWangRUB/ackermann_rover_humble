#pragma once

#include <atomic>
#include <string>
#include <thread>

#include <librealsense2/rs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace realsense_camera_bingup
{

// Supported camera models
enum class CameraModel { D435I, L515, T265 };

class RealsenseCameraNode : public rclcpp::Node
{
public:
  explicit RealsenseCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RealsenseCameraNode();

private:
  // --- Parameters ---
  std::string camera_name_;
  std::string serial_no_;
  CameraModel camera_model_;
  // D435i / L515
  bool enable_color_;
  bool enable_depth_;
  bool enable_imu_;        // D435i only
  int color_width_, color_height_, color_fps_;
  int depth_width_,  depth_height_,  depth_fps_;
  bool align_depth_to_color_;
  // T265
  int fisheye_fps_;

  // --- Publishers: D435i / L515 ---
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      color_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      depth_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr        imu_pub_;

  // --- Publishers: T265 ---
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      fisheye1_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr fisheye1_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      fisheye2_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr fisheye2_info_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr      odom_pub_;

  // --- RealSense pipeline ---
  rs2::pipeline pipe_;
  rs2::align    align_to_color_{RS2_STREAM_COLOR};

  // --- Cached camera_info (filled after pipeline start) ---
  sensor_msgs::msg::CameraInfo color_info_msg_;
  sensor_msgs::msg::CameraInfo depth_info_msg_;
  sensor_msgs::msg::CameraInfo fisheye1_info_msg_;
  sensor_msgs::msg::CameraInfo fisheye2_info_msg_;

  // --- Capture thread ---
  std::thread          capture_thread_;
  std::atomic<bool>    running_{false};

  // --- Helpers ---
  void declare_parameters();
  void create_publishers();
  void start_pipeline();
  void capture_loop();

  void process_depth_camera_frames(const rs2::frameset & frames);
  void process_t265_frames(const rs2::frameset & frames);

  sensor_msgs::msg::CameraInfo build_camera_info(
    const rs2::video_stream_profile & profile,
    const std::string & frame_id) const;

  void publish_color_frame(const rs2::video_frame & frame);
  void publish_depth_frame(const rs2::depth_frame & frame);
  void publish_imu_frame(const rs2::motion_frame & accel, const rs2::motion_frame & gyro);
  void publish_fisheye_frame(const rs2::video_frame & frame, int index,
                             const sensor_msgs::msg::CameraInfo & info_template);
  void publish_pose_frame(const rs2::pose_frame & frame);
};

}  // namespace realsense_camera_bingup
