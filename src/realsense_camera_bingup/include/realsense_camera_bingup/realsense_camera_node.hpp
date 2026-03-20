#ifndef REALSENSE_CAMERA_BINGUP_REALSENSE_CAMERA_NODE_HPP
#define REALSENSE_CAMERA_BINGUP_REALSENSE_CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <librealsense2/rs.hpp>
#include <thread>
#include <mutex>

namespace realsense_camera_bingup
{

enum class CameraModel { D435I, L515, T265 };

class RealsenseCameraNode : public rclcpp::Node
{
public:
  explicit RealsenseCameraNode(const rclcpp::NodeOptions & options);
  ~RealsenseCameraNode();

private:
  void declare_parameters();
  void create_publishers();
  void reset_device();
  void start_pipeline();
  void start_imu_sensor(rs2::device device);
  void on_frame(rs2::frame frame);
  
  sensor_msgs::msg::CameraInfo build_camera_info(const rs2::video_stream_profile& profile, const std::string& frame_id) const;
  
  void publish_video_frame(const rs2::video_frame& frame, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub, const sensor_msgs::msg::CameraInfo& info_template);
  void publish_imu_data(const rs2::motion_frame& accel, const rs2::motion_frame& gyro);
  void publish_pose_frame(const rs2::pose_frame& frame);

  // Realsense Core
  rs2::context ctx_;
  rs2::pipeline pipe_;
  rs2::config cfg_;
  
  // Parameters
  std::string camera_name_, serial_no_;
  CameraModel camera_model_;
  bool enable_color_, enable_depth_, enable_imu_;
  int color_width_, color_height_, color_fps_;
  int depth_width_, depth_height_, depth_fps_;
  int unite_imu_method_; // 1=copy, 2=linear_interpolation

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_pub_, depth_image_pub_, fisheye1_image_pub_, fisheye2_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_, depth_info_pub_, fisheye1_info_pub_, fisheye2_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // Cached Infos
  sensor_msgs::msg::CameraInfo color_info_msg_, depth_info_msg_, fisheye1_info_msg_, fisheye2_info_msg_;

  // Shutdown flag + cached accel frame for IMU pairing
  std::atomic<bool> running_{false};
  rs2::frame last_accel_frame_;
  rs2::sensor imu_sensor_;  // direct motion sensor (separate from pipeline)
};

} // namespace realsense_camera_bingup

#endif