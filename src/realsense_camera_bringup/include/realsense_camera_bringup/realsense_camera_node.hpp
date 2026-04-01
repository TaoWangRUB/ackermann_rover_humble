#ifndef REALSENSE_CAMERA_BINGUP_REALSENSE_CAMERA_NODE_HPP
#define REALSENSE_CAMERA_BINGUP_REALSENSE_CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <librealsense2/rs.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <memory>

#ifdef HAVE_CUDA
#include "realsense_camera_bringup/cuda_align.hpp"
#endif

namespace realsense_camera_bringup
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
  void apply_sensor_options(rs2::pipeline_profile & profile);
  void align_worker();

  sensor_msgs::msg::CameraInfo build_camera_info(const rs2::video_stream_profile& profile, const std::string& frame_id) const;
  static bool parse_profile_string(const std::string & profile_str, int & width, int & height, int & fps);

  void publish_video_frame(const rs2::video_frame& frame, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub, const sensor_msgs::msg::CameraInfo& info_template);
  void publish_aligned_depth(const uint16_t * data, int width, int height);
  void publish_imu_data(const rs2::motion_frame& accel, const rs2::motion_frame& gyro);
  void publish_pose_frame(const rs2::pose_frame& frame);

  // Realsense Core
  rs2::context ctx_;
  rs2::pipeline pipe_;
  rs2::config cfg_;
  std::shared_ptr<rs2::align> align_to_color_;

  // Parameters — identity
  std::string camera_name_, serial_no_;
  std::string camera_namespace_, device_type_;
  CameraModel camera_model_;

  // Parameters — streams
  bool enable_color_, enable_depth_, enable_imu_;
  bool enable_accel_, enable_gyro_;
  bool enable_infra1_, enable_infra2_;
  bool enable_fisheye_;
  int color_width_, color_height_, color_fps_;
  int depth_width_, depth_height_, depth_fps_;
  int unite_imu_method_; // 1=copy, 2=linear_interpolation

  // Parameters — sync / alignment / TF / init
  bool enable_sync_, align_depth_enable_;
  bool publish_tf_;
  bool enable_hardware_reset_;

  // Parameters — RGB sensor controls
  std::string rgb_color_profile_;
  int rgb_exposure_, rgb_gain_;
  bool rgb_auto_exposure_;

  // Parameters — depth sensor controls
  std::string depth_profile_str_;
  int depth_exposure_, depth_gain_;
  bool depth_auto_exposure_;

  // Publishers — video
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_pub_, depth_image_pub_, fisheye1_image_pub_, fisheye2_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_, depth_info_pub_, fisheye1_info_pub_, fisheye2_info_pub_;

  // Publishers — infrared
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr infra1_image_pub_, infra2_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infra1_info_pub_, infra2_info_pub_;

  // Publishers — aligned depth
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr aligned_depth_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr aligned_depth_info_pub_;

  // Publishers — IMU / odom
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Cached CameraInfo
  sensor_msgs::msg::CameraInfo color_info_msg_, depth_info_msg_, fisheye1_info_msg_, fisheye2_info_msg_;
  sensor_msgs::msg::CameraInfo infra1_info_msg_, infra2_info_msg_;

  // Async alignment worker (offloads depth alignment off the pipeline callback)
  std::queue<rs2::frameset> align_queue_;
  std::mutex align_mutex_;
  std::condition_variable align_cv_;
  std::thread align_thread_;
  static constexpr size_t ALIGN_QUEUE_MAX = 2;  // drop oldest if consumer falls behind

#ifdef HAVE_CUDA
  std::unique_ptr<CudaAligner> cuda_aligner_;
  std::vector<uint16_t> aligned_depth_buf_;  // host buffer for CUDA output
#endif

  // Shutdown flag + cached accel frame for IMU pairing
  std::atomic<bool> running_{false};
  rs2::frame last_accel_frame_;
  rs2::sensor imu_sensor_;  // direct motion sensor (separate from pipeline)
};

} // namespace realsense_camera_bringup

#endif
