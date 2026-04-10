#ifndef REALSENSE_CAMERA_BRINGUP_CUDA_ALIGN_HPP_
#define REALSENSE_CAMERA_BRINGUP_CUDA_ALIGN_HPP_

#include <cstdint>

namespace realsense_camera_bringup
{

struct CameraIntrinsics {
  float fx, fy, ppx, ppy;
  int width, height;
};

struct DepthColorExtrinsics {
  float rotation[9];
  float translation[3];
};

/// GPU-accelerated depth-to-color alignment.
/// Replaces rs2::align::process() which takes ~250ms on Jetson ARM at 640x480.
/// CUDA kernel processes all pixels in parallel — typically <3ms on Jetson.
class CudaAligner
{
public:
  CudaAligner(const CameraIntrinsics & depth_intrin,
              const CameraIntrinsics & color_intrin,
              const DepthColorExtrinsics & depth_to_color,
              float depth_scale);
  ~CudaAligner();

  CudaAligner(const CudaAligner &) = delete;
  CudaAligner & operator=(const CudaAligner &) = delete;

  /// Align depth frame to color frame.
  /// @param depth_in  Raw Z16 depth data (depth_width * depth_height)
  /// @param aligned_out  Output Z16 buffer (color_width * color_height), caller-allocated
  void align(const uint16_t * depth_in, uint16_t * aligned_out);

  int color_width() const { return color_intrin_.width; }
  int color_height() const { return color_intrin_.height; }

private:
  CameraIntrinsics depth_intrin_, color_intrin_;
  DepthColorExtrinsics extrinsics_;
  float depth_scale_;

  // Device buffers
  uint16_t * d_depth_in_ = nullptr;
  uint32_t * d_aligned_u32_ = nullptr;   // 32-bit for native atomicMin
  uint16_t * d_aligned_out_ = nullptr;   // final uint16 output
  float * d_rotation_ = nullptr;
  float * d_translation_ = nullptr;
};

} // namespace realsense_camera_bringup

#endif
