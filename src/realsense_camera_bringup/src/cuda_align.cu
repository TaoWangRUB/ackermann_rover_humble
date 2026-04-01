#include "realsense_camera_bringup/cuda_align.hpp"
#include <cuda_runtime.h>
#include <stdexcept>
#include <string>

namespace realsense_camera_bringup
{

#define CUDA_CHECK(call) \
  do { \
    cudaError_t err = (call); \
    if (err != cudaSuccess) { \
      throw std::runtime_error( \
        std::string("CUDA error in ") + __FILE__ + ":" + \
        std::to_string(__LINE__) + " — " + cudaGetErrorString(err)); \
    } \
  } while (0)

// ---------------------------------------------------------------------------
// Kernel: for each depth pixel, deproject → transform → project into color grid.
// Uses atomicMin on uint32_t to resolve conflicts (closest depth wins).
// ---------------------------------------------------------------------------
__global__ void align_depth_to_color_kernel(
    const uint16_t * __restrict__ depth_in,
    uint32_t * __restrict__ aligned_out,
    int depth_w, int depth_h,
    int color_w, int color_h,
    float d_fx, float d_fy, float d_ppx, float d_ppy,
    float c_fx, float c_fy, float c_ppx, float c_ppy,
    const float * __restrict__ rot,
    const float * __restrict__ trans,
    float depth_scale)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= depth_w * depth_h) return;

  uint16_t d = depth_in[idx];
  if (d == 0) return;

  int du = idx % depth_w;
  int dv = idx / depth_w;
  float z = static_cast<float>(d) * depth_scale;

  // Deproject depth pixel to 3D point (depth frame)
  float x = (static_cast<float>(du) - d_ppx) / d_fx * z;
  float y = (static_cast<float>(dv) - d_ppy) / d_fy * z;

  // Transform depth frame → color frame
  float tx = rot[0] * x + rot[1] * y + rot[2] * z + trans[0];
  float ty = rot[3] * x + rot[4] * y + rot[5] * z + trans[1];
  float tz = rot[6] * x + rot[7] * y + rot[8] * z + trans[2];

  if (tz <= 0.0f) return;

  // Project into color image
  int cu = __float2int_rn(tx / tz * c_fx + c_ppx);
  int cv = __float2int_rn(ty / tz * c_fy + c_ppy);

  if (cu >= 0 && cu < color_w && cv >= 0 && cv < color_h) {
    atomicMin(&aligned_out[cv * color_w + cu], static_cast<uint32_t>(d));
  }
}

// ---------------------------------------------------------------------------
// Kernel: convert uint32_t → uint16_t, replacing sentinel (>= 0xFFFF) with 0.
// ---------------------------------------------------------------------------
__global__ void pack_u32_to_u16_kernel(
    const uint32_t * __restrict__ src,
    uint16_t * __restrict__ dst,
    int n)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= n) return;
  uint32_t v = src[idx];
  dst[idx] = (v >= 0xFFFFu) ? static_cast<uint16_t>(0) : static_cast<uint16_t>(v);
}

// ---------------------------------------------------------------------------
// CudaAligner implementation
// ---------------------------------------------------------------------------

CudaAligner::CudaAligner(
    const CameraIntrinsics & depth_intrin,
    const CameraIntrinsics & color_intrin,
    const DepthColorExtrinsics & depth_to_color,
    float depth_scale)
: depth_intrin_(depth_intrin),
  color_intrin_(color_intrin),
  extrinsics_(depth_to_color),
  depth_scale_(depth_scale)
{
  size_t depth_pixels = static_cast<size_t>(depth_intrin.width) * depth_intrin.height;
  size_t color_pixels = static_cast<size_t>(color_intrin.width) * color_intrin.height;

  CUDA_CHECK(cudaMalloc(&d_depth_in_,    depth_pixels * sizeof(uint16_t)));
  CUDA_CHECK(cudaMalloc(&d_aligned_u32_, color_pixels * sizeof(uint32_t)));
  CUDA_CHECK(cudaMalloc(&d_aligned_out_, color_pixels * sizeof(uint16_t)));
  CUDA_CHECK(cudaMalloc(&d_rotation_,    9 * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_translation_, 3 * sizeof(float)));

  // Upload static extrinsics (constant across frames)
  CUDA_CHECK(cudaMemcpy(d_rotation_,    extrinsics_.rotation,    9 * sizeof(float), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_translation_, extrinsics_.translation, 3 * sizeof(float), cudaMemcpyHostToDevice));
}

CudaAligner::~CudaAligner()
{
  cudaFree(d_depth_in_);
  cudaFree(d_aligned_u32_);
  cudaFree(d_aligned_out_);
  cudaFree(d_rotation_);
  cudaFree(d_translation_);
}

void CudaAligner::align(const uint16_t * depth_in, uint16_t * aligned_out)
{
  int depth_pixels = depth_intrin_.width * depth_intrin_.height;
  int color_pixels = color_intrin_.width * color_intrin_.height;

  // Upload depth frame to GPU
  CUDA_CHECK(cudaMemcpy(d_depth_in_, depth_in,
    depth_pixels * sizeof(uint16_t), cudaMemcpyHostToDevice));

  // Clear intermediate buffer to sentinel (0xFFFFFFFF) so atomicMin works
  CUDA_CHECK(cudaMemset(d_aligned_u32_, 0xFF, color_pixels * sizeof(uint32_t)));

  // Launch alignment kernel
  constexpr int BLOCK = 256;
  int grid = (depth_pixels + BLOCK - 1) / BLOCK;

  align_depth_to_color_kernel<<<grid, BLOCK>>>(
    d_depth_in_, d_aligned_u32_,
    depth_intrin_.width, depth_intrin_.height,
    color_intrin_.width, color_intrin_.height,
    depth_intrin_.fx, depth_intrin_.fy, depth_intrin_.ppx, depth_intrin_.ppy,
    color_intrin_.fx, color_intrin_.fy, color_intrin_.ppx, color_intrin_.ppy,
    d_rotation_, d_translation_,
    depth_scale_);

  // Convert uint32 → uint16, sentinel → 0
  int grid2 = (color_pixels + BLOCK - 1) / BLOCK;
  pack_u32_to_u16_kernel<<<grid2, BLOCK>>>(d_aligned_u32_, d_aligned_out_, color_pixels);

  // Download result
  CUDA_CHECK(cudaDeviceSynchronize());
  CUDA_CHECK(cudaMemcpy(aligned_out, d_aligned_out_,
    color_pixels * sizeof(uint16_t), cudaMemcpyDeviceToHost));
}

} // namespace realsense_camera_bringup
